#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import serial
import struct
import math
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class CatheterBridge(Node):
    def __init__(self):
        super().__init__('catheter_bridge')

        # --- SERİ PORT AYARLARI ---
        serial_port = '/dev/ttyUSB0' 
        baud_rate = 115200
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Seri port baglantisi kuruldu: {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Port acilamadi: {str(e)}")
            raise e

        # --- PARAMETRELER ---
        self.v_min, self.v_max = 0.015, 0.045
        self.r_min, self.r_max = 1.5, 2.7
        self.visual_x_scale = 10.0
        
        # YARIÇAP ÇARPANI: Eğer damar hala ince gelirse bu 5.0 değerini artır (örn: 10.0 yap)
        self.radius_multiplier = 30.0 
        # Çizim hassasiyeti (Segment boyu)
        self.DRAW_THRESHOLD = 0.005 * self.visual_x_scale # 1cm adımlarla çiz

        self.colors = [
            (1.0, 0.0, 0.0), (1.0, 0.4, 0.0), (1.0, 1.0, 0.0), (0.5, 1.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, 1.0, 0.5), (0.0, 1.0, 1.0), (0.0, 0.5, 1.0), (0.0, 0.0, 1.0), (0.4, 0.0, 1.0),
            (0.7, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 0.0, 0.5), (0.6, 0.6, 0.6), (1.0, 1.0, 1.0)
        ]
        self.num_bins = len(self.colors)
        self.bin_width = (self.r_max - self.r_min) / self.num_bins
        self.P_METRE = (math.pi * 3.0 / 100.0) / (360 / 0.4)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._send_static_tf()

        self.ilk_enc = None
        self.vessel_segments = [] 
        self.last_x_vis = 0.0
        self.marker_id_counter = 0
        self.leftover_data = b''
        self.filt_r = self.r_max
        self.r_accumulator = []
        
        self._publish_legend()

    def _get_color_and_index(self, r_val):
        val = max(self.r_min, min(self.r_max - 0.0001, r_val))
        idx = int((val - self.r_min) / self.bin_width)
        return self.colors[idx]

    def _publish_legend(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i in range(self.num_bins):
            low = self.r_min + (i * self.bin_width); high = low + self.bin_width
            col = self.colors[i]
            m_c = Marker(header=Header(stamp=now, frame_id='odom'), ns="legend_cube", id=i, type=Marker.CUBE, action=Marker.ADD)
            m_c.pose.position.x, m_c.pose.position.y, m_c.pose.position.z = -0.5, -1.5, 0.5 + (i*0.1)
            m_c.scale.x = m_c.scale.y = m_c.scale.z = 0.08
            m_c.color.r, m_c.color.g, m_c.color.b, m_c.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
            ma.markers.append(m_c)
            m_t = Marker(header=Header(stamp=now, frame_id='odom'), ns="legend_text", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            m_t.pose.position.x, m_t.pose.position.y, m_t.pose.position.z = -0.1, -1.5, 0.5 + (i*0.1)
            m_t.scale.z = 0.06; m_t.color.a = 1.0; m_t.text = f"{low:.2f}-{high:.2f}mm"
            ma.markers.append(m_t)
        self.marker_pub.publish(ma)

    def _voltage_to_radius(self, v_raw):
        v = max(self.v_min, min(self.v_max, v_raw))
        norm = (v - self.v_min) / (self.v_max - self.v_min)
        target_r = self.r_max - norm * (self.r_max - self.r_min)
        self.filt_r = self.filt_r * 0.8 + target_r * 0.2
        return self.filt_r

    def run(self):
        self.get_logger().info("Sistem aktif...")
        while rclpy.ok():
            waiting = self.ser.in_waiting
            if waiting < 6:
                rclpy.spin_once(self, timeout_sec=0.001); continue

            chunk = self.ser.read(waiting)
            if self.leftover_data: chunk = self.leftover_data + chunk
            full_packets = (len(chunk) // 6) * 6
            data = chunk[:full_packets]
            self.leftover_data = chunk[full_packets:]

            now = self.get_clock().now().to_msg()
            loop_marker_array = MarkerArray()

            for adc_raw, enc_raw in struct.iter_unpack('>Hi', data):
                v_raw = abs((adc_raw * 3.3 / 4095.0))
                current_r = self._voltage_to_radius(v_raw)
                self.r_accumulator.append(current_r)
                
                if self.ilk_enc is None:
                    self.ilk_enc = enc_raw; continue
                
                latest_x = (enc_raw - self.ilk_enc) * self.P_METRE * self.visual_x_scale

                # --- 1. GERİ ÇEKME ---
                while self.vessel_segments and self.vessel_segments[-1][2] > (latest_x + 0.0001):
                    m_id, _, _, _ = self.vessel_segments.pop()
                    dm = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=int(m_id), action=Marker.DELETE)
                    loop_marker_array.markers.append(dm)
                    self.r_accumulator = []

                # --- 2. İLERİ HAREKET (DOLGULU) ---
                if latest_x > self.last_x_vis:
                    v_tip = self.vessel_segments[-1][2] if self.vessel_segments else 0.0
                    while (latest_x - v_tip) >= self.DRAW_THRESHOLD:
                        next_v_tip = v_tip + self.DRAW_THRESHOLD
                        new_marker = self._create_segment_marker(next_v_tip, v_tip, now)
                        loop_marker_array.markers.append(new_marker)
                        v_tip = next_v_tip

                self.last_x_vis = latest_x

            if loop_marker_array.markers:
                self.marker_pub.publish(loop_marker_array)

            self._publish_fast_tf(self.last_x_vis, now)
            rclpy.spin_once(self, timeout_sec=0)

    def _create_segment_marker(self, target_end_x, start_x, now):
        self.marker_id_counter += 1
        avg_r = sum(self.r_accumulator) / len(self.r_accumulator) if self.r_accumulator else self.filt_r
        col = self._get_color_and_index(avg_r)
        
        m = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=self.marker_id_counter, type=Marker.CYLINDER, action=Marker.ADD)
        m.pose.position.x = float((start_x + target_end_x) / 2.0)
        m.pose.orientation.y = m.pose.orientation.w = 0.7071
        
        # --- YARIÇAP DÜZELTMESİ ---
        # radius_multiplier ekledik ki damar daha kalın görünsün
        radius_val = float(avg_r * self.visual_x_scale * self.radius_multiplier / 1000.0)
        m.scale.x = m.scale.y = radius_val
        
        # --- BOŞLUK (Z-FIGHTING) DÜZELTMESİ ---
        # Bindirmeyi %20 yaptık (1.2), böylece silindirler birbirinin içine geçer, boşluk kalmaz.
        m.scale.z = float(abs(target_end_x - start_x) * 1.2)
        
        m.color.r, m.color.g, m.color.b, m.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
        
        self.vessel_segments.append((self.marker_id_counter, start_x, target_end_x, avg_r))
        self.r_accumulator = []
        return m

    def _send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _publish_fast_tf(self, x_vis, now):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
        t.transform.translation.x, t.transform.rotation.w = float(x_vis), 1.0
        self.tf_broadcaster.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.orientation.w = float(x_vis), 1.0
        self.odom_pub.publish(o)

def main():
    rclpy.init()
    node = CatheterBridge()
    try: node.run()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()