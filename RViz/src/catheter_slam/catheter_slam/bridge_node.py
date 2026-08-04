#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import serial
import struct
import numpy as np
import math
import time
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan

class CatheterBridge(Node):
    def __init__(self):
        super().__init__('catheter_bridge')

        serial_port = '/dev/ttyUSB1' 
        baud_rate = 115200

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Seri port baglantisi kuruldu: {serial_port} @ {baud_rate}")
        except Exception as e:
            self.get_logger().error(f"Seri port acilamadi ({serial_port}): {str(e)}")
            raise e

        # ── Publishers ───────────────────────────────────────────
        self.scan_pub   = self.create_publisher(LaserScan,   '/scan',           1000)
        self.odom_pub   = self.create_publisher(Odometry,    '/odom',           1000)
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', 10000)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._send_static_tf()

        # --- PARAMETRELER ---
        # Voltaj sınırlarınızı buradan belirleyin (Dinamik kalibrasyon kapatıldı)
        self.v_min, self.v_max = 0.15, 1.4  
        self.r_min, self.r_max = 0.015, 0.05
        self.visual_x_scale = 10.0
        self.thickness_color_step = (self.r_max - self.r_min) / 15.0
        self.DRAW_THRESHOLD = 0.0005 * self.visual_x_scale # 1.5mm cizim adimi
        
        # Encoder
        CAP_CM = 3.0
        ADIM_ACISI = 0.4
        self.P_METRE = (math.pi * CAP_CM / 100.0) / (360 / ADIM_ACISI)

        self.colors = [
            (1.0, 0.0, 0.0), (1.0, 0.4, 0.0), (1.0, 1.0, 0.0), (0.5, 1.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, 1.0, 0.5), (0.0, 1.0, 1.0), (0.0, 0.5, 1.0), (0.0, 0.0, 1.0), (0.4, 0.0, 1.0),
            (0.7, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 0.0, 0.5), (0.6, 0.6, 0.6), (1.0, 1.0, 1.0)
        ]

        # --- DURUM TAKİBİ ---
        self.ilk_enc = None
        self.vessel_segments = []           # [(id, start_x, end_x, r)]
        self.last_x_vis = 0.0
        self.marker_id_counter = 0
        self.leftover_data = b''

        # --- PURUZSUZLUK FILTRELERI ---
        self.filt_r = self.r_max
        self.max_r_step = 0.004             # Slew rate sınırı
        self.r_accumulator = []
        
        self._publish_legend()

    def _voltage_to_radius(self, v_raw):
        """
        TERS LİNEER EŞLEME:
        v_min (Düşük Voltaj) → r_max (Geniş Damar)
        v_max (Yüksek Voltaj) → r_min (Dar Damar)
        """
        # Voltajı belirlenen v_min ve v_max arasında sınırla (Clamp)
        v = max(self.v_min, min(self.v_max, v_raw))

        # Normalize et (0.0 ile 1.0 arasında)
        norm = (v - self.v_min) / (self.v_max - self.v_min)

        # Ters haritalama yap
        target_r = self.r_max - norm * (self.r_max - self.r_min)

        # Slew rate sınırlayıcı (ani zıplamaları önler)
        diff = target_r - self.filt_r
        if abs(diff) > self.max_r_step:
            target_r = self.filt_r + (self.max_r_step if diff > 0 else -self.max_r_step)

        self.filt_r = target_r
        return float(self.filt_r)

    def _send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _publish_legend(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, col in enumerate(self.colors):
            r_val = self.r_min + (i * self.thickness_color_step)
            for t, ns in [(Marker.CUBE, "legend_c"), (Marker.TEXT_VIEW_FACING, "legend_t")]:
                m = Marker(header=Header(stamp=now, frame_id='odom'), ns=ns, id=i, type=t, action=Marker.ADD)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = -0.5, -1.0, 0.5 + (i*0.06)
                m.scale.x = m.scale.y = m.scale.z = 0.04
                m.color.r, m.color.g, m.color.b, m.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
                if t == Marker.TEXT_VIEW_FACING:
                    m.text = f"r: {r_val:.3f}m"
                    m.pose.position.x += 0.2
                    m.color.r = m.color.g = m.color.b = 1.0
                ma.markers.append(m)
        self.marker_pub.publish(ma)

    def run(self):
        self.get_logger().info("Akis basladi...")
        while rclpy.ok():
            waiting = self.ser.in_waiting
            if waiting < 6:
                rclpy.spin_once(self, timeout_sec=0.0001); continue

            chunk = self.ser.read(waiting)
            if self.leftover_data: chunk = self.leftover_data + chunk
            full_packets = (len(chunk) // 6) * 6
            data = chunk[:full_packets]
            self.leftover_data = chunk[full_packets:]

            if not data: continue
            now = self.get_clock().now().to_msg()
            latest_x = self.last_x_vis
            batch_ma = MarkerArray()

            for adc_raw, enc_raw in struct.iter_unpack('>Hi', data):
                # Voltaj hesabı (0 - 3.0V arası)
                v_raw = abs((adc_raw * 3.0 / 4095.0) - 1.5)
                
                r_inst = self._voltage_to_radius(v_raw)
                self.r_accumulator.append(r_inst)

                if self.ilk_enc is None:
                    self.ilk_enc = enc_raw; continue
                latest_x = (enc_raw - self.ilk_enc) * self.P_METRE * self.visual_x_scale

            # --- GERİ ÇEKİLME KONTROLÜ VE ZORLAMALI SİLME ---
            ids_to_remove = [i for i, seg in enumerate(self.vessel_segments) if seg[2] > latest_x + 0.001]
            
            if ids_to_remove:
                delete_index = ids_to_remove[0]
                to_delete = self.vessel_segments[delete_index:]
                self.vessel_segments = self.vessel_segments[:delete_index]
                
                for m_id, s_x, e_x, r_val in to_delete:
                    dm = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=int(m_id), action=Marker.DELETE)
                    batch_ma.markers.append(dm)
                
                self.r_accumulator = []

            # --- SADECE İLERİ HAREKETTE ÇİZME ---
            v_tip = self.vessel_segments[-1][2] if self.vessel_segments else 0.0
            dist = latest_x - v_tip
            
            if latest_x < v_tip:
                self.r_accumulator = []

            if dist >= self.DRAW_THRESHOLD:
                self.marker_id_counter += 1
                avg_r = sum(self.r_accumulator) / len(self.r_accumulator) if self.r_accumulator else self.filt_r
                
                m = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=self.marker_id_counter, type=Marker.CYLINDER, action=Marker.ADD)
                m.pose.position.x = float((v_tip + latest_x) / 2.0)
                m.pose.orientation.y = m.pose.orientation.w = 0.7071068
                m.scale.x = m.scale.y = float(avg_r * 2.0)
                m.scale.z = float(abs(latest_x - v_tip) * 1.05)
                
                # Renk indeksi hesaplama
                idx = int(np.floor((avg_r - self.r_min) / self.thickness_color_step))
                col = self.colors[max(0, min(idx, len(self.colors)-1))]
                m.color.r, m.color.g, m.color.b, m.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
                
                batch_ma.markers.append(m)
                self.vessel_segments.append((self.marker_id_counter, v_tip, latest_x, avg_r))
                self.r_accumulator = []

            if batch_ma.markers: self.marker_pub.publish(batch_ma)
            self._publish_fast_tf(latest_x, now)
            self.last_x_vis = latest_x
            rclpy.spin_once(self, timeout_sec=0)

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
    try:
        node.run()
    except KeyboardInterrupt: pass
    finally:
        if hasattr(node, 'ser'): node.ser.close()
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()