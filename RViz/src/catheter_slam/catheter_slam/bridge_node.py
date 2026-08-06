#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header
import serial
import struct
import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
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
            self.get_logger().info(f"Seri port aktif: {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Port hatasi: {e}"); raise e

        # --- PARAMETRELER ---
        self.anlik_konum = 0
        self.v_min, self.v_max = 1.2, 2.5
        self.r_min = 0.0075  # En Dar (Kırmızı)
        self.r_max = 0.0225   # En Geniş (Beyaz)
        
        self.visual_x_scale = 25.0
        self.radius_multiplier = 40.0 
        self.sensitivity = 1  # 1: Doğrusal (Gerçekçi), >1: Daralmaları vurgular
        self.DRAW_THRESHOLD = 0.0002 * self.visual_x_scale 

        # 15 Renk Tanımı
        self.colors = [(1,0,0),(1,0.4,0),(1,1,0),(0.5,1,0),(0,1,0),(0,1,0.5),(0,1,1),(0,0.5,1),(0,0,1),(0.4,0,1),(0.7,0,1),(1,0,1),(1,0,0.5),(0.6,0.6,0.6),(1,1,1)]
        
        # --- ENCODER FİZİKSEL HESAPLAMA ---
        WHEEL_RADIUS_M = 0.015 # (m cinsinden)
        PPR = 800.0            # Pulse Per Revolution
        self.P_METRE = (2 * math.pi * WHEEL_RADIUS_M) / PPR

        # QoS ve Publishers
        marker_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', marker_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._send_static_tf()

        # Durum Takibi
        self.prev_enc = None
        self.enc_temp = None 
        self.current_x_vis = 0.0
        self.vessel_segments = [] 
        self.marker_id_counter = 0
        self.leftover_data = b''
        
        self.create_timer(2.0, self._publish_legend)

    def _get_smooth_color(self, r_val):
        """r_min (Kırmızı) -> r_max (Beyaz) arası geçiş sağlar."""
        norm = np.clip((r_val - self.r_min) / (self.r_max - self.r_min), 0.0, 1.0)
        float_idx = norm * (len(self.colors) - 1)
        idx1 = int(float_idx)
        idx2 = min(idx1 + 1, len(self.colors) - 1)
        w = float_idx - idx1
        c1, c2 = self.colors[idx1], self.colors[idx2]
        return (c1[0]*(1-w)+c2[0]*w, c1[1]*(1-w)+c2[1]*w, c1[2]*(1-w)+c2[2]*w, 0.8)

    def _voltage_to_radius(self, v):
        """
        TERS LİNEER EŞLEME (Entegre Edilen Mantık):
        Küçük Vpp -> Geniş damar (r_max)
        Büyük Vpp -> Dar damar (r_min)
        """
        if self.v_min is None or self.v_max is None:
            return float(self.r_max)

        if abs(self.v_max - self.v_min) < 1e-9:
            return float(self.r_min)

        # Voltajı v_min ve v_max arasına sınırla
        v_clipped = np.clip(v, self.v_min, self.v_max)

        # Normalizasyon (0.0 - 1.0 arası)
        norm = (v_clipped - self.v_min) / (self.v_max - self.v_min)
        
        if self.sensitivity != 1:
            norm = math.pow(norm, self.sensitivity)

        # Ters eşleme: norm=0 (v_min) -> r_max, norm=1 (v_max) -> r_min
        r = self.r_max - norm * (self.r_max - self.r_min)

        return float(r)

    def run(self):
        self.get_logger().info("Sistem Aktif")
        while rclpy.ok():
            waiting = self.ser.in_waiting
            if waiting < 6:
                rclpy.spin_once(self, timeout_sec=0.001); continue

            data_chunk = self.ser.read(waiting)
            if self.leftover_data: data_chunk = self.leftover_data + data_chunk
            full_packets = (len(data_chunk)//6)*6
            data = data_chunk[:full_packets]
            self.leftover_data = data_chunk[full_packets:]

            now = self.get_clock().now().to_msg()
            loop_markers = MarkerArray()

            for adc_raw, enc_raw in struct.iter_unpack('>Hi', data):
                v_raw = abs((adc_raw * 3.3 / 4095.0))
                current_r = self._voltage_to_radius(v_raw)
                
                if self.prev_enc is None:
                    self.prev_enc = enc_raw
                    self.enc_temp = enc_raw
                    continue
                
                delta = enc_raw - self.prev_enc
                self.prev_enc = enc_raw
                self.current_x_vis += delta * self.P_METRE * self.visual_x_scale
                lx = self.current_x_vis
                self.anlik_konum += delta * self.P_METRE
                print(self.anlik_konum * 100)

                # GERİ HAREKET ŞARTI
                if (self.prev_enc < self.enc_temp):
                    while self.vessel_segments and self.vessel_segments[-1][2] > (lx + 0.0001):
                        mid, _, _, _ = self.vessel_segments.pop()
                        dm = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=int(mid), action=Marker.DELETE)
                        loop_markers.markers.append(dm)

                # İLERİ HAREKET ŞARTI
                elif (self.prev_enc > self.enc_temp):
                    v_tip = self.vessel_segments[-1][2] if self.vessel_segments else 0.0
                    if (lx - v_tip) >= self.DRAW_THRESHOLD:
                        m = self._create_marker(lx, v_tip, now, current_r)
                        loop_markers.markers.append(m)

                self.enc_temp = enc_raw

            if loop_markers.markers: self.marker_pub.publish(loop_markers)
            
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
            t.transform.translation.x = float(self.current_x_vis)
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            
            rclpy.spin_once(self, timeout_sec=0)

    def _create_marker(self, target_x, start_x, now, r):
        self.marker_id_counter += 1
        red, g, b, a = self._get_smooth_color(r)
        m = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=self.marker_id_counter, type=Marker.CYLINDER, action=Marker.ADD)
        m.pose.position.x = float((start_x + target_x) / 2.0)
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.y = m.pose.orientation.w = 0.7071
        
        diameter = float(r * 2.0 * self.radius_multiplier)
        m.scale.x = m.scale.y = diameter
        m.scale.z = float(abs(target_x - start_x) * 1.02)
        
        m.color.r, m.color.g, m.color.b, m.color.a = red, g, b, a
        self.vessel_segments.append((self.marker_id_counter, start_x, target_x, r))
        return m

    def _publish_legend(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        step = (self.r_max - self.r_min) / 14.0
        for i in range(15):
            val = self.r_min + (i * step)
            col = self.colors[i]
            m_c = Marker(header=Header(stamp=now, frame_id='odom'), ns="l_c", id=i, type=Marker.CUBE, action=Marker.ADD)
            m_c.pose.position.x, m_c.pose.position.y, m_c.pose.position.z = -0.5, -1.5, 0.5 + (i*0.1)
            m_c.scale.x = m_c.scale.y = m_c.scale.z = 0.08
            m_c.color.r, m_c.color.g, m_c.color.b, m_c.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
            ma.markers.append(m_c)
            m_t = Marker(header=Header(stamp=now, frame_id='odom'), ns="l_t", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            m_t.pose.position.x, m_t.pose.position.y, m_t.pose.position.z = -0.1, -1.5, 0.5 + (i*0.1)
            m_t.scale.z = 0.06; m_t.color.a = 1.0; m_t.text = f"{val:.4f}mm"
            ma.markers.append(m_t)
        self.marker_pub.publish(ma)

    def _send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

def main():
    rclpy.init(); node = CatheterBridge()
    try: node.run()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()