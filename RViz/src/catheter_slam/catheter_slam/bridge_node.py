#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header, String 
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import numpy as np
import sys
import select
import termios
import tty
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener

class CatheterBridge(Node):
    def __init__(self):
        super().__init__('catheter_bridge')

        # --- SERİ PORT AYARLARI ---
        serial_port = '/dev/ttyUSB0' 
        baud_rate = 115200
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.01)
            self.get_logger().info(f"Seri port aktif: {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Port hatasi: {e}"); raise e

        # --- PARAMETRELER (Metre cinsinden)---
        self.v_min, self.v_max = 1.7, 2.20
        self.r_min, self.r_max = 0.015, 0.025

        #(Slame işlem kolaylığı için bu çarpanları büyük tutun)
        # Hareket mesafesini büyüten çarpan
        self.slam_scale = 100.0 

        # Rvizde görsel çarpanı
        self.visual_scale = 25.0 

        self.draw_ratio = self.visual_scale / self.slam_scale
        
        self.colors = [(1,0,0),(1,0.4,0),(1,1,0),(0.5,1,0),(0,1,0),(0,1,0.5),(0,1,1),(0,0.5,1),(0,0,1),(0.4,0,1),(0.7,0,1),(1,0,1),(1,0,0.5),(0.6,0.6,0.6),(1,1,1)]
        
        WHEEL_RADIUS_M = 0.015 #(Metre cinsinden)
        PPR = 800.0
        self.P_METRE = (2 * math.pi * WHEEL_RADIUS_M) / PPR

        # --- KONTROL BAYRAKLARI ---
        self.noise_enabled = False

        # --- GÜRÜLTÜ ZAMANLAMASI ---
        self.noise_cycle_period = 5.0  # Toplam döngü süresi (saniye)
        self.noise_on_duration = 0.5   # Gürültünün açık kalacağı süre (saniye)
        # --- FİLTRE PARAMETRELERİ ---
        self.v_filtered = None
        self.v_alpha = 0.15 

        #self.cmd_sub = self.create_subscription(String, 'catheter_cmd', self._cmd_callback, 10) (Tuş ile kontrol)

        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            try: self.settings = termios.tcgetattr(sys.stdin)
            except Exception: self.is_tty = False

        # --- DURUM TAKİBİ ---
        self.prev_enc = None
        self.current_lx = 0.0     
        self.odom_x = 0.0          
        self.marker_id_counter = 0
        self.leftover_data = b''
        self.vessel_segments = [] 
        self.last_scan_publish_time = self.get_clock().now()

        # --- TF ALTYAPISI ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        marker_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', marker_qos)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', scan_qos)
        
        self._send_static_tf()
        self.create_timer(2.0, self._publish_legend_static)
        self.create_timer(0.1, self._publish_legend_dynamic)
        self.create_timer(0.4, self._republish_all_markers)

    #-- Tuş ile kontrol --
    #def _cmd_callback(self, msg):
    #    command = msg.data.lower()
    #    if command == 'n': self.noise_enabled = not self.noise_enabled

    #def _get_key(self):
    #    if not self.is_tty: return None
    #    tty.setraw(sys.stdin.fileno())
    #    key = sys.stdin.read(1)
    #    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    #    return key

    def run(self):
        while rclpy.ok():
            try:

                #-- Tuş ile kontrol --
                #if self.is_tty and select.select([sys.stdin], [], [], 0)[0]:
                #    key = self._get_key()
                #    if key == 'n': self.noise_enabled = not self.noise_enabled


                waiting = self.ser.in_waiting
                if waiting < 6:
                    rclpy.spin_once(self, timeout_sec=0.001); continue

                data = self.ser.read((waiting // 6) * 6)
                if self.leftover_data: data = self.leftover_data + data
                
                now_ros = self.get_clock().now()
                now_msg = now_ros.to_msg()
                now_sec = now_ros.nanoseconds / 1e9

                # Noise için zaman döngüsü hesaplama
                time_in_cycle = now_sec % self.noise_cycle_period

                is_noise_time = time_in_cycle < self.noise_on_duration #Noise süresinin açık durması

                for adc_raw, enc_raw in struct.iter_unpack('>Hi', data):
                    if self.prev_enc is None:
                        self.prev_enc = enc_raw; continue
                    
                    delta = enc_raw - self.prev_enc
                    
                    current_noise = 0.0
                    if self.noise_enabled and is_noise_time:
                        current_noise = abs(np.random.normal(0, 0.2))

                    self.odom_x += (delta + current_noise) * self.P_METRE * self.slam_scale
                    
                    self.prev_enc = enc_raw

                    v_raw = abs((adc_raw * 3.3 / 4095.0))
                    if self.v_filtered is None:
                        self.v_filtered = v_raw
                    else:
                        self.v_filtered = (self.v_alpha * v_raw) + ((1.0 - self.v_alpha) * self.v_filtered)
                    
                    v_clipped = np.clip(self.v_filtered, self.v_min, self.v_max)
                    norm = (v_clipped - self.v_min) / (self.v_max - self.v_min)
                    current_r = self.r_max - norm * (self.r_max - self.r_min)

                    # 1. TF Yayınla (Senkron)
                    t = TransformStamped()
                    t.header.stamp = now_msg
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = float(self.odom_x)
                    t.transform.rotation.w = 1.0
                    self.tf_broadcaster.sendTransform(t)

                    # 2. Scan Yayınla (Senkron)
                    if (now_ros - self.last_scan_publish_time).nanoseconds / 1e9 > 0.05: 
                        scan_msg = LaserScan()
                        scan_msg.header.stamp = self.get_clock().now().to_msg() 
                        scan_msg.header.frame_id = 'laser'
                        scan_msg.angle_min, scan_msg.angle_max = 0.0, 2.0 * math.pi
                        num_pts = 360
                        scan_msg.angle_increment = (2.0 * math.pi) / num_pts
                        scan_msg.range_min, scan_msg.range_max = 0.001, 100.0

                        #Slam verisini büyüt
                        r_slam = float(current_r * self.slam_scale)
                        scan_msg.ranges = [float(r_slam + np.random.uniform(-0.01, 0.01)) for _ in range(num_pts)]
                        
                        self.scan_pub.publish(scan_msg)
                        self.last_scan_publish_time = now_ros

                    # 3. SLAM Feedback
                    try:
                        trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                        slam_x = trans.transform.translation.x
                        self.current_lx = slam_x * self.draw_ratio
                    except:
                        self.current_lx = self.odom_x * self.draw_ratio

                    # Marker Güncelleme
                    for seg in self.vessel_segments:
                        seg[4] = True if self.current_lx < seg[2] - 0.005 else False
                        if seg[1] <= self.current_lx <= seg[2]:
                            seg[3] = current_r

                    # Yeni Segment Ekleme
                    last_end_x = self.vessel_segments[-1][2] if self.vessel_segments else 0.0
                    if self.current_lx > last_end_x + 0.001:
                        self.marker_id_counter += 1
                        self.vessel_segments.append([self.marker_id_counter, last_end_x, self.current_lx, current_r, False])
                
                rclpy.spin_once(self, timeout_sec=0)
            except Exception: pass

    def _get_smooth_color(self, r_val):
        norm = np.clip((r_val - self.r_min) / (self.r_max - self.r_min), 0.0, 1.0)
        float_idx = norm * (len(self.colors) - 1)
        idx1 = int(float_idx); idx2 = min(idx1 + 1, len(self.colors) - 1)
        w = float_idx - idx1; c1, c2 = self.colors[idx1], self.colors[idx2]
        return (float(c1[0]*(1-w)+c2[0]*w), float(c1[1]*(1-w)+c2[1]*w), float(c1[2]*(1-w)+c2[2]*w))

    def _create_marker_msg(self, m_id, start_x, end_x, r, alpha, now):
        red, g, b = self._get_smooth_color(r)
        m = Marker()
        m.header.stamp, m.header.frame_id = now, "map"
        m.ns, m.id = "vessel", int(m_id)
        m.type, m.action = Marker.CYLINDER, Marker.ADD
        m.pose.position.x = float((start_x + end_x) / 2.0)
        m.pose.orientation.y = m.pose.orientation.w = 0.7071
        m.scale.x = m.scale.y = float(r * 2.0 * self.visual_scale)
        m.scale.z = float(abs(end_x - start_x))
        m.color.r, m.color.g, m.color.b, m.color.a = red, g, b, alpha
        return m

    def _republish_all_markers(self):
        if not self.vessel_segments: return
        ma = MarkerArray(); now = self.get_clock().now().to_msg()
        for seg in self.vessel_segments:
            ma.markers.append(self._create_marker_msg(seg[0], seg[1], seg[2], seg[3], 0.15 if seg[4] else 0.8, now))
        self.marker_pub.publish(ma)

    def _publish_legend_static(self):
            ma = MarkerArray()
            now = self.get_clock().now().to_msg()
            step = (self.r_max - self.r_min) / 14.0
            
            for i in range(15):
                val = self.r_min + (i * step)
                col = self.colors[i]
                
                # 1. Renkli Kutu (Küp)
                m_c = Marker()
                m_c.header.stamp, m_c.header.frame_id = now, "map"
                m_c.ns, m_c.id, m_c.type = "legend_cubes", i, Marker.CUBE
                m_c.pose.position.x, m_c.pose.position.y, m_c.pose.position.z = -0.1, -0.3, 0.1 + (i*0.02)
                m_c.scale.x = m_c.scale.y = m_c.scale.z = 0.015
                m_c.color.r, m_c.color.g, m_c.color.b, m_c.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
                ma.markers.append(m_c)

                # 2. Değer Metni (Yazı)
                m_t = Marker()
                m_t.header.stamp, m_t.header.frame_id = now, "map"
                m_t.ns, m_t.id, m_t.type = "legend_text", i, Marker.TEXT_VIEW_FACING
                m_t.pose.position.x, m_t.pose.position.y, m_t.pose.position.z = -0.1, -0.35, 0.1 + (i*0.02)
                m_t.scale.z = 0.015 # Yazı boyutu
                m_t.color.r, m_t.color.g, m_t.color.b, m_t.color.a = 1.0, 1.0, 1.0, 1.0 # Beyaz yazı
                
                # Metreyi mm'ye çevirip yazdırıyoruz (Örn: 0.01 -> 10.0 mm)
                m_t.text = f"{val * 1000:.1f} mm"
                ma.markers.append(m_t)
                
            self.marker_pub.publish(ma)

    def _publish_legend_dynamic(self):
            ma = MarkerArray()
            now = self.get_clock().now().to_msg()
            # Ölçek çarpanı kullanılarak cm hesabı yapıldı
            scale_to_cm = 100.0 / self.visual_x_scale
            dist_slam, dist_odom = self.current_lx * scale_to_cm, self.odom_x * scale_to_cm
            diff = dist_slam - dist_odom

            texts = [
                (f"SLAM: {dist_slam:.2f} cm", 0.5, (1.0, 1.0, 1.0), 99),
                (f"Odom: {dist_odom:.2f} cm", 0.45, (1.0, 1.0, 1.0), 100),
                (f"Hata: {diff:.2f} cm", 0.4, (1.0, 1.0, 1.0), 101)
            ]

            for text_val, z_pos, color, m_id in texts:
                m = Marker()
                m.header.stamp, m.header.frame_id = now, "map"
                m.ns, m.id, m.type = "ui_stats", m_id, Marker.TEXT_VIEW_FACING
                m.pose.position.x, m.pose.position.y, m.pose.position.z = -0.05, -0.3, z_pos
                m.scale.z, m.color.a = 0.03, 1.0
                m.color.r, m.color.g, m.color.b = color
                m.text = text_val
                ma.markers.append(m)
            self.marker_pub.publish(ma)

    def _send_static_tf(self):
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.7071
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.7071
        self.static_tf_broadcaster.sendTransform(t)

def main():
    rclpy.init(); node = CatheterBridge(); node.run()
    if node.is_tty: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
    rclpy.shutdown()

if __name__ == '__main__': main()