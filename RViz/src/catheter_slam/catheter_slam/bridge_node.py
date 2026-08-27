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
        serial_port = '/dev/ttyUSB1' 
        baud_rate = 115200
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.01)
            self.get_logger().info(f"Seri port aktif: {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Port hatasi: {e}"); raise e

        # --- PARAMETRELER (Metre cinsinden)---
        self.v_min, self.v_max = 1.4, 2.05
        self.r_min, self.r_max = 0.01, 0.017
        self.visual_x_scale = 25.0 
        self.radius_multiplier = 20.0 
        self.colors = [(1,0,0),(1,0.4,0),(1,1,0),(0.5,1,0),(0,1,0),(0,1,0.5),(0,1,1),(0,0.5,1),(0,0,1),(0.4,0,1),(0.7,0,1),(1,0,1),(1,0,0.5),(0.6,0.6,0.6),(1,1,1)]
        
        WHEEL_RADIUS_M = 0.015
        PPR = 800.0
        self.P_METRE = (2 * math.pi * WHEEL_RADIUS_M) / PPR

        self.noise_enabled = True
        self.noise_cycle_period = 10.0 
        self.noise_on_duration = 1.0   
        self.v_filtered = None
        self.v_alpha = 0.15 

        self.isNoised = 0

        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            try: self.settings = termios.tcgetattr(sys.stdin)
            except Exception: self.is_tty = False

        # --- DURUM TAKİBİ ---
        self.prev_enc = None
        self.current_lx = 0.0     
        self.odom_x = 0.0          
        self.vessel_segments = [] 
        self.last_recorded_odom_x = -1.0 # Son kaydedilen SLAM konumu için
        self.last_scan_publish_time = self.get_clock().now()

        # --- TF ALTYAPISI ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        marker_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', marker_qos)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', scan_qos)
        
        self._send_static_tf()
        self.create_timer(2.0, self._publish_legend_static)
        self.create_timer(0.1, self._publish_legend_dynamic)
        self.create_timer(0.4, self._republish_all_markers)

    def run(self):
        while rclpy.ok():
            try:
                waiting = self.ser.in_waiting
                if waiting < 6:
                    rclpy.spin_once(self, timeout_sec=0.001); continue

                data = self.ser.read((waiting // 6) * 6)
                
                now_ros = self.get_clock().now()
                now_msg = now_ros.to_msg()
                now_sec = now_ros.nanoseconds / 1e9
                time_in_cycle = now_sec % self.noise_cycle_period
                is_noise_time = time_in_cycle < self.noise_on_duration

                current_r = self.r_min

                for adc_raw, enc_raw in struct.iter_unpack('>Hi', data):
                    if self.prev_enc is None:
                        self.prev_enc = enc_raw; continue
                    
                    delta = enc_raw - self.prev_enc
                    current_noise = 0.0
                    if self.noise_enabled and is_noise_time and self.isNoised < 2:
                        current_noise = abs(np.random.normal(0.2, 0.2))
                        self.isNoised = self.isNoised + 1

                    self.odom_x += (delta + current_noise) * self.P_METRE * self.visual_x_scale
                    self.prev_enc = enc_raw

                    v_raw = abs((adc_raw * 3.3 / 4095.0))
                    if self.v_filtered is None: self.v_filtered = v_raw
                    else: self.v_filtered = (self.v_alpha * v_raw) + ((1.0 - self.v_alpha) * self.v_filtered)
                    
                    v_clipped = np.clip(self.v_filtered, self.v_min, self.v_max)
                    norm = (v_clipped - self.v_min) / (self.v_max - self.v_min)
                    current_r = self.r_max - norm * (self.r_max - self.r_min)

                # --- SLAM VE YAYINLAMA ---
                now_ros = self.get_clock().now()
                now_msg = now_ros.to_msg()

                # 1. TF Yayınla
                t = TransformStamped()
                t.header.stamp = now_msg
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = float(self.odom_x)
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

                # 2. SLAM Feedback (Gerçek konumu al)
                try:
                    trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    self.current_lx = trans.transform.translation.x
                except:
                    self.current_lx = self.odom_x

                # 3. SLAM Marker Kaydı (Encoder yerine SLAM konumunu baz al)
                if abs(self.current_lx - self.last_recorded_odom_x) > 0.001:
                    self.vessel_segments.append((self.current_lx, current_r))
                    self.last_recorded_odom_x = self.current_lx

                # 4. Scan Yayınla
                if (now_ros - self.last_scan_publish_time).nanoseconds / 1e9 > 0.05: 
                    self._publish_scan(now_msg, current_r)
                    self.last_scan_publish_time = now_ros

                rclpy.spin_once(self, timeout_sec=0)
            except Exception: pass

    def _publish_scan(self, now_msg, r):
        scan_msg = LaserScan()
        scan_msg.header.stamp = now_msg 
        scan_msg.header.frame_id = 'laser'
        scan_msg.angle_min, scan_msg.angle_max = 0.0, 2.0 * math.pi
        num_pts = 360
        scan_msg.angle_increment = (2.0 * math.pi) / num_pts
        scan_msg.range_min, scan_msg.range_max = 0.001, 100.0
        r_vis = float(r * self.radius_multiplier)
        scan_msg.ranges = [r_vis + np.random.uniform(-0.0005, 0.0005) for _ in range(num_pts)]
        self.scan_pub.publish(scan_msg)

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
        m.scale.x = m.scale.y = float(r * 2.0 * self.radius_multiplier)
        m.scale.z = float(abs(end_x - start_x) * 1.05)
        m.color.r, m.color.g, m.color.b, m.color.a = red, g, b, alpha
        return m

    def _republish_all_markers(self):
        if len(self.vessel_segments) < 2: return
        # Veriler zaten SLAM (map) koordinatında olduğu için offset eklemeye gerek yok
        ma = MarkerArray(); now = self.get_clock().now().to_msg()
        for i in range(1, len(self.vessel_segments)):
            p1_map, _ = self.vessel_segments[i-1]
            p2_map, r2 = self.vessel_segments[i]
            # Kateterin gerisinde kalıp kalmadığını kontrol et
            is_past = True if self.current_lx < p2_map - 0.005 else False
            ma.markers.append(self._create_marker_msg(i, p1_map, p2_map, r2, 0.15 if is_past else 0.8, now))
        self.marker_pub.publish(ma)

    def _publish_legend_static(self):
        ma = MarkerArray(); now = self.get_clock().now().to_msg()
        step = (self.r_max - self.r_min) / 14.0
        for i in range(15):
            val = self.r_min + (i * step); col = self.colors[i]
            m_c = Marker()
            m_c.header.stamp, m_c.header.frame_id = now, "map"
            m_c.ns, m_c.id, m_c.type = "legend_cubes", i, Marker.CUBE
            m_c.pose.position.x, m_c.pose.position.y, m_c.pose.position.z = -0.1, -0.3, 0.1 + (i*0.02)
            m_c.scale.x = m_c.scale.y = m_c.scale.z = 0.015
            m_c.color.r, m_c.color.g, m_c.color.b, m_c.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
            ma.markers.append(m_c)
            m_t = Marker()
            m_t.header.stamp, m_t.header.frame_id = now, "map"
            m_t.ns, m_t.id, m_t.type = "legend_text", i, Marker.TEXT_VIEW_FACING
            m_t.pose.position.x, m_t.pose.position.y, m_t.pose.position.z = -0.1, -0.35, 0.1 + (i*0.02)
            m_t.scale.z = 0.015; m_t.color.r, m_t.color.g, m_t.color.b, m_t.color.a = 1.0, 1.0, 1.0, 1.0
            m_t.text = f"{val * 1000:.1f} mm"; ma.markers.append(m_t)
        self.marker_pub.publish(ma)

    def _publish_legend_dynamic(self):
        ma = MarkerArray(); now = self.get_clock().now().to_msg()
        scale_to_cm = 100.0 / self.visual_x_scale
        dist_slam, dist_odom = self.current_lx * scale_to_cm, self.odom_x * scale_to_cm
        texts = [(f"SLAM: {dist_slam:.2f} cm", 0.5, (1.0, 1.0, 1.0), 99),
                 (f"Odom: {dist_odom:.2f} cm", 0.45, (1.0, 1.0, 1.0), 100),
                 (f"Hata: {dist_slam - dist_odom:.2f} cm", 0.4, (1.0, 1.0, 1.0), 101)]
        for text_val, z_pos, color, m_id in texts:
            m = Marker(); m.header.stamp, m.header.frame_id = now, "map"
            m.ns, m.id, m.type = "ui_stats", m_id, Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = -0.05, -0.3, z_pos
            m.scale.z, m.color.a = 0.03, 1.0; m.color.r, m.color.g, m.color.b = color; m.text = text_val
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def _send_static_tf(self):
        t_map = TransformStamped()
        t_map.header.stamp = self.get_clock().now().to_msg()
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.y, t_map.transform.rotation.w = 0.7071, 0.7071
        self.static_tf_broadcaster.sendTransform(t_map)

def main():
    rclpy.init(); node = CatheterBridge(); node.run()
    if node.is_tty: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
    rclpy.shutdown()

if __name__ == '__main__': main()