#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
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
        self.v_min, self.v_max = 1.0, 1.5
        self.r_min, self.r_max = 0.015, 0.025
        self.visual_x_scale = 25.0
        self.radius_multiplier = 40.0

        self.colors = [(1,0,0),(1,0.4,0),(1,1,0),(0.5,1,0),(0,1,0),(0,1,0.5),(0,1,1),(0,0.5,1),(0,0,1),(0.4,0,1),(0.7,0,1),(1,0,1),(1,0,0.5),(0.6,0.6,0.6),(1,1,1)]
        
        WHEEL_RADIUS_M = 0.015
        PPR = 800.0
        self.P_METRE = (2 * math.pi * WHEEL_RADIUS_M) / PPR

        # --- DURUM TAKİBİ ---
        self.prev_enc = None
        self.current_x_vis = 0.0
        self.anlik_konum = 0.0
        self.marker_id_counter = 0
        self.leftover_data = b''
        self.vessel_segments = [] 

        # ROS Altyapısı
        marker_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', marker_qos)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        self._send_static_tf()
        self.create_timer(0.1, self._publish_legend)

    def _get_smooth_color(self, r_val):
        norm = np.clip((r_val - self.r_min) / (self.r_max - self.r_min), 0.0, 1.0)
        float_idx = norm * (len(self.colors) - 1)
        idx1 = int(float_idx)
        idx2 = min(idx1 + 1, len(self.colors) - 1)
        w = float_idx - idx1
        c1, c2 = self.colors[idx1], self.colors[idx2]
        return (float(c1[0]*(1-w)+c2[0]*w), float(c1[1]*(1-w)+c2[1]*w), float(c1[2]*(1-w)+c2[2]*w))

    def _create_marker_msg(self, m_id, start_x, end_x, r, alpha, now):
        red, g, b = self._get_smooth_color(r)
        m = Marker(header=Header(stamp=now, frame_id='odom'), ns='vessel', id=int(m_id), 
                   type=Marker.CYLINDER, action=Marker.ADD)
        m.pose.position.x = float((start_x + end_x) / 2.0)
        m.pose.orientation.y = m.pose.orientation.w = 0.7071
        m.scale.x = m.scale.y = float(r * 2.0 * self.radius_multiplier)
        m.scale.z = float(abs(end_x - start_x) * 1.02) # Parçalar arası boşluk kalmasın diye %2 pay
        m.color.r, m.color.g, m.color.b, m.color.a = red, g, b, alpha
        return m

    def run(self):
        self.get_logger().info("Sistem Aktif")
        while rclpy.ok():
            waiting = self.ser.in_waiting
            if waiting < 6:
                rclpy.spin_once(self, timeout_sec=0.001); continue

            now = self.get_clock().now().to_msg()
            data_chunk = self.ser.read(waiting)
            if self.leftover_data: data_chunk = self.leftover_data + data_chunk
            full_packets = (len(data_chunk)//6)*6
            data = data_chunk[:full_packets]
            self.leftover_data = data_chunk[full_packets:]

            loop_markers = MarkerArray()

            for adc_raw, enc_raw in struct.iter_unpack('>Hi', data):
                v_raw = abs((adc_raw * 3.3 / 4095.0))
                v_clipped = np.clip(v_raw, self.v_min, self.v_max)
                norm = (v_clipped - self.v_min) / (self.v_max - self.v_min)
                current_r = self.r_max - norm * (self.r_max - self.r_min)

                if self.prev_enc is None:
                    self.prev_enc = enc_raw
                    continue

                now = self.get_clock().now().to_msg()

                #-- SLAM için LaserScan oluşturulması --
                scan_msg = LaserScan()
                scan_msg.header.stamp = now
                scan_msg.header.frame_id = 'laser'


                # Lidar parametreleri
                scan_msg.angle_min = 0.0
                scan_msg.angle_max = 2.0 * math.pi
                scan_msg.angle_increment = (2.0 * math.pi) / 36 # 36 nokta için
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = 0.1
                scan_msg.range_min = 0.001
                scan_msg.range_max = 10.0 # Damar çapına göre (radius_multiplier ile çarpılıyor)

                scan_msg.ranges = [float(current_r * self.radius_multiplier)] * 36
                self.scan_pub.publish(scan_msg)
                
                delta = enc_raw - self.prev_enc
                self.prev_enc = enc_raw

                noise = np.random.normal(0, .0) #Slam için encoder noisesi, aralığı arttırarak noiseyi artırabilirsin

                delta_noisy = delta + noise
                
                # --- KONUM GÜNCELLEME ---
                self.anlik_konum += delta_noisy * self.P_METRE 
                self.current_x_vis += delta_noisy * self.P_METRE * self.visual_x_scale
                lx = self.current_x_vis

                if delta_noisy < 0: # GERİ HAREKET
                    for seg in self.vessel_segments:
                        if seg[2] > (lx + 0.001) and not seg[4]: 
                            seg[4] = True 
                            loop_markers.markers.append(self._create_marker_msg(seg[0], seg[1], seg[2], seg[3], 0.15, now))

                elif delta_noisy > 0: # İLERİ HAREKET
                    found_existing = False
                    for seg in self.vessel_segments:
                        if seg[1] <= lx <= seg[2]: 
                            found_existing = True
                            if seg[4]: 
                                seg[4] = False
                                loop_markers.markers.append(self._create_marker_msg(seg[0], seg[1], seg[2], seg[3], 0.8, now))
                            break
                    
                    if not found_existing: 
                        last_end_x = self.vessel_segments[-1][2] if self.vessel_segments else 0.0
                        if lx > last_end_x:
                            self.marker_id_counter += 1
                            new_seg = [self.marker_id_counter, last_end_x, lx, current_r, False]
                            self.vessel_segments.append(new_seg)
                            loop_markers.markers.append(self._create_marker_msg(new_seg[0], new_seg[1], new_seg[2], new_seg[3], 0.8, now))

                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
                t.transform.translation.x = float(self.current_x_vis)
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)
                rclpy.spin_once(self, timeout_sec=0)
                if loop_markers.markers: self.marker_pub.publish(loop_markers)
            

    def _publish_legend(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # 1. Renk Skalası
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
            m_t.scale.z = 0.06; m_t.color.a = 1.0; m_t.text = f"{val*1000:.2f}mm"
            ma.markers.append(m_t)

        # 2. Canlı Mesafe Göstergesi
        dist_m = Marker(header=Header(stamp=now, frame_id='odom'), ns="distance_ui", id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        dist_m.pose.position.x, dist_m.pose.position.y, dist_m.pose.position.z = -0.3, -1.5, 2.2
        dist_m.scale.z = 0.15 
        dist_m.color.r, dist_m.color.g, dist_m.color.b, dist_m.color.a = 1.0, 1.0, 1.0, 1.0 
        
        dist_cm = self.anlik_konum * 100.0
        dist_m.text = f"Mesafe: {dist_cm:.2f} cm"
        ma.markers.append(dist_m)

        self.marker_pub.publish(ma)

    def _send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.7071
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.7071
        self.static_tf_broadcaster.sendTransform(t)

def main():
    rclpy.init(); node = CatheterBridge()
    try: node.run()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()