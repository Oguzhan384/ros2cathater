#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import struct

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import pandas as pd
import numpy as np
import math
import time
import os


class CatheterBridge(Node):
    def __init__(self):
        super().__init__('catheter_bridge')

        # ── Publishers ───────────────────────────────────────────
        self.scan_pub   = self.create_publisher(LaserScan,   '/scan',           10)
        self.odom_pub   = self.create_publisher(Odometry,    '/odom',           10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vessel_markers', 10)

        # ── TF Broadcasters ─────────────────────────────────────
        self.tf_broadcaster        = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._send_static_tf()

        # ── CSV path ─────────────────────────────────────────────
        self.csv_path = os.path.expanduser(
            '/home/oguzhan/Desktop/ros2Cathater/ros2cathater/src/catheter_slam/data/Deney3_Combined_Interpolated-Combined Data.csv'
            )

        # ── Voltage/Vpp → radius calibration ─────────────────────
        # Voltage zaten Vpp kabul ediliyor.
        # Gerçek v_min ve v_max CSV okunduktan sonra datadan bulunacak.
        self.v_min = None
        self.v_max = None

        # ── Fiziksel damar yarıçap sınırları ─────────────────────
        # 15 cm çap → r = 0.075 m
        # 45 cm çap → r = 0.225 m
        self.r_min = 0.075
        self.r_max = 0.225

        # ── Interpolation settings ───────────────────────────────
        self.use_interpolation = True

        # x artık mm geldiği için gerçek analizde 1 mm aralık daha mantıklı.
        # 0.001 m = 1 mm
        self.interp_dx = 0.001

        # CSV'deki x verisi mm cinsinden geliyor.
        self.x_unit = 'mm'

        # Sadece RViz görselleştirmesi için x eksenini büyütür.
        # Analizde x hâlâ gerçek mm/metre olarak kullanılır.
        self.visual_x_scale = 10.0

        # Yayınlama periyodu
        self.publish_period = 0.01   # 20 Hz

        # Renk değişimi çap/kalınlık için
        # 2 cm = 0.02 m
        self.thickness_color_step = 0.02

        # RViz renk açıklaması
        self.show_legend = True

        self.cloud_pub = self.create_publisher(
        PointCloud2,
        '/vessel_cloud',
        10
        )
        self.all_points = []
        self.active_markers = {}
        self.last_x_vis = -999.0
    # ─────────────────────────────────────────────────────────────
    # TF
    # ─────────────────────────────────────────────────────────────
    def _send_static_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'laser'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    # ─────────────────────────────────────────────────────────────
    # Helper functions
    # ─────────────────────────────────────────────────────────────
    def _clamp_voltage(self, v):
        if self.v_min is None or self.v_max is None:
            return float(v)

        return max(self.v_min, min(self.v_max, float(v)))

    def _update_voltage_limits(self, voltage_values):
        """
        Datadaki hazır Vpp değerleri içinden direkt min ve max alır.
        Burada 0-1 varsayımı yok.
        """

        voltage_values = np.asarray(voltage_values, dtype=float)

        if len(voltage_values) < 2:
            self.get_logger().error('Voltage/Vpp limiti için en az 2 değer gerekli.')
            return False

        self.v_min = float(np.min(voltage_values))
        self.v_max = float(np.max(voltage_values))

        if abs(self.v_max - self.v_min) < 1e-9:
            self.get_logger().error(
                f'v_min ve v_max neredeyse aynı! '
                f'v_min={self.v_min:.6f}, v_max={self.v_max:.6f}'
            )
            return False

        self.get_logger().info(
            f'Datadan bulunan Vpp aralığı: '
            f'v_min={self.v_min:.6f} V, v_max={self.v_max:.6f} V'
        )

        self.get_logger().info(
            f'Çap aralığı: '
            f'min={self.r_min * 200:.2f} cm, '
            f'max={self.r_max * 200:.2f} cm'
        )

        self.get_logger().info(
            'Eşleşme: küçük Vpp → geniş çap, büyük Vpp → dar çap'
        )

        return True

    def _voltage_to_radius(self, v):
        """
        TERS LİNEER EŞLEME:

        v_min → r_max
        v_max → r_min

        Yani:
        Küçük Vpp → geniş damar
        Büyük Vpp → dar damar
        """

        if self.v_min is None or self.v_max is None:
            self.get_logger().error('v_min/v_max henüz belirlenmedi!')
            return float(self.r_min)

        if abs(self.v_max - self.v_min) < 1e-9:
            return float(self.r_min)

        v = self._clamp_voltage(v)

        norm = (v - self.v_min) / (self.v_max - self.v_min)
        norm = float(np.clip(norm, 0.0, 1.0))

        r = self.r_max - norm * (self.r_max - self.r_min)

        return float(r)

    def _convert_x_to_meters(self, x_values):
        x_values = np.asarray(x_values, dtype=float)
        unit = self.x_unit.lower()

        if unit == 'auto':
            max_abs_x = float(np.nanmax(np.abs(x_values)))

            if max_abs_x > 200.0:
                unit = 'mm'
            elif max_abs_x > 2.0:
                unit = 'cm'
            else:
                unit = 'm'

            self.get_logger().info(f'x birimi otomatik: "{unit}"')
        else:
            self.get_logger().info(f'x birimi manuel: "{unit}"')

        if unit == 'm':
            return x_values

        if unit == 'cm':
            return x_values / 100.0

        if unit == 'mm':
            return x_values / 1000.0

        raise ValueError("x_unit: 'auto', 'm', 'cm' veya 'mm' olmalı.")

    def _load_csv_data(self):
            if not os.path.exists(self.csv_path):
                self.get_logger().error(f'CSV bulunamadı: {self.csv_path}')
                return None

            df = pd.read_csv(self.csv_path)
            # Kolon isimlerini temizle: 'Time (s)' -> 'time', 'Voltage' -> 'voltage'
            df.columns = df.columns.str.strip().str.lower()
            df.columns = [c.split(' ')[0] for c in df.columns]

            # Sayısallaştır
            for col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce')
            df = df.dropna().reset_index(drop=True)

            # Eğer x değerleri mm ise (225 gibi), metreye çevir (0.225)
            if df['x'].max() > 10.0:
                df['x'] = df['x'] / 1000.0

            self._update_voltage_limits(df['voltage'].to_numpy())
            return df
    # ─────────────────────────────────────────────────────────────
    # Scan
    # ─────────────────────────────────────────────────────────────
    def _publish_scan(self, r, x_vis, now):
        num_beams = 360
        angle_inc = (2.0 * math.pi) / num_beams

        scan = LaserScan()
        scan.header.stamp     = now
        scan.header.frame_id  = 'laser'
        scan.angle_min        = 0.0
        scan.angle_max        = 2.0 * math.pi - angle_inc
        scan.angle_increment  = angle_inc
        scan.time_increment   = 0.0
        scan.scan_time        = self.publish_period
        scan.range_min        = 0.001
        scan.range_max        = 1.0

        noise_std = r * 0.02

        center_offset_y = r * 0.03 * math.sin(x_vis * 50.0)
        center_offset_z = r * 0.02 * math.cos(x_vis * 50.0)

        ranges = []

        for i in range(num_beams):
            angle = i * angle_inc

            dy = center_offset_y * math.cos(angle)
            dz = center_offset_z * math.sin(angle)

            effective_r = r + dy + dz
            noisy_r = effective_r + np.random.normal(0.0, noise_std)
            noisy_r = float(np.clip(noisy_r, scan.range_min, scan.range_max))

            ranges.append(noisy_r)

        scan.ranges = ranges
        self.scan_pub.publish(scan)

    # ─────────────────────────────────────────────────────────────
    # Thickness color helper
    # ─────────────────────────────────────────────────────────────
    def _get_thickness_color(self, r):
        """
        Damar çapı/kalınlığı her 2 cm değiştiğinde renk değiştirir.

        Burada kalınlık = çap = 2*r olarak alınmıştır.
        """

        diameter = 2.0 * r
        min_diameter = 2.0 * self.r_min
        max_diameter = 2.0 * self.r_max
        step = self.thickness_color_step

        thickness_segment = int(np.floor((diameter - min_diameter) / step))

        max_segment = int(np.ceil((max_diameter - min_diameter) / step)) - 1
        thickness_segment = int(np.clip(thickness_segment, 0, max_segment))

        colors = [
            (1.0, 0.0, 0.0),   # kırmızı
            (1.0, 0.4, 0.0),   # turuncu
            (1.0, 1.0, 0.0),   # sarı
            (0.5, 1.0, 0.0),   # açık yeşil
            (0.0, 1.0, 0.0),   # yeşil
            (0.0, 1.0, 0.5),   # yeşil-camgöbeği
            (0.0, 1.0, 1.0),   # camgöbeği
            (0.0, 0.5, 1.0),   # açık mavi
            (0.0, 0.0, 1.0),   # mavi
            (0.4, 0.0, 1.0),   # mor-mavi
            (0.7, 0.0, 1.0),   # mor
            (1.0, 0.0, 1.0),   # pembe/magenta
            (1.0, 0.0, 0.5),   # pembe-kırmızı
            (0.6, 0.6, 0.6),   # gri
            (1.0, 1.0, 1.0),   # beyaz
        ]

        color = colors[thickness_segment % len(colors)]

        return color, thickness_segment

    # ─────────────────────────────────────────────────────────────
    # Marker
    # ─────────────────────────────────────────────────────────────
    def _make_cylinder_marker(self, idx, x_vis, r, v, now):
        m = Marker()

        m.header.stamp    = now
        m.header.frame_id = 'odom'

        m.ns     = 'vessel'
        m.id     = int(idx)
        m.type   = Marker.CYLINDER
        m.action = Marker.ADD

        m.pose.position.x = float(x_vis)
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0

        # Silindiri X eksenine yatır
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.7071068
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 0.7071068

        # Cylinder marker default olarak z ekseni boyunca uzar.
        # Bu quaternion ile X'e yatırdığımız için scale.z damar uzunluk parçası olur.
        # Görselde x ekseni büyütüldüğü için marker uzunluğu da büyütülüyor.
        m.scale.x = float(r * 2.0)
        m.scale.y = float(r * 2.0)
        m.scale.z = float(self.interp_dx * self.visual_x_scale * 1.05)

        color, _ = self._get_thickness_color(r)
        r_col, g_col, b_col = color

        m.color.r = r_col
        m.color.g = g_col
        m.color.b = b_col
        m.color.a = 1.0

        m.lifetime.sec = 0

        return m

    def _clear_markers(self):
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL

        ma = MarkerArray()
        ma.markers.append(delete_marker)

        self.marker_pub.publish(ma)

    # ─────────────────────────────────────────────────────────────
    # Legend Marker
    # ─────────────────────────────────────────────────────────────
    def _add_color_legend(self, marker_array, now):
        """
        RViz içinde renklerin hangi çap aralığına karşılık geldiğini gösterir.
        2 cm aralıklarla otomatik legend oluşturur.
        """

        min_diameter_cm = self.r_min * 200.0
        max_diameter_cm = self.r_max * 200.0
        step_cm = self.thickness_color_step * 100.0

        start_x = 0.0
        start_y = -0.80
        start_z = 0.90

        num_segments = int(np.ceil((max_diameter_cm - min_diameter_cm) / step_cm))

        for i in range(num_segments):
            lower_cm = min_diameter_cm + i * step_cm
            upper_cm = min(lower_cm + step_cm, max_diameter_cm)

            mid_diameter_m = ((lower_cm + upper_cm) / 2.0) / 100.0
            mid_r = mid_diameter_m / 2.0

            color, _ = self._get_thickness_color(mid_r)
            r_col, g_col, b_col = color

            # Renk kutusu
            box = Marker()
            box.header.stamp = now
            box.header.frame_id = 'odom'
            box.ns = 'diameter_legend_box'
            box.id = 10000 + i
            box.type = Marker.CUBE
            box.action = Marker.ADD

            box.pose.position.x = start_x
            box.pose.position.y = start_y
            box.pose.position.z = start_z - i * 0.055
            box.pose.orientation.w = 1.0

            box.scale.x = 0.04
            box.scale.y = 0.04
            box.scale.z = 0.04

            box.color.r = r_col
            box.color.g = g_col
            box.color.b = b_col
            box.color.a = 1.0

            box.lifetime.sec = 0
            marker_array.markers.append(box)

            # Yazı
            text = Marker()
            text.header.stamp = now
            text.header.frame_id = 'odom'
            text.ns = 'diameter_legend_text'
            text.id = 10100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = start_x + 0.12
            text.pose.position.y = start_y
            text.pose.position.z = start_z - i * 0.055
            text.pose.orientation.w = 1.0

            text.scale.z = 0.045
            text.text = f'{lower_cm:.0f}-{upper_cm:.0f} cm cap'

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            text.lifetime.sec = 0
            marker_array.markers.append(text)

    # ─────────────────────────────────────────────────────────────
    # TF + Odom publish
    # ─────────────────────────────────────────────────────────────
    def _publish_tf(self, x_vis, now):
        t = TransformStamped()

        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'

        t.transform.translation.x = float(x_vis)
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def _publish_odom(self, x_vis, now):
        odom = Odometry()

        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = float(x_vis)
        odom.pose.pose.position.y    = 0.0
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.w = 1.0

        odom.pose.covariance[0]  = 1e-3
        odom.pose.covariance[7]  = 1e-3
        odom.pose.covariance[14] = 1e-6
        odom.pose.covariance[21] = 1e-6
        odom.pose.covariance[28] = 1e-6
        odom.pose.covariance[35] = 1e-3

        self.odom_pub.publish(odom)

    def _publish_cloud(self, now):
            if not self.all_points: return
            
            # Sadece son 5000 noktayı göster (Opsiyonel: Eğer CSV çok büyükse kasmayı önler)
            # points_to_send = self.all_points[-5000:] 
            points_to_send = self.all_points

            msg = PointCloud2()
            msg.header.stamp = now
            msg.header.frame_id = 'odom'
            msg.height = 1
            msg.width = len(points_to_send)
            msg.point_step = 12
            msg.row_step = 12 * len(points_to_send)
            msg.is_dense = True
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            msg.data = struct.pack('<' + 'fff' * len(points_to_send), 
                                *[v for p in points_to_send for v in p])
            self.cloud_pub.publish(msg)

    def _publish_legend(self):
        """Renk skalası."""
        legend_ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # 15 renk
        colors = [
            (1.0, 0.0, 0.0), (1.0, 0.4, 0.0), (1.0, 1.0, 0.0), (0.5, 1.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, 1.0, 0.5), (0.0, 1.0, 1.0), (0.0, 0.5, 1.0), (0.0, 0.0, 1.0), (0.4, 0.0, 1.0),
            (0.7, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 0.0, 0.5), (0.6, 0.6, 0.6), (1.0, 1.0, 1.0)
        ]

        # Başlangıç konumu
        start_x = -0.1
        start_y = -0.4
        start_z = 0.1
        spacing = 0.04  # Her satır arası boşluk

        for i, col in enumerate(colors):
            # r aralığı hesaplama (Her segment 2cm çap, yani 1cm yarıçap farkına denk gelir)
            # Formül: r_start = r_min + (i * step / 2)
            r_low = self.r_min + (i * self.thickness_color_step / 2.0)
            r_high = r_low + (self.thickness_color_step / 2.0)

            # 1. RENK KÜPÜ
            cube = Marker()
            cube.header.frame_id = "odom"
            cube.header.stamp = now
            cube.ns = "legend_cubes"
            cube.id = 50000 + i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = start_x
            cube.pose.position.y = start_y
            cube.pose.position.z = start_z + (i * spacing)
            cube.scale.x = 0.03
            cube.scale.y = 0.03
            cube.scale.z = 0.03
            cube.color.r, cube.color.g, cube.color.b = col
            cube.color.a = 1.0
            legend_ma.markers.append(cube)

            # 2. R DEĞERİ METNİ
            text = Marker()
            text.header.frame_id = "odom"
            text.header.stamp = now
            text.ns = "legend_text"
            text.id = 60000 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = start_x + 0.05 # Küpün biraz sağında
            text.pose.position.y = start_y
            text.pose.position.z = start_z + (i * spacing)
            text.scale.z = 0.025 # Yazı boyutu
            text.text = f"r: {r_low:.3f}-{r_high:.3f}m"
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            legend_ma.markers.append(text)

        self.marker_pub.publish(legend_ma)

    def _delete_ahead_visuals(self, current_x_vis):
            """Mevcut x konumunun ilerisindeki ve ucundaki her şeyi temizler."""
            delete_ma = MarkerArray()
            # Tolerans: 0.0005 (Görsel ölçekte 0.5mm). 
            # Bu değer, tam sınırda kalan parçaların atlanmasını önler.
            tolerans = 0.005 
            
            # Hafızadaki x konumu mevcut konumdan BÜYÜK veya EŞİT olan HER ŞEYİ bul
            to_remove = [m_id for m_id, pos in self.active_markers.items() if pos > (current_x_vis - tolerans)]
            
            for m_id in to_remove:
                m = Marker()
                m.header.frame_id = 'odom'
                m.ns = 'vessel'
                m.id = int(m_id)
                m.action = Marker.DELETE
                delete_ma.markers.append(m)
                
                # Hafızadan (sözlükten) sildiğinden emin ol
                if m_id in self.active_markers:
                    del self.active_markers[m_id]
                
            if delete_ma.markers:
                self.marker_pub.publish(delete_ma)
                
            # PointCloud'u temizle ve anında yayınla
            self.all_points = [p for p in self.all_points if p[0] <= (current_x_vis + tolerans)]
            self._publish_cloud(self.get_clock().now().to_msg())

    def _dynamic_erase(self, current_x_vis):
        """
        Mevcut x konumundan ileride olan her şeyi görselden siler.
        """
        delete_marker_array = MarkerArray()
        
        # 1. Silinecek Marker ID'lerini bul (Mevcut konumun ilerisindekiler)
        ids_to_remove = [m_id for m_id, pos in self.active_markers.items() if pos > current_x_vis]
        
        for m_id in ids_to_remove:
            del_m = Marker()
            del_m.header.frame_id = 'odom'
            del_m.ns = 'vessel'
            del_m.id = int(m_id)
            del_m.action = Marker.DELETE  # RViz'e silme komutu gönder
            delete_marker_array.markers.append(del_m)
            
            # Takip listemizden (sözlükten) çıkar
            del self.active_markers[m_id]
        
        # RViz'e silme paketini gönder
        if delete_marker_array.markers:
            self.marker_pub.publish(delete_marker_array)
            
        # 2. PointCloud'u Temizle (Listeden sadece geride kalan noktaları tut)
        self.all_points = [p for p in self.all_points if p[0] <= current_x_vis]

    # ─────────────────────────────────────────────────────────────
    # Main run
    # ─────────────────────────────────────────────────────────────
    def run(self):
            df = self._load_csv_data()
            if df is None: return

            self._clear_markers()
            self.all_points = []
            self.active_markers = {}

            # Referanslar
            first_x_vis = float(df['x'].iloc[0]) * self.visual_x_scale
            self.last_x_vis = first_x_vis
            self.last_draw_x = first_x_vis 
            last_row_time = float(df['time'].iloc[0])

            # Üst üste binmeyi önlemek için minimum hareket eşiği
            min_move_dist = 0.000005 * self.visual_x_scale

            self._publish_legend()

            for idx, row in df.iterrows():
                if not rclpy.ok(): break

                now = self.get_clock().now().to_msg()
                t_val, x_val, v_val = float(row['time']), float(row['x']), float(row['voltage'])
                x_vis = x_val * self.visual_x_scale
                r = self._voltage_to_radius(v_val)

                # --- 1. GERİ HAREKET (SİLME) ---
                if x_vis < self.last_x_vis:
                    self._delete_ahead_visuals(x_vis)
                    # Geri gidince çizim referansını da o noktaya çek
                    self.last_draw_x = x_vis
                
                # --- 2. İLERİ HAREKET (ÇİZME) ---
                # Sadece yeterli mesafe gidildiyse yeni marker ekle
                dist_since_last_draw = x_vis - self.last_draw_x
                
                if dist_since_last_draw >= min_move_dist:
                    marker = self._make_cylinder_marker(idx, x_vis, r, v_val, now)
                    
                    # Marker boyu, kat edilen mesafe kadar olsun
                    marker.scale.z = dist_since_last_draw * 1.1 # Hafif pay bırak
                    
                    self.active_markers[idx] = x_vis
                    self.last_draw_x = x_vis # Çizilen son noktayı güncelle
                    
                    ma = MarkerArray()
                    ma.markers.append(marker)
                    self.marker_pub.publish(ma)

                    for i in range(12):
                        theta = 2.0 * math.pi * i / 12
                        self.all_points.append((float(x_vis), float(r * math.cos(theta)), float(r * math.sin(theta))))

                # --- 3. YAYINLAR ---
                self._publish_tf(x_vis, now)
                self._publish_odom(x_vis, now)
                self._publish_cloud(now)
                
                print(f"Time: {t_val:7.3f}s | x: {x_val:8.4f} | Voltage: {v_val:6.4f}V | r: {r:6.4f}m", flush=True)

                # --- 4. ZAMANLAMA ---
                csv_diff = t_val - last_row_time
                wait_time = max(csv_diff, self.publish_period)
                if wait_time > 0.5: wait_time = self.publish_period

                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(wait_time)
                
                last_row_time = t_val
                self.last_x_vis = x_vis

            self.get_logger().info('Tamamlandı.')
            rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)

    node = CatheterBridge()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()