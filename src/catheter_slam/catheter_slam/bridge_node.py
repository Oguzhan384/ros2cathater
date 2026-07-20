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
            '/home/oguzhan/Desktop/ros2Cathater/ros2cathater/src/catheter_slam/data/data_3.csv'
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

    def _load_and_interpolate_csv(self):
        if not os.path.exists(self.csv_path):
            self.get_logger().error(f'CSV bulunamadı: {self.csv_path}')
            return None

        df = pd.read_csv(self.csv_path)
        df.columns = df.columns.str.strip().str.lower()

        if not {'x', 'voltage'}.issubset(df.columns):
            self.get_logger().error(
                f'CSV kolonları eksik. Bulunan kolonlar: {list(df.columns)}'
            )
            return None
        #x verisi csv den
        # Time kolonu varsa kullanılmıyor.
        # Voltage zaten hazır Vpp kabul ediliyor.
        df = df[['x', 'voltage']].copy()

        df['x']       = pd.to_numeric(df['x'],       errors='coerce')
        df['voltage'] = pd.to_numeric(df['voltage'], errors='coerce')
        df = df.dropna()

        if len(df) < 2:
            self.get_logger().error('Geçerli veri sayısı yetersiz.')
            return None

        # Aynı x konumunda birden fazla Vpp varsa ortalamasını al.
        # Burada max-min yapılmıyor.
        df = df.groupby('x', as_index=False)['voltage'].mean()

        # x'i metreye çevir
        df['x'] = self._convert_x_to_meters(df['x'].to_numpy())

        # x'e göre sırala
        df = df.sort_values('x').reset_index(drop=True)

        x_old = df['x'].to_numpy(dtype=float)
        v_old = df['voltage'].to_numpy(dtype=float)

        if len(df) < 2:
            self.get_logger().error('Geçerli x noktası sayısı yetersiz.')
            return None

        # Datadaki gerçek Vpp min/max değerlerini bul
        if not self._update_voltage_limits(v_old):
            return None

        if not self.use_interpolation:
            return df

        x_min = float(np.min(x_old))
        x_max = float(np.max(x_old))

        x_new = np.arange(
            x_min,
            x_max + self.interp_dx * 0.5,
            self.interp_dx
        )

        if len(x_new) == 0:
            self.get_logger().error('Interpolation için x_new boş oluştu.')
            return None

        if x_new[-1] < x_max:
            x_new = np.append(x_new, x_max)

        # x boyunca Vpp interpolation
        v_new = np.interp(x_new, x_old, v_old)

        self.get_logger().info(
            f'{len(df)} ham x noktası → {len(x_new)} interpole nokta. '
            f'x_real: {x_min:.4f} m → {x_max:.4f} m | '
            f'x_visual_scale={self.visual_x_scale:.1f}'
        )

        return pd.DataFrame({'x': x_new, 'voltage': v_new})

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
    def _publish_cloud(self, r, x_vis, now):

        num_points = 360

        # Bu kesitin noktalarını üret
        for i in range(num_points):

            theta = 2.0 * math.pi * i / num_points

            y = r * math.cos(theta)
            z = r * math.sin(theta)

            # Yeni noktaları kalıcı listeye ekle
            self.all_points.append((x_vis, y, z))

        msg = PointCloud2()

        msg.header.stamp = now
        msg.header.frame_id = 'odom'

        msg.height = 1
        msg.width = len(self.all_points)

        msg.is_bigendian = False
        msg.is_dense = True

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width

        msg.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        msg.data = struct.pack(
            '<' + 'fff' * len(self.all_points),
            *[v for p in self.all_points for v in p]
        )

        self.cloud_pub.publish(msg)
    # ─────────────────────────────────────────────────────────────
    # Main run
    # ─────────────────────────────────────────────────────────────
    def run(self):
        df = self._load_and_interpolate_csv()

        if df is None:
            return

        while rclpy.ok():

            self._clear_markers()             # RViz'deki tüm markerları temizle
            self.all_points = []              # Nokta bulutu listesini sıfırla
            self.active_markers = {}          # Takip sözlüğünü sıfırla
            self.last_x_real = -999.0         # Geri hareket referansını sıfırla
            
            time.sleep(1.0)

            self._clear_markers()

            time.sleep(0.2)

            marker_array = MarkerArray()

            if self.show_legend:
                now = self.get_clock().now().to_msg()
                self._add_color_legend(marker_array, now)
                self.marker_pub.publish(marker_array)

            self.get_logger().info('Yayın başlıyor...')

            for idx, row in df.iterrows():
                now = self.get_clock().now().to_msg()

                

                x_real = float(row['x'])                  # gerçek x, metre cinsinden
                x_vis  = x_real * self.visual_x_scale     # RViz'de çizilecek x

                v = float(row['voltage'])                 # hazır Vpp
                r = self._voltage_to_radius(v)

                # --- GERİ HAREKET KONTROLÜ VE SİLME MANTIĞI ---
                if x_vis < self.last_x_vis:
                    delete_ma = MarkerArray()
                    # Mevcut markerlardan, şu anki konumumuzdan ileride olanları bul
                    ids_to_remove = [m_id for m_id, m_pos in self.active_markers.items() if m_pos > x_vis]
                    
                    for m_id in ids_to_remove:
                        del_m = Marker()
                        del_m.header.frame_id = 'odom'
                        del_m.ns = 'vessel'
                        del_m.id = m_id
                        del_m.action = Marker.DELETE # Silme emri
                        delete_ma.markers.append(del_m)
                        # Takip listesinden de sil
                        del self.active_markers[m_id]

                    if delete_ma.markers:
                        self.marker_pub.publish(delete_ma)
                    
                    # PointCloud (nokta bulutu) listesini de temizle
                    self.all_points = [p for p in self.all_points if p[0] <= x_vis]

                # Marker oluştur ve takip listesine ekle
                marker = self._make_cylinder_marker(idx, x_vis, r, v, now)
                self.active_markers[idx] = x_vis # ID ve konumu kaydet


                diameter_cm = r * 200.0
                min_diameter_cm = self.r_min * 200.0
                step_cm = self.thickness_color_step * 100.0

                _, thickness_segment = self._get_thickness_color(r)

                lower_cm = min_diameter_cm + thickness_segment * step_cm
                upper_cm = lower_cm + step_cm

                ma = MarkerArray()
                ma.markers.append(marker)
                self.marker_pub.publish(ma)

                self._publish_tf(x_vis, now)
                self._publish_odom(x_vis, now)
                self._publish_scan(r, x_vis, now)
                self._publish_cloud(r, x_vis, now)

                self.last_x_vis = x_vis # Son konumu güncelle

                marker = self._make_cylinder_marker(idx, x_vis, r, v, now)
                marker_array.markers.append(marker)
                self.marker_pub.publish(marker_array)

                self.get_logger().info(
                    f'[{idx:04d}] '
                    f'x_real={x_real * 100:.2f}cm | '
                    f'x_vis={x_vis * 100:.2f}cm | '
                    f'Vpp={v:.3f}V | '
                    f'r={r * 100:.2f}cm | '
                    f'cap={diameter_cm:.2f}cm | '
                    f'renk_araligi={lower_cm:.1f}-{upper_cm:.1f}cm'
                )

                rclpy.spin_once(self, timeout_sec=0.001)
                time.sleep(self.publish_period)

        #self.get_logger().info('✓ Tüm veri yayınlandı.')
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