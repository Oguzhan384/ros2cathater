from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('catheter_slam')
    
    # Dosya yolları
    slam_config = os.path.join(pkg, 'config', 'slam.yaml') 
    rviz_config = os.path.join(pkg, 'config', 'catheter.rviz')

    # 1. Statik Transform: base_link -> laser (Yeni Stil Argümanlar)
    # Jazzy'de hata almamak için açık isimli argümanlar (--frame-id vb.) kullanıldı.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_laser',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--yaw', '0', '--pitch', '0', '--roll', '0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'laser'
        ]
    )

    # 2. SLAM Node (Standart Düğüm Olarak Başlatılır)
    # Lifecycle otomasyonu kaldırıldı. Düğüm 'unconfigured' modda bekleyecektir.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam', # YAML'daki 'slam:' başlığı ile aynı olmalı
        parameters=[slam_config, {'use_sim_time': False}],
        output='screen'
    )

    # 3. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # 4. Bridge Node (Python Kodun)
    # Veri akışını başlatan düğüm. 
    bridge_node = TimerAction(
        period=5.0, # Sistem oturduktan sonra başlasın
        actions=[Node(
            package='catheter_slam',
            executable='catheter_bridge',
            name='catheter_bridge',
            parameters=[{'use_sim_time': False}],
            output='screen'
        )]
    )

    return LaunchDescription([
        static_tf_node,
        slam_node,
        rviz_node,
        bridge_node
    ])