from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

#--- NASIL ÇALIŞTIRILIR ---

#cd /home/(kullanıcı adı)/ros2cathater/ros2cathater/RViz        Klasörün olduğu konuma gidin
#source /opt/ros/humble/setup.bash                              ROS2'yi tanıt
#colcon build --packages-select catheter_slam --symlink-install Paketi derle
#source install/setup.bash                                      Workspaceni tanıt
#ros2 launch catheter_slam catheter_slam.launch.py              Launch dosyasını çalıştır

#Eğer ros2nin çalışmadığı bir işletim sistemi versiyonu kullanıyorsanız (Mesela ben Ubuntu 26.04 LTS kullanıyorum) bir docker açıp herşeyi onun içinde yapmanız lazım
#Ayrıca dockerle çalıştırdığınızda slam hemen aktif olmuyor dockerin içine girip bu komutları yazmanız lazım:
#source /opt/ros/jazzy/setup.bash
#ros2 lifecycle set /slam configure
#ros2 lifecycle set /slam activate

#Bunların hepsini takma isim altında da çalıştırabilirsiniz

def generate_launch_description():
    pkg = get_package_share_directory('catheter_slam')
    
    # Dosya yolları
    slam_config = os.path.join(pkg, 'config', 'slam.yaml') 
    rviz_config = os.path.join(pkg, 'config', 'catheter.rviz')

    # 1. Statik Transform: base_link -> laser (Yeni Stil Argümanlar)
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

    # 2. SLAM Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam', # .yaml dosyasının ismiyle aynı olmalı
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

    # 4. Bridge Node
    bridge_node = TimerAction(
        period=5.0,
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