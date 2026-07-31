from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('catheter_slam')
    slam_config = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg, 'config', 'catheter.rviz')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    bridge_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='catheter_slam',
            executable='catheter_bridge',
            name='catheter_bridge',
            output='screen'
        )]
    )

    return LaunchDescription([slam_node, rviz_node, bridge_node])
