from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mavlink_url',
            default_value='tcp:192.168.2.2:6777',
            description='MAVLink endpoint URL'
        ),
        Node(
            package='my_bridge_pkg',
            executable='ros2_mavlink_bridge',
            name='ros2_mavlink_bridge',
            output='screen',
            arguments=['--mavlink-url', LaunchConfiguration('mavlink_url')]
        )
    ])