"""Launch PICO VR input bridge nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pico_vr_bridge'),
                'config',
                'pico_vr_bridge.yaml',
            ]),
            description='YAML config for PICO VR bridge nodes',
        ),
        Node(
            package='pico_vr_bridge',
            executable='tcp_server',
            name='vr_socket_server',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='pico_vr_bridge',
            executable='raw_parser',
            name='raw_vr_parser',
            output='screen',
            parameters=[config_file],
        ),
    ])
