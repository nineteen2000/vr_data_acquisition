"""
teleop_core 是纯映射层
VR 数据接收由 pico_vr_bridge 启动；仿真执行由 xcore_sim 启动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_vr_arm_mapping = LaunchConfiguration('use_vr_arm_mapping')
    use_vr_hand_mapping = LaunchConfiguration('use_vr_hand_mapping')
    vr_arm_mapping_config = LaunchConfiguration('vr_arm_mapping_config')
    vr_hand_mapping_config = LaunchConfiguration('vr_hand_mapping_config')

    vr_to_arm_mapper = Node(
        package='teleop_core',
        executable='vr_to_arm_mapper',
        name='vr_to_arm_mapper',
        output='screen',
        parameters=[vr_arm_mapping_config],
        condition=IfCondition(use_vr_arm_mapping),
    )

    vr_to_hand_mapper = Node(
        package='teleop_core',
        executable='vr_to_hand_mapper',
        name='vr_to_hand_mapper',
        output='screen',
        parameters=[vr_hand_mapping_config],
        condition=IfCondition(use_vr_hand_mapping),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_vr_arm_mapping',
            default_value='true',
            description='Start VR-to-arm ArmServoCommand mapper',
        ),
        DeclareLaunchArgument(
            'use_vr_hand_mapping',
            default_value='true',
            description='Start VR-to-hand HandCommand mapper',
        ),
        DeclareLaunchArgument(
            'vr_arm_mapping_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('teleop_core'),
                'config',
                'vr_arm_mapping.yaml',
            ]),
            description='YAML config for VR-to-arm mapper',
        ),
        DeclareLaunchArgument(
            'vr_hand_mapping_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('teleop_core'),
                'config',
                'vr_hand_mapping.yaml',
            ]),
            description='YAML config for VR-to-hand mapper',
        ),
        vr_to_arm_mapper,
        vr_to_hand_mapper,
    ])
