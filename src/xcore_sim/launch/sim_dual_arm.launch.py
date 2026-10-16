"""
测试版:不适合证明双臂独立运动；两侧同时使能时会竞争同一个 fake controller。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def arm_gateway(arm_name, sim_config):

    return Node(
        package='xcore_sim',
        executable='ar5_arm_gateway',
        namespace=f'/{arm_name}_arm',
        name='ar5_arm_gateway',
        output='screen',
        parameters=[sim_config, {'arm_name': arm_name}],
    )


def generate_launch_description():
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')

    sim_config = PathJoinSubstitution([
        FindPackageShare('xcore_sim'),
        'config',
        'sim_arms.yaml',
    ])

    # 这里只创建一套 AR5L 模型和 controller_manager，这是当前“双臂”能力的边界。
    rokae_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rokae_hardware'),
                'launch',
                'rokae_moveit_launch.py',
            ])
        ),
        launch_arguments={
            'use_fake_hardware': use_sim_hardware,
            'robot_type': 'AR5L',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_hardware',
            default_value='true',
            description='Use fake hardware (ros2_control mock)',
        ),
        rokae_moveit,
        # 两个 gateway 的输入分开，但输出 controller 仍相同，不能同时作为独立双臂使用。
        TimerAction(
            period=10.0,
            actions=[
                arm_gateway('left', sim_config),
                arm_gateway('right', sim_config),
            ],
        ),
    ])
