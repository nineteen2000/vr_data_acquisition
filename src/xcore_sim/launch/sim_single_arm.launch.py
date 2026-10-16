"""
只用URDF/MoveIt 配置与ros2_control fake hardware 验证上层算法:VR 相对位姿、坐标变换、IK、关节
步长限制和 trajectory 输出。PICO bridge 与 teleop_core 映射层需要单独启动。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_name = LaunchConfiguration('arm_name')
    use_sim_hardware = LaunchConfiguration('use_sim_hardware')

    sim_config = PathJoinSubstitution([
        FindPackageShare('xcore_sim'),
        'config',
        'sim_arms.yaml',
    ])

    # 下游基础设施：创建 robot_state_publisher、controller_manager、MoveIt、
    # /compute_ik、trajectory controller 和 RViz。fake hardware 会立即接受轨迹，
    # 不包含真实机械臂的网络抖动、动力学、碰撞后果或控制器安全状态机。
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

    # 仿真适配与计算层：
    # 新架构主路径为 ArmServoCommand(MODE_JOINT) -> JointTrajectory；
    # MODE_POSE 仍保留 MoveIt IK 对照路径，但不再使用经验 fallback。
    # namespace 只隔离节点名；实际 controller_topic 由 sim_arms.yaml 决定。
    gateway = Node(
        package='xcore_sim',
        executable='ar5_arm_gateway',
        namespace=[arm_name, '_arm'],
        name='ar5_arm_gateway',
        output='screen',
        parameters=[sim_config, {'arm_name': arm_name}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_name',
            default_value='left',
            description='Arm to control: left or right',
        ),
        DeclareLaunchArgument(
            'use_sim_hardware',
            default_value='true',
            description='Use fake hardware (ros2_control mock)',
        ),
        rokae_moveit,
        # TimerAction 是当前的启动时序缓冲，不是 readiness 检查；机器较慢时仍可能
        # 出现 /compute_ik 或 controller 尚未就绪，生产化应改成基于服务/进程状态触发。
        TimerAction(period=10.0, actions=[gateway]),
    ])
