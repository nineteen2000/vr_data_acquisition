"""
单臂 xCoreSDK 实机接口适配入口。
该 launch 只负责真实控制器适配：
xcore_robot_interface 连接真实控制器，通过 xCoreSDK 执行实时/非实时运动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 公共 YAML 提供稳定默认值；launch 参数用于覆盖网络、命名空间和安全时序。
    config_file = PathJoinSubstitution([
        FindPackageShare('xcore_gateway'),
        'config',
        'xcore_arms.yaml',
    ])

    # 进程选择开关。bringup_vendor 只加载供应商描述，不负责本节点的 SDK 连接。
    bringup_vendor = LaunchConfiguration('bringup_vendor')
    use_sdk_interface = LaunchConfiguration('use_sdk_interface')
    # 身份与网络参数。arm_name 参与消息过滤，interface_namespace 隔离 ROS API。
    robot_type = LaunchConfiguration('robot_type')
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
    arm_name = LaunchConfiguration('arm_name')
    interface_namespace = LaunchConfiguration('interface_namespace')
    command_topic = LaunchConfiguration('command_topic')
    # 实机实时控制参数：状态采样周期、ServoJ 周期/前瞻/增益和 watchdog。
    connect_on_start = LaunchConfiguration('connect_on_start')
    state_period_s = LaunchConfiguration('state_period_s')
    servo_period_s = LaunchConfiguration('servo_period_s')
    servo_lookahead_s = LaunchConfiguration('servo_lookahead_s')
    servo_kp = LaunchConfiguration('servo_kp')
    servo_max_step_rad = LaunchConfiguration('servo_max_step_rad')
    servo_command_timeout_s = LaunchConfiguration('servo_command_timeout_s')
    rt_network_tolerance = LaunchConfiguration('rt_network_tolerance')
    cartesian_input_unit = LaunchConfiguration('cartesian_input_unit')
    arm_servo_input_unit = LaunchConfiguration('arm_servo_input_unit')
    cartesian_output_unit = LaunchConfiguration('cartesian_output_unit')

    # 仅加载供应商 robot description。这里明确关闭 vendor driver 和 MoveIt，
    # 避免再创建一套真实硬件控制进程；完整仿真由 xcore_sim 负责。
    vendor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rokae_hardware'),
                'launch',
                'real_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'robot_type': robot_type,
            'robot_ip': robot_ip,
            'local_ip': local_ip,
            'enable_driver': 'false',
            'enable_moveit': 'false',
        }.items(),
        condition=IfCondition(bringup_vendor),
    )

    # 实机路径：相对 topic/service 名会落到 interface_namespace 下。
    # 例如默认 namespace=/left_arm 时，状态为 /left_arm/joint_states，
    # 服务为 /left_arm/connect、/left_arm/servo_move_enable 等。
    sdk_interface_node = Node(
        package='xcore_gateway',
        executable='xcore_robot_interface',
        namespace=interface_namespace,
        name='xcore_robot_interface',
        output='screen',
        parameters=[config_file, {
            'robot_ip': robot_ip,
            'local_ip': local_ip,
            'arm_name': arm_name,
            'command_topic': command_topic,
            'connect_on_start': ParameterValue(connect_on_start, value_type=bool),
            'state_period_s': ParameterValue(state_period_s, value_type=float),
            'servo_period_s': ParameterValue(servo_period_s, value_type=float),
            'servo_lookahead_s': ParameterValue(servo_lookahead_s, value_type=float),
            'servo_kp': ParameterValue(servo_kp, value_type=float),
            'servo_max_step_rad': ParameterValue(servo_max_step_rad, value_type=float),
            'servo_command_timeout_s': ParameterValue(servo_command_timeout_s, value_type=float),
            'rt_network_tolerance': ParameterValue(rt_network_tolerance, value_type=int),
            'cartesian_input_unit': cartesian_input_unit,
            'arm_servo_input_unit': arm_servo_input_unit,
            'cartesian_output_unit': cartesian_output_unit,
            'accept_empty_arm_name': True,
        }],
        condition=IfCondition(use_sdk_interface),
    )

    return LaunchDescription([
        # 默认配置强调“可启动但不误连真机”。
        DeclareLaunchArgument('bringup_vendor', default_value='false'),
        DeclareLaunchArgument('use_sdk_interface', default_value='true'),
        DeclareLaunchArgument('robot_type', default_value='AR5L'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.160'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.3'),
        DeclareLaunchArgument('arm_name', default_value='left'),
        DeclareLaunchArgument('interface_namespace', default_value='/left_arm'),
        DeclareLaunchArgument('command_topic', default_value='/arm_servo_command'),
        DeclareLaunchArgument('connect_on_start', default_value='false'),
        DeclareLaunchArgument('state_period_s', default_value='0.008'),
        DeclareLaunchArgument('servo_period_s', default_value='0.02'),
        DeclareLaunchArgument('servo_lookahead_s', default_value='0.06'),
        DeclareLaunchArgument('servo_kp', default_value='1.0'),
        DeclareLaunchArgument('servo_max_step_rad', default_value='0.05'),
        DeclareLaunchArgument('servo_command_timeout_s', default_value='0.1'),
        DeclareLaunchArgument('rt_network_tolerance', default_value='80'),
        DeclareLaunchArgument('cartesian_input_unit', default_value='mm'),
        DeclareLaunchArgument('arm_servo_input_unit', default_value='mm'),
        DeclareLaunchArgument('cartesian_output_unit', default_value='m'),
        vendor_bringup,
        sdk_interface_node,
    ])
