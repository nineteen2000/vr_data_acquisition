"""双实机 xCoreSDK gateway 入口。

左右臂各创建一个 ``xcore_robot_interface``，通过 namespace、控制器 IP、命令
topic 和 ``arm_name`` 四层隔离。该文件不启动 MoveIt、ros2_control 或 VR mapper；
上游应分别发布 ``/left_arm/arm_servo_command`` 和
``/right_arm/arm_servo_command``。

两个节点共享本机网卡，因此真机联调前必须确认本机 IP、两台控制器 IP、实时
周期和网络带宽。``connect_on_start`` 默认关闭，避免启动 launch 即给硬件上电。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def arm_node(name, robot_ip, command_topic):
    """构造一侧实机节点；返回值只是 launch action，不在此处连接硬件。"""

    # 节点参数 command_topic 保持统一根名称，再由 remap 绑定到左右私有输入。
    # accept_empty_arm_name=False 可防止空 arm_name 命令被两侧同时接受。
    config_file = PathJoinSubstitution([
        FindPackageShare('xcore_gateway'),
        'config',
        'xcore_arms.yaml',
    ])

    return Node(
        package='xcore_gateway',
        executable='xcore_robot_interface',
        namespace=f'/{name}_arm',
        name='xcore_robot_interface',
        output='screen',
        parameters=[config_file, {
            'arm_name': name,
            'robot_ip': robot_ip,
            'local_ip': LaunchConfiguration('local_ip'),
            'command_topic': '/arm_servo_command',
            'connect_on_start': ParameterValue(LaunchConfiguration('connect_on_start'), value_type=bool),
            'state_period_s': ParameterValue(LaunchConfiguration('state_period_s'), value_type=float),
            'servo_period_s': ParameterValue(LaunchConfiguration('servo_period_s'), value_type=float),
            'servo_lookahead_s': ParameterValue(LaunchConfiguration('servo_lookahead_s'), value_type=float),
            'servo_kp': ParameterValue(LaunchConfiguration('servo_kp'), value_type=float),
            'servo_max_step_rad': ParameterValue(LaunchConfiguration('servo_max_step_rad'), value_type=float),
            'servo_command_timeout_s': ParameterValue(LaunchConfiguration('servo_command_timeout_s'), value_type=float),
            'rt_network_tolerance': ParameterValue(LaunchConfiguration('rt_network_tolerance'), value_type=int),
            'cartesian_input_unit': LaunchConfiguration('cartesian_input_unit'),
            'arm_servo_input_unit': LaunchConfiguration('arm_servo_input_unit'),
            'cartesian_output_unit': LaunchConfiguration('cartesian_output_unit'),
            'accept_empty_arm_name': False,
        }],
        # 左右节点最终分别订阅 /left_arm/... 与 /right_arm/...。
        remappings=[('/arm_servo_command', command_topic)],
        condition=IfCondition(LaunchConfiguration('use_sdk_interface')),
    )


def generate_launch_description():
    # 两节点可同启，但 connect/power/servo_enable 仍应按臂分别执行并验收。
    return LaunchDescription([
        DeclareLaunchArgument('use_sdk_interface', default_value='true'),
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.1.160'),
        DeclareLaunchArgument('right_robot_ip', default_value='10.0.2.164'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.3'),
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
        arm_node('left', LaunchConfiguration('left_robot_ip'), '/left_arm/arm_servo_command'),
        arm_node('right', LaunchConfiguration('right_robot_ip'), '/right_arm/arm_servo_command'),
    ])
