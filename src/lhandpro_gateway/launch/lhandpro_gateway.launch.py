from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_index = LaunchConfiguration('channel_index')
    interface_names = LaunchConfiguration('interface_names')
    command_topic = LaunchConfiguration('command_topic')
    hand_state_topic = LaunchConfiguration('hand_state_topic')
    service_namespace = LaunchConfiguration('service_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_index',
            default_value='1',
            description='Fallback EtherCAT network interface index'
        ),
        DeclareLaunchArgument(
            'interface_names',
            default_value='enp0s31f6,enx6c1ff714c4ec',
            description='Comma-separated EtherCAT interface names for left and right hands'
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/hand_command',
            description='Unified hand command topic'
        ),
        DeclareLaunchArgument(
            'hand_state_topic',
            default_value='/hand_state',
            description='Unified hand state topic'
        ),
        DeclareLaunchArgument(
            'service_namespace',
            default_value='/lhandpro_service',
            description='Namespace where LHandPro services are available'
        ),
        Node(
            package='lhandpro_service',         # 包名
            executable='lhandpro_service',  # 可执行文件名
            name='lhandpro_service', # 节点名称
            namespace='/lhandpro_service',      # 设置命名空间
            parameters=[
                {'channel_index': channel_index},
                {'interface_names': interface_names},
            ],
            output='screen',                 # 输出日志到终端
            emulate_tty=True,                 # 更好地显示日志            
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='lhandpro_gateway',
                    executable='hand_command_adapter_dual',
                    name='hand_command',
                    namespace='',
                    parameters=[{
                        'command_topic': command_topic,
                        'service_namespace': service_namespace,
                        'connected_hand_ids': [0, 1],
                        'left_hand_joint_ids': [1, 2, 3, 4, 5, 6],
                        'right_hand_joint_ids': [7, 8, 9, 10, 11, 12],
                    }],
                    output='screen',
                    emulate_tty=True
                ),
                Node(
                    package='lhandpro_gateway',
                    executable='hand_state_publisher',
                    name='hand_state_publisher',
                    namespace='',
                    parameters=[{
                        'state_topic': hand_state_topic,
                        'command_topic': command_topic,
                        'service_namespace': service_namespace,
                        'connected_hand_ids': [0, 1],
                        'left_hand_joint_ids': [1, 2, 3, 4, 5, 6],
                        'right_hand_joint_ids': [7, 8, 9, 10, 11, 12],
                    }],
                    output='screen',
                    emulate_tty=True
                )
            ]
        )
    ])
