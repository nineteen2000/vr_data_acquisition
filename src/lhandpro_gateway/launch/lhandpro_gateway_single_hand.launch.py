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
            default_value='enp0s31f6',
            description='EtherCAT interface name for the left LHandPro hand'
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
                    executable='hand_command_adapter',
                    name='hand_command',
                    namespace='',
                    parameters=[{
                        'command_topic': command_topic,
                        'service_namespace': service_namespace,
                        'supported_hand_ids': [0],
                    }],
                    output='screen',
                    emulate_tty=True
                ),
                Node(
                    package='lhandpro_gateway',
                    executable='hand_state_publisher_single',
                    name='hand_state_publisher_single',
                    namespace='',
                    parameters=[{
                        'state_topic': hand_state_topic,
                        'command_topic': command_topic,
                        'service_namespace': service_namespace,
                        'supported_hand_ids': [0],
                        'left_hand_joint_ids': [1, 2, 3, 4, 5, 6],
                    }],
                    output='screen',
                    emulate_tty=True
                )
            ]
        )
    ])
