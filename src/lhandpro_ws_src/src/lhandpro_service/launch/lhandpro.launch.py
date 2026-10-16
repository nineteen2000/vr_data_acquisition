from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_index = LaunchConfiguration('channel_index')
    interface_name = LaunchConfiguration('interface_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_index',
            default_value='1',
            description='EtherCAT network interface index from scanNetworkInterfaces()'
        ),
        DeclareLaunchArgument(
            'interface_name',
            default_value='',
            description='EtherCAT network interface name, for example enp0s31f6'
        ),
        Node(
            package='lhandpro_service',         # 包名
            executable='lhandpro_service',  # 可执行文件名
            name='lhandpro_service', # 节点名称
            namespace='/lhandpro_service',      # 设置命名空间
            parameters=[
                {'channel_index': channel_index},
                {'interface_name': interface_name}
            ],
            output='screen',                 # 输出日志到终端
            emulate_tty=True,                 # 更好地显示日志            
        )
    ])
