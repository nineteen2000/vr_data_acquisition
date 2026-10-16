"""
VR 到 xCoreSDK 真机遥操作入口。
启动左臂或双臂 xcore_robot_interface；VR 映射由 teleop_core 启动。
默认不自动连接或上电真实控制器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def arm_node(name, condition):
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
        parameters=[config_file],
        condition=condition,
    )


def generate_launch_description():
    right_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('arms'), "' == 'dual'"]))

    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='left', description='left 或 dual'),
        arm_node(
            'left',
            IfCondition(PythonExpression(["'", LaunchConfiguration('arms'), "' in ['left', 'dual']"])),
        ),
        arm_node(
            'right',
            right_condition,
        ),
    ])
