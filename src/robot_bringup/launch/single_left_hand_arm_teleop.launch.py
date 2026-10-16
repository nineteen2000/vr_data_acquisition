"""Single left LHandPro hand + left xCore arm VR teleoperation entrypoint."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _load_bringup_config(filename):
    config_path = Path(get_package_share_directory('robot_bringup')) / 'config' / filename
    with config_path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _bool_default(value):
    return 'true' if bool(value) else 'false'


def _first_hand_interface(network_config):
    interface_names = network_config.get('lhandpro_interface_names', [])
    if interface_names:
        return str(interface_names[0])
    return 'enp0s31f6'


def generate_launch_description():
    system_config = _load_bringup_config('system.yaml').get('system', {})
    network_config = _load_bringup_config('network.yaml').get('network', {})
    camera_config = system_config.get('cameras', {})

    use_wrist_camera = LaunchConfiguration('use_wrist_camera')
    use_global_camera = LaunchConfiguration('use_global_camera')
    use_lhandpro_hands = LaunchConfiguration('use_lhandpro_hands')
    use_pico_vr = LaunchConfiguration('use_pico_vr')
    use_teleop_core = LaunchConfiguration('use_teleop_core')
    use_xcore_arms = LaunchConfiguration('use_xcore_arms')
    lhandpro_interface_name = LaunchConfiguration('lhandpro_interface_name')

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'drivers_single_hand.launch.py',
            ])
        ),
        launch_arguments={
            'use_wrist_camera': use_wrist_camera,
            'use_global_camera': use_global_camera,
            'use_lhandpro_hands': use_lhandpro_hands,
            'interface_names': lhandpro_interface_name,
        }.items(),
    )

    pico_vr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pico_vr_bridge'),
                'launch',
                'pico_vr_bridge.launch.py',
            ])
        ),
        condition=IfCondition(use_pico_vr),
    )

    teleop_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('teleop_core'),
                'launch',
                'teleop_core.launch.py',
            ])
        ),
        condition=IfCondition(use_teleop_core),
    )

    xcore_arms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xcore_gateway'),
                'launch',
                'xcore_teleop.launch.py',
            ])
        ),
        launch_arguments={'arms': 'left'}.items(),
        condition=IfCondition(use_xcore_arms),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_wrist_camera',
            default_value=_bool_default((camera_config.get('wrist') or {}).get('enabled', True)),
            description='Start the Gemini 305 wrist camera',
        ),
        DeclareLaunchArgument(
            'use_global_camera',
            default_value=_bool_default((camera_config.get('global') or {}).get('enabled', True)),
            description='Start the Gemini 336L global camera',
        ),
        DeclareLaunchArgument(
            'use_lhandpro_hands',
            default_value=_bool_default(system_config.get('use_lhandpro_hands', True)),
            description='Start the single left LHandPro driver stack',
        ),
        DeclareLaunchArgument(
            'use_pico_vr',
            default_value=_bool_default(system_config.get('use_pico_vr', True)),
            description='Start PICO VR input bridge',
        ),
        DeclareLaunchArgument(
            'use_teleop_core',
            default_value=_bool_default(system_config.get('use_teleop_core', True)),
            description='Start VR-to-arm and VR-to-hand mappers',
        ),
        DeclareLaunchArgument(
            'use_xcore_arms',
            default_value=_bool_default(system_config.get('use_xcore_arms', True)),
            description='Start left xCore arm interface',
        ),
        DeclareLaunchArgument(
            'lhandpro_interface_name',
            default_value=_first_hand_interface(network_config),
            description='EtherCAT interface name for the left LHandPro hand',
        ),
        drivers,
        pico_vr,
        teleop_core,
        xcore_arms,
    ])
