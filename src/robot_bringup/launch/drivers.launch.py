"""硬件启动:Orbbec camera + LHandPro hands.  /lhandpro_service  /hand_command  /hand_state"""

import shlex
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


CAMERA_DEFAULTS = {
    'wrist': {
        'enabled': True,
        'launch_file': 'gemini_301_series.launch.py',
        'name': 'wrist_camera',
        'serial_number': '',
        'usb_port': '',
        'config_file': 'gemini305_dual_color.yaml',
    },
    'global': {
        'enabled': True,
        'launch_file': 'gemini_330_series.launch.py',
        'name': 'global_camera',
        'serial_number': '',
        'usb_port': '',
        'config_file': 'camera_params.yaml',
    },
}


def _camera_defaults():
    config_path = Path(__file__).resolve().parent.parent / 'config' / 'system.yaml'
    try:
        with config_path.open('r', encoding='utf-8') as handle:
            configured = (yaml.safe_load(handle) or {}).get('system', {}).get('cameras', {})
    except OSError:
        configured = {}

    result = {}
    for role, defaults in CAMERA_DEFAULTS.items():
        result[role] = {**defaults, **(configured.get(role, {}) or {})}
    return result


def _bool_default(value):
    return 'true' if bool(value) else 'false'


def _find_orbbec_setup():
    current = Path(__file__).resolve()
    for parent in current.parents:
        setup_path = parent / 'install/setup.bash'
        if setup_path.exists() and (parent / 'install/orbbec_camera').exists():
            return setup_path
    return None


def _resolve_orbbec_config_path(setup_path, config_file):
    if not config_file or Path(config_file).is_absolute():
        return config_file

    installed_config = setup_path.parent / 'orbbec_camera/share/orbbec_camera/config' / config_file
    if installed_config.exists():
        return str(installed_config)
    return config_file


def _start_orbbec_camera(context, role, *args, **kwargs):
    setup_path = _find_orbbec_setup()
    if setup_path is None:
        raise RuntimeError(
            'Orbbec install setup.bash not found. Build src/vendor_ros/OrbbecSDK_ROS2 first '
            'so install/orbbec_camera is available.'
        )

    camera_launch_file = LaunchConfiguration(f'{role}_camera_launch_file').perform(context)
    camera_name = LaunchConfiguration(f'{role}_camera_name').perform(context)
    serial_number = LaunchConfiguration(f'{role}_camera_serial_number').perform(context)
    usb_port = LaunchConfiguration(f'{role}_camera_usb_port').perform(context)
    config_file = LaunchConfiguration(f'{role}_camera_config_file').perform(context)
    both_cameras_enabled = all(
        LaunchConfiguration(f'use_{camera_role}_camera').perform(context).lower() == 'true'
        for camera_role in CAMERA_DEFAULTS
    )
    if both_cameras_enabled and not (serial_number or usb_port):
        raise RuntimeError(
            f'{role}_camera requires serial_number or usb_port when both cameras are enabled. '
            'Set it in robot_bringup/config/system.yaml.'
        )

    device_num = '2' if both_cameras_enabled else '1'
    config_file = _resolve_orbbec_config_path(setup_path, config_file)
    command = (
        f'source {shlex.quote(str(setup_path))}; '
        f'exec ros2 launch orbbec_camera {shlex.quote(camera_launch_file)} '
        f'camera_name:={shlex.quote(camera_name)} '
        f'serial_number:={shlex.quote(serial_number)} '
        f'usb_port:={shlex.quote(usb_port)} '
        f'device_num:={device_num} '
        f'config_file_path:={shlex.quote(config_file)}'
    )

    return [
        ExecuteProcess(
            cmd=['bash', '-lc', command],
            output='screen',
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    camera_defaults = _camera_defaults()
    use_wrist_camera = LaunchConfiguration('use_wrist_camera')
    use_global_camera = LaunchConfiguration('use_global_camera')
    use_lhandpro_hands = LaunchConfiguration('use_lhandpro_hands')
    interface_names = LaunchConfiguration('interface_names')
    command_topic = LaunchConfiguration('command_topic')
    hand_state_topic = LaunchConfiguration('hand_state_topic')
    service_namespace = LaunchConfiguration('service_namespace')

    wrist_camera_launch = OpaqueFunction(
        function=lambda context, *args, **kwargs: _start_orbbec_camera(context, 'wrist', *args, **kwargs),
        condition=IfCondition(use_wrist_camera),
    )
    global_camera_launch = OpaqueFunction(
        function=lambda context, *args, **kwargs: _start_orbbec_camera(context, 'global', *args, **kwargs),
        condition=IfCondition(use_global_camera),
    )

    lhandpro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lhandpro_gateway'),
                'launch',
                'lhandpro_gateway.launch.py',
            ])
        ),
        launch_arguments={
            'interface_names': interface_names,
            'command_topic': command_topic,
            'hand_state_topic': hand_state_topic,
            'service_namespace': service_namespace,
        }.items(),
        condition=IfCondition(use_lhandpro_hands),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_wrist_camera',
            default_value=_bool_default(camera_defaults['wrist']['enabled']),
            description='Start the Gemini 305 wrist camera',
        ),
        DeclareLaunchArgument(
            'use_global_camera',
            default_value=_bool_default(camera_defaults['global']['enabled']),
            description='Start the Gemini 336L global camera',
        ),
        DeclareLaunchArgument(
            'use_lhandpro_hands',
            default_value='true',
            description='Start LHandPro service, command adapter, and state publisher',
        ),
        DeclareLaunchArgument(
            'wrist_camera_launch_file',
            default_value=str(camera_defaults['wrist']['launch_file']),
            description='Gemini 305 Orbbec launch file',
        ),
        DeclareLaunchArgument(
            'wrist_camera_name',
            default_value=str(camera_defaults['wrist']['name']),
            description='Gemini 305 ROS namespace',
        ),
        DeclareLaunchArgument(
            'wrist_camera_serial_number',
            default_value=str(camera_defaults['wrist']['serial_number']),
            description='Gemini 305 serial number; set this or usb_port before dual-camera collection',
        ),
        DeclareLaunchArgument(
            'wrist_camera_usb_port',
            default_value=str(camera_defaults['wrist']['usb_port']),
            description='Gemini 305 USB port; set this or serial_number before dual-camera collection',
        ),
        DeclareLaunchArgument(
            'wrist_camera_config_file',
            default_value=str(camera_defaults['wrist']['config_file']),
            description='Gemini 305 Orbbec parameter file',
        ),
        DeclareLaunchArgument(
            'global_camera_launch_file',
            default_value=str(camera_defaults['global']['launch_file']),
            description='Gemini 336L Orbbec launch file',
        ),
        DeclareLaunchArgument(
            'global_camera_name',
            default_value=str(camera_defaults['global']['name']),
            description='Gemini 336L ROS namespace',
        ),
        DeclareLaunchArgument(
            'global_camera_serial_number',
            default_value=str(camera_defaults['global']['serial_number']),
            description='Gemini 336L serial number; set this or usb_port before dual-camera collection',
        ),
        DeclareLaunchArgument(
            'global_camera_usb_port',
            default_value=str(camera_defaults['global']['usb_port']),
            description='Gemini 336L USB port; set this or serial_number before dual-camera collection',
        ),
        DeclareLaunchArgument(
            'global_camera_config_file',
            default_value=str(camera_defaults['global']['config_file']),
            description='Gemini 336L Orbbec parameter file',
        ),
        DeclareLaunchArgument(
            'interface_names',
            default_value='enp0s31f6,enx6c1ff714c4ec',
            description='Comma-separated EtherCAT interfaces for left and right hands',
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/hand_command',
            description='Unified hand command topic',
        ),
        DeclareLaunchArgument(
            'hand_state_topic',
            default_value='/hand_state',
            description='Unified hand state topic',
        ),
        DeclareLaunchArgument(
            'service_namespace',
            default_value='/lhandpro_service',
            description='Namespace where LHandPro services are available',
        ),
        wrist_camera_launch,
        TimerAction(period=2.0, actions=[global_camera_launch]),
        lhandpro_launch,
    ])
