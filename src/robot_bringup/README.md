# robot_bringup

Launch package for the current hardware bringup.

Current scope:

- Orbbec Gemini 305 wrist camera and Gemini 336L global camera
- LHandPro dual EtherCAT hands through `lhandpro_gateway`

Not enabled yet:

- PICO VR bridge
- xCore arms
- full teleoperation state machine

## Start Drivers

```bash
cd ~/projects/VR_Data_Collect
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup drivers.launch.py interface_names:=enp0s31f6,enx6c1ff714c4ec
```

Start both cameras only:

```bash
ros2 launch robot_bringup drivers.launch.py use_lhandpro_hands:=false
```

Start hands only:

```bash
ros2 launch robot_bringup drivers.launch.py use_wrist_camera:=false use_global_camera:=false interface_names:=enp0s31f6,enx6c1ff714c4ec
```

## Check

```bash
ros2 node list
ros2 topic list | grep -E 'camera|hand'
ros2 service list | grep -E 'set_enable|set_position|move_motors'
```

Expected main topics:

```text
/wrist_camera/left_color/image_raw
/wrist_camera/right_color/image_raw
/global_camera/color/image_raw
/global_camera/depth/image_raw
/hand_command
/hand_state
```

## Camera Configuration

The two camera roles are configured in `config/system.yaml`:

- `wrist`: Gemini 305, launched with `gemini_301_series.launch.py` in `/wrist_camera`.
- `global`: Gemini 336L, launched with `gemini_330_series.launch.py` in `/global_camera`.

Set either `serial_number` or `usb_port` for each camera before dual-camera collection. They are intentionally empty by default because device identifiers depend on the connected hardware, and the launch aborts if either identifier is missing while both cameras are enabled. The bringup passes `device_num:=2` to both Orbbec launch files and starts the global camera two seconds after the wrist camera.

Build the checked-in Orbbec driver once before launch:

```bash
colcon build --packages-select orbbec_camera orbbec_camera_msgs orbbec_description robot_bringup
source install/setup.bash
```

## Hand Test

```bash
ros2 topic pub --once /hand_command vr_robot_interfaces/msg/HandCommand "{hand_id: 0, preset_name: open, motor_positions: [], motor_velocities: [], enabled: true}"
ros2 topic pub --once /hand_command vr_robot_interfaces/msg/HandCommand "{hand_id: 1, preset_name: open, motor_positions: [], motor_velocities: [], enabled: true}"
ros2 topic pub --once /hand_command vr_robot_interfaces/msg/HandCommand "{hand_id: 2, preset_name: middle, motor_positions: [], motor_velocities: [], enabled: true}"
```

If LHandPro fails to connect after rebuilding `lhandpro_service`, restore capabilities:

```bash
sudo setcap cap_net_raw,cap_net_admin+ep install/lhandpro_service/lib/lhandpro_service/lhandpro_service
```
