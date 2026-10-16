# xcore_gateway

ROKAE AR5L xCore gateway for this project.

The default node is `xcore_robot_interface`, a JAKA-style ROS wrapper over the
ROKAE xCoreSDK. It exposes the necessary robot lifecycle, motion, servo and
kinematics interfaces directly in ROS, while still accepting the project-level
`vr_robot_interfaces/msg/ArmServoCommand` topic.

Default behavior is safe for development without robot hardware:
`connect_on_start` and vendor bringup are disabled.

## File Map

| File | Role |
|------|------|
| `CMakeLists.txt` | Builds the real xCoreSDK gateway |
| `package.xml` | ROS2 package metadata and runtime/build dependencies |
| `config/xcore_arms.yaml` | Joint names and limits shared by single-arm and dual-arm launches |
| `launch/xcore_gateway.launch.py` | Single-arm launch; defaults to no hardware connection |
| `launch/xcore_dual_arm.launch.py` | Dual-arm launch with left/right command-topic isolation |
| `src/xcore_robot_interface*.cpp` | Real xCoreSDK gateway for robot state, services, ServoJ, and `ArmServoCommand` |
| `../vr_robot_interfaces/srv/GetRobotInfo.srv` | Controller and SDK information query |
| `../vr_robot_interfaces/srv/JointMove.srv` | Non-RT MOVEJ debug service |
| `../vr_robot_interfaces/srv/LinearMove.srv` | Non-RT MOVEL debug service |
| `../vr_robot_interfaces/srv/KineInverse7.srv` | 7-DOF inverse kinematics service |
| `../vr_robot_interfaces/srv/CalculateFK.srv` | Forward kinematics service |

## Quick Start

```bash
# Single-arm (left arm by default, namespace /left_arm):
ros2 launch xcore_gateway xcore_gateway.launch.py

# Dual-arm (both /left_arm and /right_arm):
ros2 launch xcore_gateway xcore_dual_arm.launch.py
```

With AR5L hardware:

```bash
# Single arm
ros2 launch xcore_gateway xcore_gateway.launch.py \
  connect_on_start:=true \
  robot_ip:=192.168.1.160 \
  local_ip:=192.168.1.3

# Dual arm
ros2 launch xcore_gateway xcore_dual_arm.launch.py \
  connect_on_start:=true \
  left_robot_ip:=192.168.1.160 \
  right_robot_ip:=10.0.2.164 \
  local_ip:=192.168.1.3
```

## JAKA-Compatible Interface

Each arm instance lives in its own namespace following the JAKA convention:

| Arm | Namespace |
|-----|-----------|
| Left  | `/left_arm`  |
| Right | `/right_arm` |

### Services (under arm namespace)

| Service | Type | Description |
|---------|------|-------------|
| `connect` | `std_srvs/Trigger` | Connect to robot controller |
| `stop` | `std_srvs/Trigger` | Stop motion and disable servo |
| `get_robot_info` | `vr_robot_interfaces/srv/GetRobotInfo` | Robot type, version, SDK version |
| `set_power` | `std_srvs/SetBool` | Power on/off |
| `set_operate_mode` | `std_srvs/SetBool` | Automatic/manual mode |
| `servo_move_enable` | `std_srvs/SetBool` | Enable/disable ServoJ mode |
| `joint_move` | `vr_robot_interfaces/srv/JointMove` | Non-RT joint move (MOVEJ) |
| `linear_move` | `vr_robot_interfaces/srv/LinearMove` | Non-RT linear move (MOVEL) |
| `kine_inverse7` | `vr_robot_interfaces/srv/KineInverse7` | Inverse kinematics (7-DOF) |
| `calculate_fk` | `vr_robot_interfaces/srv/CalculateFK` | Forward kinematics |

### Subscriptions (under arm namespace)

| Topic | Type | Description |
|-------|------|-------------|
| `robot_command_servo_j` | `sensor_msgs/JointState` | JAKA-compatible ServoJ (7 joints) |
| `robot_command_servo_p` | `sensor_msgs/JointState` | JAKA-compatible ServoP ([x,y,z,rx,ry,rz,elbow]) |
| `/arm_servo_command` | `vr_robot_interfaces/ArmServoCommand` | Project-level arm servo (MODE_JOINT + MODE_POSE) |

### Publishers (under arm namespace)

| Topic | Type | Description |
|-------|------|-------------|
| `joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, torques |
| `joint_states_vel` | `sensor_msgs/JointState` | Joint velocities only |
| `cur_tcp_pose` | `geometry_msgs/PoseStamped` | Current TCP pose |
| `raw_torque_sensor_val` | `sensor_msgs/JointState` | Joint torques |
| `status` | `std_msgs/String` | Connection, servo, and watchdog status |

## Cartesian Unit Convention

JAKA's VR pipeline expresses Cartesian positions in mm (multiplied by 1000),
while xCoreSDK uses meters/radians. The gateway handles conversion transparently:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cartesian_input_unit` | `"mm"` | Unit for incoming cartesian poses (motion/IK services, servo_p) |
| `arm_servo_input_unit` | `"mm"` | Unit for `ArmServoCommand MODE_POSE` positions |
| `cartesian_output_unit` | `"m"` | Unit for published `cur_tcp_pose` |

Set `cartesian_input_unit:="m"` if your upstream nodes already produce meter-scale
positions.

## ServoJ Safety (Watchdog)

The servo loop includes a command timeout watchdog:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `servo_command_timeout_s` | `0.1` | Max time without new command before emergency servo stop |

When a timeout occurs, the gateway sends a `finished` marker, stops ServoJ,
switches to Idle mode, and disables the servo. This prevents the arm from
continuing to track the last command when VR/PICO disconnects.

## Timing

| Parameter | Default | JAKA Reference |
|-----------|---------|---------------|
| `state_period_s` | `0.008` (8ms) | 8ms in `robot_interface.py:23` |
| `servo_period_s` | `0.02` (20ms) | Matches xCoreSDK ServoJ demo plan period |
| `servo_lookahead_s` | `0.06` (60ms) | 3× servo_period_s |

## Simulation Boundary

MoveIt/ros2_control fake-hardware simulation is no longer launched from this
package. Use `xcore_sim` for simulation, where `ar5_arm_gateway` adapts
`ArmServoCommand` to `JointTrajectory`.
