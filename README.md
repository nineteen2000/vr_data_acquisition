# VR 遥操作与数据采集

基于 ROS 2 的 PICO VR 遥操作项目，用于控制 ROKAE AR5 系列机械臂和 LHandPro 灵巧手，并录制相机与机器人数据，为 LeRobot / ACT 模仿学习准备数据集。

当前以**单左臂 + 左手**为主：左臂真机遥操作已打通，仍在优化跟随性、IK 连续性和限幅。仓库包含双臂入口与仿真包，但双臂协同、真 bag 到 LeRobot 的端到端验证及 ACT 训练部署仍属于后续工作。

## 系统链路

```text
PICO App（TCP 8018）
  -> pico_vr_bridge -> /vr_controller_state
     -> teleop_core/vr_to_arm_mapper -> /left_arm/arm_servo_command
        -> xcore_gateway -> SDK IK -> ServoJ -> 左臂
     -> teleop_core/vr_to_hand_mapper -> /hand_command
        -> lhandpro_gateway -> LHandPro EtherCAT 服务 -> 左手

Gemini 305 腕部双彩色 + Gemini 336L 全局彩色
  + 机器人状态 / 执行确认话题
  -> rosbag -> staging -> LeRobot dataset（10 Hz）
```

## 目录

| 路径                                   | 用途                            |
| ------------------------------------ | ----------------------------- |
| `src/robot_bringup/`                 | 系统启动入口、子系统开关、相机与网络配置          |
| `src/pico_vr_bridge/`                | 接收 PICO TCP 文本并解析手柄状态         |
| `src/teleop_core/`                   | VR 到机械臂位姿、灵巧手预设的映射            |
| `src/xcore_gateway/`                 | xCoreSDK 真机接口、IK、ServoJ、状态与诊断 |
| `src/lhandpro_gateway/`              | 手部命令适配、执行确认和状态发布              |
| `src/vr_robot_interfaces/`           | 项目共用 ROS 消息与服务                |
| `src/xcore_sim/`                     | 机械臂仿真入口                       |
| `src/vendor_ros/`、`src/third_party/` | 厂商 ROS 驱动与 SDK                |
| `scripts/`                           | rosbag 抽取、LeRobot 数据集构建和校验    |
| `tests/`                             | 数据对齐与转换逻辑测试                   |
| `docs/`                              | 硬件手册、模型及驱动说明                  |

## 环境与构建

首次检出需安装 Git LFS，以获取仓库中的厂商 SDK 库：

```bash
git lfs install
git clone https://github.com/nineteen2000/vr_data_acquisition.git
cd vr_data_acquisition
git lfs pull
```

后续开发默认提交到 GitHub 的 `origin/main`。构建产物、运行日志、采集数据、调试符号和本地操作笔记不纳入版本管理。

以下命令在 Linux / ROS 2 Humble 环境、项目根目录执行。ROS 节点及 rosbag 抽取使用系统 Python 3.10；LeRobot 构建与校验使用独立 Python 3.12 环境。

运行前准备好 ROS 2 Humble、colcon、C++17 编译工具，以及对应平台的 xCoreSDK 和 LHandProLib。xCoreSDK 路径由 `src/xcore_gateway/CMakeLists.txt` 指定，LHandPro 服务依赖其 `thirdparty/` 中的厂商库。部分厂商目录被 Git 忽略，新检出环境需另行准备，不能仅依赖仓库中的构建产物。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_bringup lhandpro_service
source install/setup.bash
```

相机还需构建 `orbbec_camera`、`orbbec_camera_msgs` 和 `orbbec_description`，并使本工作区存在 `install/orbbec_camera`。当前 `src/vendor_ros/orbbec_305/` 带有 `COLCON_IGNORE`，默认构建会跳过它；启用相机前需单独准备可构建的 Orbbec 驱动，再构建这些包。

LHandPro EtherCAT 服务需要网卡访问权限。构建后按需设置；重建可执行文件后可能需要重新设置：

```bash
sudo setcap cap_net_raw,cap_net_admin+ep \
  install/lhandpro_service/lib/lhandpro_service/lhandpro_service
```

相关库加载和权限说明见 [LHandPro 驱动说明](docs/lhandpro_ros2_bringup_changes.md)。

## 启动前配置

| 配置文件                                                                   | 需要确认的内容                                                            |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------ |
| [system.yaml](src/robot_bringup/config/system.yaml)                    | 子系统开关、相机角色、序列号与 USB 端口                                             |
| [network.yaml](src/robot_bringup/config/network.yaml)                  | bringup 使用的 EtherCAT 网卡默认值                                         |
| [pico\_vr\_bridge.yaml](src/pico_vr_bridge/config/pico_vr_bridge.yaml) | TCP 监听地址与端口，默认 `0.0.0.0:8018`                                      |
| [xcore\_arms.yaml](src/xcore_gateway/config/xcore_arms.yaml)           | `/left_arm/xcore_robot_interface` 下的控制器 IP、本机 IP、坐标映射、缩放、工作空间和伺服限幅 |
| [vr\_arm\_mapping.yaml](src/teleop_core/config/vr_arm_mapping.yaml)    | 机械臂映射模式、扳机阈值及左右臂独立话题                                               |
| [vr\_hand\_mapping.yaml](src/teleop_core/config/vr_hand_mapping.yaml)  | 手柄按键到手部预设的映射                                                       |

真机链路使用 `output_mode: pose`，坐标转换和运动缩放由 xCore gateway 执行。当前左臂 YAML 启用的平移、旋转缩放均为 `0.4`；修改参数时以实际 YAML 为准。

PICO App 的目标地址应设为运行 bridge 的电脑 IP，端口与 bridge 配置一致。相机序列号、USB 端口和网卡名均需按实际设备修改；同时启用两台相机时，每台至少配置一个设备标识。

## 单左臂与左手遥操作

每个新终端先加载 ROS 和工作区环境，再启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup single_left_hand_arm_teleop.launch.py
```

该入口默认启动两台相机、单左手、PICO bridge、映射节点和左臂接口。未准备相机时可关闭相机：

```bash
ros2 launch robot_bringup single_left_hand_arm_teleop.launch.py \
  use_wrist_camera:=false use_global_camera:=false
```

可通过 `lhandpro_interface_name:=实际网卡名` 覆盖左手网卡。机械臂默认 `connect_on_start: false`；启动节点后，确认工作空间、初始姿态及现场安全条件，再逐条连接、切换自动模式、上电并启用伺服，每步检查返回的 `success`：

```bash
ros2 service call /left_arm/connect std_srvs/srv/Trigger '{}'
ros2 service call /left_arm/set_operate_mode std_srvs/srv/SetBool '{data: true}'
ros2 service call /left_arm/set_power std_srvs/srv/SetBool '{data: true}'
ros2 service call /left_arm/servo_move_enable std_srvs/srv/SetBool '{data: true}'
```

左手柄扳机按下时激活左臂跟随，释放后停止跟随；X / Y 分别映射左手张开 / 闭合。首次测试采用低速、小幅动作，持续观察 IK、工作空间和 `[step-limited]` 诊断。当前没有完整避障功能，软件停止不能替代硬件急停。

```bash
ros2 topic echo /left_arm/status
ros2 topic hz /vr_controller_state
ros2 topic hz /left_arm/joint_states

# 请求停止机械臂运动并退出伺服
ros2 service call /left_arm/stop std_srvs/srv/Trigger '{}'
```

## 数据采集与 LeRobot 转换

首版数据为三路 RGB、7 维机械臂关节和 6 维手部电机位置，state / action 均为 13 维。正式 action 使用 `/left_arm/debug_servo_sent_target` 和 `/hand_applied_command`；它们表示 SDK / 服务调用成功后的命令确认，实际状态仍以状态话题为准。

先录制 10–30 秒短 bag，确认话题有数据。开始录制后明确发送一次初始手部预设，保证手部 action 有初始值。下面命令持续录制，使用 Ctrl+C 正常结束：

```bash
mkdir -p /home/lc/datasets/bags
ros2 bag record -o /home/lc/datasets/bags/smoke_000 \
  /wrist_camera/left_color/image_raw /wrist_camera/left_color/camera_info \
  /wrist_camera/right_color/image_raw /wrist_camera/right_color/camera_info \
  /global_camera/color/image_raw /global_camera/color/camera_info \
  /left_arm/joint_states /left_arm/cur_tcp_pose /left_arm/status \
  /left_arm/debug_servo_sent_target /left_arm/debug_servo_target \
  /left_arm/debug_raw_ik_target /left_arm/debug_servo_error \
  /hand_state /hand_applied_command /hand_command /vr_controller_state /rosout
```

转换分两个 Python 环境执行。LeRobot 需另行准备于 `src/lerobot/`，并安装到该目录的 Python 3.12 虚拟环境；该目录被 Git 忽略。

```bash
# 系统 ROS Python 3.10：抽取 rosbag 到 staging
python3 scripts/extract_rosbag_episode.py \
  --bag /home/lc/datasets/bags/smoke_000 \
  --episode-id smoke_000 --task single_left_arm_lhandpro_smoke

# LeRobot Python 3.12：构建并校验 smoke 数据集
src/lerobot/.venv/bin/python scripts/build_lerobot_dataset.py \
  --mode smoke --episode smoke_000
src/lerobot/.venv/bin/python scripts/validate_lerobot_dataset.py \
  --root /home/lc/datasets/ar5_lhandpro_v1_smoke \
  --repo-id local/ar5_lhandpro_v1_smoke
```

默认 staging 位于 `/home/lc/datasets/ar5_lhandpro_v1_staging`。其他机器可通过抽取脚本的 `--output-root`、构建脚本的 `--staging-root` / `--root` 及校验脚本的 `--root` 指定目录。构建不会覆盖已有数据集，重跑需先备份或选择新的输出目录。

检查 staging 的 `episode_report.json` 和数据集的 `conversion_report.json`，确认保留帧数、丢帧原因及时间偏差。当前转换以 bag 时间戳对齐，相机没有硬件同步。缺少执行确认话题的旧 bag 只能用于 smoke；正式模式使用 `--mode formal`，默认输出 `/home/lc/datasets/ar5_lhandpro_v1`，应在短 bag 转换和质量验证通过后使用。

数据转换公共逻辑可单独验证：

```bash
python3 -m pytest tests/test_rosbag_lerobot_common.py
```

该测试需要 NumPy 和 pytest，不代替真机、真实 rosbag 或 LeRobot 环境的端到端验证。
