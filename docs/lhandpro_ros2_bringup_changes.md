# LHandPro ROS2 Bringup Changes

Date: 2026-05-22

## Root Cause

- `LHandProLib_EtherCAT_Test` and `ros2 launch lhandpro_service lhandpro.launch.py` both failed before SDK initialization.
- `strace` showed `socket(AF_PACKET, SOCK_RAW, 0x88a4) = -1 EPERM`, which means the EtherCAT raw socket lacked `cap_net_raw` permission.
- After capabilities are added, Linux secure-exec ignores `LD_LIBRARY_PATH`; ROS Humble and workspace libraries must be available from RUNPATH or `ldconfig`.
- The ROS2 service also had follow-on risks:
  - EtherCAT interface index was hard-coded in source.
  - The selected index was used before bounds checking.
  - The SDK `std::function` RPDO callback was a local variable, while the SDK header requires the function object to outlive `LHandProLib`.
  - CMake could resolve `libLHandProLib.so` from `/usr/local/lib` instead of the package `thirdparty/lib`.

## Code Changes

- `lhandpro_service` launch now accepts `channel_index`, defaulting to `1`.
- `HandControlService` declares the `channel_index` ROS parameter and checks it against scanned interfaces before use.
- The RPDO callback is stored as a `HandControlService` member so its lifetime matches the service node.
- `lhandpro_service` CMake now resolves `libLHandProLib.so` only from `thirdparty/lib`.
- The installed service executable has RUNPATH entries for package-local SDK libraries, ROS Humble libraries, and generated interface libraries.
- A small CTest-based regression test records these constraints without requiring hardware.

## Runtime Setup

Run after building:

```bash
sudo sh -c 'printf "%s\n" /opt/ros/humble/lib > /etc/ld.so.conf.d/ros2_humble.conf'
sudo cp src/lhandpro_ws_src/src/ros2_lhandpro.conf /etc/ld.so.conf.d/
sudo ldconfig

sudo setcap cap_net_raw,cap_net_admin+ep \
  src/lhandpro_ws_src/install/lhandpro_service/lib/lhandpro_service/lhandpro_service

sudo setcap cap_net_raw,cap_net_admin+ep \
  src/LHandProLib-API-Linux-20250928/x86_64/bin/LHandProLib_EtherCAT_Test

sudo setcap cap_net_raw,cap_net_admin+ep \
  src/lhandpro_ws_src/src/lhandpro_service/thirdparty/bin/LHandProLib_EtherCAT_Test
```

Confirm:

```bash
getcap src/lhandpro_ws_src/install/lhandpro_service/lib/lhandpro_service/lhandpro_service
getcap src/LHandProLib-API-Linux-20250928/x86_64/bin/LHandProLib_EtherCAT_Test
getcap src/lhandpro_ws_src/src/lhandpro_service/thirdparty/bin/LHandProLib_EtherCAT_Test
```

If initialization still fails, use root `strace` to distinguish permission issues from missing EtherCAT responses:

```bash
cd src/LHandProLib-API-Linux-20250928/x86_64/bin
printf "1\n" | sudo strace -f -e trace=socket,bind,setsockopt,ioctl,sendto,recvfrom ./LHandProLib_EtherCAT_Test
```

`socket(AF_PACKET, SOCK_RAW, ...) = 3` followed by repeated `recvfrom(...)=EAGAIN` means raw socket permission is fixed but no EtherCAT slave response is reaching the selected VM interface.

## Verification Commands

```bash
source /opt/ros/humble/setup.bash
cd src/lhandpro_ws_src
colcon build --packages-select lhandpro_service sequence_demo_cpp --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/lhandpro_service -R test_lhandpro_service_config --output-on-failure
source install/setup.bash
ros2 launch lhandpro_service lhandpro.launch.py channel_index:=1
```

Only run the sequence demo after the service reports a successful EtherCAT connection:

```bash
ros2 launch sequence_demo_cpp sequence.launch.py
```

## Rollback

- Revert the source changes in `src/lhandpro_ws_src/src/lhandpro_service`.
- Remove this document and the regression test.
- Remove capabilities with:

```bash
sudo setcap -r <executable>
```
