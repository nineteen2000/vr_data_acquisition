cmake_minimum_required(VERSION 3.8)
project(xcore_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(vr_robot_interfaces REQUIRED)
find_package(Threads REQUIRED)

# xCoreSDK 以 third_party 静态库接入；只有实机 gateway 链接 SDK。
set(XCORE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/xCoreSDK_cpp-v0.7.1/xCoreSDK_cpp-v0.7.1"
)
set(XCORE_SDK_LIB_DIR "${XCORE_SDK_ROOT}/lib/Linux/${CMAKE_SYSTEM_PROCESSOR}")

add_library(xcore_sdk STATIC IMPORTED)
set_target_properties(xcore_sdk PROPERTIES
  IMPORTED_LOCATION "${XCORE_SDK_LIB_DIR}/libxCoreSDK.a"
)

add_library(xmate_model STATIC IMPORTED)
set_target_properties(xmate_model PROPERTIES
  IMPORTED_LOCATION "${XCORE_SDK_LIB_DIR}/libxMateModel.a"
)

# 实机 SDK gateway：真实 AR5L 控制入口。
add_executable(xcore_robot_interface
  src/xcore_robot_interface.cpp
  src/xcore_robot_interface_commands.cpp
  src/xcore_robot_interface_connection.cpp
  src/xcore_robot_interface_pose.cpp
  src/xcore_robot_interface_services.cpp
  src/xcore_robot_interface_servo.cpp
  src/xcore_robot_interface_state.cpp
)

ament_target_dependencies(xcore_robot_interface
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  vr_robot_interfaces
)

target_include_directories(xcore_robot_interface PRIVATE
  include
  "${XCORE_SDK_ROOT}/include"
  "${XCORE_SDK_ROOT}/external"
)

target_link_libraries(xcore_robot_interface
  xcore_sdk
  xmate_model
  Threads::Threads
  dl
)

install(TARGETS xcore_robot_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
