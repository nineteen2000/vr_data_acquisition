cmake_minimum_required(VERSION 3.8)
project(xcore_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vr_robot_interfaces REQUIRED)

# 仿真适配器：只面向 MoveIt/ros2_control fake hardware，不连接 xCoreSDK。
add_executable(ar5_arm_gateway
  src/ar5_arm_gateway.cpp
)

target_include_directories(ar5_arm_gateway PRIVATE
  include
)

ament_target_dependencies(ar5_arm_gateway
  geometry_msgs
  moveit_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  trajectory_msgs
  vr_robot_interfaces
)

install(TARGETS ar5_arm_gateway
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
