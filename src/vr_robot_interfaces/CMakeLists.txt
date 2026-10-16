cmake_minimum_required(VERSION 3.8)
project(vr_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmServoCommand.msg"
  "msg/HandCommand.msg"
  "msg/HandState.msg"
  "msg/VrControllerState.msg"
  "srv/CalculateFK.srv"
  "srv/GetRobotInfo.srv"
  "srv/JointMove.srv"
  "srv/KineInverse7.srv"
  "srv/LinearMove.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
