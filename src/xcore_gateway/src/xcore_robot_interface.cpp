/*
 * xCoreSDK 中间层接口适配节点。
 * 1. 连接 xCore 控制器
 * 2. 发布关节角、关节速度、力矩、TCP 位姿
 * 3. 提供 connect、stop、set_power、joint_move、linear_move、FK、IK服务
 * 4. 接收 ArmServoCommand、robot_command_servo_j、robot_command_servo_p
 * 5. 固定周期 servo_timer_ 调用 SDK 的 sendCommand()
 */


#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "xcore_gateway/xcore_robot_interface.hpp"
#include "xcore_gateway/xcore_robot_interface_utils.hpp"

using namespace std::chrono_literals;

namespace xcore_gateway
{
XcoreRobotInterface::XcoreRobotInterface() : Node("xcore_robot_interface")
  {
    // 控制器身份、网络和命令路由。
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.1.160");
    local_ip_ = declare_parameter<std::string>("local_ip", "192.168.1.3");
    arm_name_ = declare_parameter<std::string>("arm_name", "left");
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm_servo_command");
    connect_on_start_ = declare_parameter<bool>("connect_on_start", false);
    // 实时参数
    // 状态发布125Hz
    state_period_s_ = declare_parameter<double>("state_period_s", 0.008);
    // ServoJ 发送周期50Hz；对齐 xCoreSDK servoj_demo.cpp 的 planPeriod=0.02。
    servo_period_s_ = declare_parameter<double>("servo_period_s", 0.02);
    // SDK 伺服前瞻时间，默认 3 * servo_period_s。
    servo_lookahead_s_ = declare_parameter<double>("servo_lookahead_s", 0.06);
    // SDK 伺服增益1.0
    servo_kp_ = declare_parameter<double>("servo_kp", 1.0);
    // SDK 伺服最大步长0.05rad
    servo_max_step_rad_ = declare_parameter<double>("servo_max_step_rad", 0.05);
    // SDK 网络容差80ms
    rt_network_tolerance_ = declare_parameter<int>("rt_network_tolerance", 20);
    // 单位边界：SDK 平移使用米，兼容输入可配置为毫米；姿态始终为弧度。
    cartesian_input_unit_ = declare_parameter<std::string>("cartesian_input_unit", "mm");
    arm_servo_input_unit_ = declare_parameter<std::string>("arm_servo_input_unit", "mm");
    cartesian_output_unit_ = declare_parameter<std::string>("cartesian_output_unit", "m");
    servo_command_timeout_s_ = declare_parameter<double>("servo_command_timeout_s", 0.1);
    accept_empty_arm_name_ = declare_parameter<bool>("accept_empty_arm_name", true);
    // 空关节限位数组表示暂不做软件绝对限位；真机部署前必须提供完整七轴范围。
    joint_limits_min_ = declare_parameter<std::vector<double>>("joint_limits_min", std::vector<double>{});
    joint_limits_max_ = declare_parameter<std::vector<double>>("joint_limits_max", std::vector<double>{});
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());
    // VR MODE_POSE 相对跟随参数：target_pose 按 VR 原始米制位姿处理，不走 mm 输入转换。
    // VR 原始位姿到机器人 base 的 row-major 4x4 方向映射矩阵。
    // 当前 PICO raw: +X 左, +Y 上, +Z 后 -> robot: +Y 左, +Z 上, -X 后。
    vr_rot_ = declare_parameter<std::vector<double>>(
      "vr_rot",
      // 旧 PICO raw(+X 右, +Y 上, +Z 前):
      // {0.0, 0.0, 1.0,
      //  -1.0, 0.0, 0.0,
      //  0.0, 1.0, 0.0}
      {0.0, 0.0, -1.0, -1.0,
       1.0, 0.0, 0.0, 1.0,
       0.0, 1.0, 0.0, 1.0,
       0.0, 0.0, 0.0, 1.0});
    position_scale_ = declare_parameter<double>("position_scale", 1.0);
    rotation_scale_ = declare_parameter<double>("rotation_scale", 0.5);
    workspace_min_ = declare_parameter<std::vector<double>>("workspace_min", std::vector<double>{});
    workspace_max_ = declare_parameter<std::vector<double>>("workspace_max", std::vector<double>{});
    ik_hold_on_fail_ = declare_parameter<bool>("ik_hold_on_fail", true);
    ik_fail_stop_timeout_s_ = declare_parameter<double>("ik_fail_stop_timeout_s", 1.0);
    pose_reference_settle_s_ = declare_parameter<double>("pose_reference_settle_s", 0.3);
    pose_max_linear_speed_mps_ = declare_parameter<double>("pose_max_linear_speed_mps", 0.20);
    pose_max_angular_speed_radps_ = declare_parameter<double>("pose_max_angular_speed_radps", 0.8);

    if (joint_names_.size() != kDof) {
      RCLCPP_WARN(get_logger(), "joint_names must contain 7 joints, fallback to joint1..joint7.");
      joint_names_ = default_joint_names();
    }
    if (state_period_s_ <= 0.0) {
      state_period_s_ = 0.008;
    }
    if (servo_period_s_ <= 0.0) {
      servo_period_s_ = 0.008;
    }
    if (servo_lookahead_s_ <= 0.0) {
      servo_lookahead_s_ = servo_period_s_ * 3.0;
    }
    if (servo_max_step_rad_ <= 0.0) {
      servo_max_step_rad_ = 0.05;
    }
    if ((vr_rot_.size() != 9 && vr_rot_.size() != 16) || !finite_values(vr_rot_)) {
      RCLCPP_WARN(get_logger(), "vr_rot must contain 9 or 16 finite row-major values, use default.");
      vr_rot_ = {
        0.0, 0.0, -1.0, -1.0,
        1.0, 0.0, 0.0, 1.0,
        0.0, 1.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0};
    }
    if (!std::isfinite(position_scale_) || position_scale_ < 0.0) {
      RCLCPP_WARN(get_logger(), "position_scale must be non-negative, use 1.0.");
      position_scale_ = 1.0;
    }
    if (!std::isfinite(rotation_scale_) || rotation_scale_ < 0.0) {
      RCLCPP_WARN(get_logger(), "rotation_scale must be non-negative, use 0.5.");
      rotation_scale_ = 0.5;
    }
    if (!std::isfinite(pose_reference_settle_s_) || pose_reference_settle_s_ < 0.0) {
      RCLCPP_WARN(get_logger(), "pose_reference_settle_s must be non-negative, use 0.3.");
      pose_reference_settle_s_ = 0.3;
    }
    if (!std::isfinite(pose_max_linear_speed_mps_) || pose_max_linear_speed_mps_ < 0.0) {
      RCLCPP_WARN(get_logger(), "pose_max_linear_speed_mps must be non-negative, use 0.20.");
      pose_max_linear_speed_mps_ = 0.20;
    }
    if (!std::isfinite(pose_max_angular_speed_radps_) || pose_max_angular_speed_radps_ < 0.0) {
      RCLCPP_WARN(get_logger(), "pose_max_angular_speed_radps must be non-negative, use 0.8.");
      pose_max_angular_speed_radps_ = 0.8;
    }
    if (workspace_min_.size() != 3 || workspace_max_.size() != 3 ||
      !finite_values(workspace_min_) || !finite_values(workspace_max_))
    {
      workspace_min_.clear();
      workspace_max_.clear();
    }

    current_positions_.assign(kDof, std::numeric_limits<double>::quiet_NaN());
    latest_servo_target_.assign(kDof, std::numeric_limits<double>::quiet_NaN());

    // 发布关节状态
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_vel_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states_vel", 10);
    tcp_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("cur_tcp_pose", 10);
    torque_pub_ = create_publisher<sensor_msgs::msg::JointState>("raw_torque_sensor_val", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("status", 10);
    debug_servo_target_pub_ = create_publisher<sensor_msgs::msg::JointState>("debug_servo_target", 10);
    debug_servo_sent_target_pub_ = create_publisher<sensor_msgs::msg::JointState>("debug_servo_sent_target", 10);
    debug_raw_ik_target_pub_ = create_publisher<sensor_msgs::msg::JointState>("debug_raw_ik_target", 10);
    debug_servo_error_pub_ = create_publisher<sensor_msgs::msg::JointState>("debug_servo_error", 10);

    // 服务端
    connect_srv_ = create_service<std_srvs::srv::Trigger>(
      "connect",
      std::bind(&XcoreRobotInterface::on_connect, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop",
      std::bind(&XcoreRobotInterface::on_stop, this, std::placeholders::_1, std::placeholders::_2));
    get_info_srv_ = create_service<vr_srv::GetRobotInfo>(
      "get_robot_info",
      std::bind(&XcoreRobotInterface::on_get_robot_info, this, std::placeholders::_1, std::placeholders::_2));
    set_power_srv_ = create_service<std_srvs::srv::SetBool>(
      "set_power",
      std::bind(&XcoreRobotInterface::on_set_power, this, std::placeholders::_1, std::placeholders::_2));
    set_operate_mode_srv_ = create_service<std_srvs::srv::SetBool>(
      "set_operate_mode",
      std::bind(&XcoreRobotInterface::on_set_operate_mode, this, std::placeholders::_1, std::placeholders::_2));
    servo_enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "servo_move_enable",
      std::bind(&XcoreRobotInterface::on_servo_enable, this, std::placeholders::_1, std::placeholders::_2));
    joint_move_srv_ = create_service<vr_srv::JointMove>(
      "joint_move",
      std::bind(&XcoreRobotInterface::on_joint_move, this, std::placeholders::_1, std::placeholders::_2));
    linear_move_srv_ = create_service<vr_srv::LinearMove>(
      "linear_move",
      std::bind(&XcoreRobotInterface::on_linear_move, this, std::placeholders::_1, std::placeholders::_2));
    ik_srv_ = create_service<vr_srv::KineInverse7>(
      "kine_inverse7",
      std::bind(&XcoreRobotInterface::on_kine_inverse, this, std::placeholders::_1, std::placeholders::_2));
    fk_srv_ = create_service<vr_srv::CalculateFK>(
      "calculate_fk",
      std::bind(&XcoreRobotInterface::on_calculate_fk, this, std::placeholders::_1, std::placeholders::_2));

    servo_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    command_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions command_sub_options;
    command_sub_options.callback_group = command_callback_group_;

    // robot_command_servo_j/p 暂时没用
    servo_j_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "robot_command_servo_j", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_servo_j(msg); },
      command_sub_options);
    servo_p_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "robot_command_servo_p", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_servo_p(msg); },
      command_sub_options);
    arm_command_sub_ = create_subscription<vr_robot_interfaces::msg::ArmServoCommand>(
      command_topic_, 10,
      [this](const vr_robot_interfaces::msg::ArmServoCommand::SharedPtr msg) { on_arm_servo_command(msg); },
      command_sub_options);

    // 状态发布125hz
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(state_period_s_)),
      [this]() { publish_state(); },
      state_callback_group_);
    // servoj 发布周期需和 setServoJoint 的 ServoJ_T 一致。
    servo_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(servo_period_s_)),
      [this]() { servo_tick(); },
      servo_callback_group_);

    set_status("xCore robot interface started; not connected");
    if (connect_on_start_) {
      std::string message;
      if (!connect_robot(message)) {
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      }
    }
  }

XcoreRobotInterface::~XcoreRobotInterface()
  {
    // 析构优先结束实时会话，再断开控制器，避免退出时保留运动模式。
    stop_servo_session();
    if (robot_) {
      std::error_code ec;
      robot_->disconnectFromRobot(ec);
    }
  }
}  // namespace xcore_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xcore_gateway::XcoreRobotInterface>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
