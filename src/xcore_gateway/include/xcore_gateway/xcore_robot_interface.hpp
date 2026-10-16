#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "vr_robot_interfaces/msg/arm_servo_command.hpp"
#include "vr_robot_interfaces/srv/calculate_fk.hpp"
#include "vr_robot_interfaces/srv/get_robot_info.hpp"
#include "vr_robot_interfaces/srv/joint_move.hpp"
#include "vr_robot_interfaces/srv/kine_inverse7.hpp"
#include "vr_robot_interfaces/srv/linear_move.hpp"
#include "xcore_gateway/vr_pose_mapping.hpp"

#include "rokae/data_types.h"
#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"

namespace xcore_gateway
{

namespace vr_srv = vr_robot_interfaces::srv;

class XcoreRobotInterface : public rclcpp::Node
{
public:
  XcoreRobotInterface();
  ~XcoreRobotInterface() override;

private:
  bool connected() const;
  bool connect_robot(std::string & message);
  bool require_robot(std::string & message);
  bool set_power(bool on, std::string & message);
  bool set_automatic(bool automatic, std::string & message);
  bool prepare_automatic_powered(std::string & message);
  bool enable_servo(bool enable, std::string & message);
  void stop_servo_session();
  void stop_servo_session_locked();
  void on_connect(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_get_robot_info(
    const std::shared_ptr<vr_srv::GetRobotInfo::Request> request,
    std::shared_ptr<vr_srv::GetRobotInfo::Response> response);
  void on_set_power(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void on_set_operate_mode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void on_servo_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void on_joint_move(
    const std::shared_ptr<vr_srv::JointMove::Request> request,
    std::shared_ptr<vr_srv::JointMove::Response> response);
  void on_linear_move(
    const std::shared_ptr<vr_srv::LinearMove::Request> request,
    std::shared_ptr<vr_srv::LinearMove::Response> response);
  void on_kine_inverse(
    const std::shared_ptr<vr_srv::KineInverse7::Request> request,
    std::shared_ptr<vr_srv::KineInverse7::Response> response);
  void on_calculate_fk(
    const std::shared_ptr<vr_srv::CalculateFK::Request> request,
    std::shared_ptr<vr_srv::CalculateFK::Response> response);
  void on_servo_j(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_servo_p(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_arm_servo_command(const vr_robot_interfaces::msg::ArmServoCommand::SharedPtr msg);
  bool lock_pose_reference(const geometry_msgs::msg::Pose & vr_pose, bool reset_lock_time = true);
  void handle_pose_command(const geometry_msgs::msg::Pose & pose);
  bool within_workspace(const tf2::Vector3 & position) const;
  tf2::Vector3 limit_workspace_position(const tf2::Vector3 & position, bool & limited) const;
  void clear_pose_reference();
  void reset_ik_fail_state();
  void reset_cartesian_target_filter();
  void limit_cartesian_target(
    const tf2::Vector3 & desired_position,
    const tf2::Quaternion & desired_quat,
    tf2::Vector3 & limited_position,
    tf2::Quaternion & limited_quat);
  void handle_ik_failure(const std::error_code & ec, const std::string & source);
  void accept_servo_target(const std::vector<double> & target, const std::string & source);
  void publish_debug_servo_target(const std::vector<double> & target);
  void publish_debug_servo_sent_target(const std::vector<double> & target, const rclcpp::Time & stamp);
  bool limit_servo_step(
    const std::vector<double> & target,
    const std::vector<double> & current,
    std::vector<double> & limited_target,
    bool & step_limited);
  void limit_joint_positions(std::vector<double> & target, bool & joint_limited) const;
  void servo_tick();
  void publish_state();
  void publish_status();
  std::vector<double> current_joint_positions_or_sdk();
  bool wait_until_idle(double timeout_s);
  rokae::CartesianPosition to_cartesian(const std::vector<double> & values) const;
  std::vector<double> convert_input_pose(const std::vector<double> & values, const std::string & unit) const;
  std::vector<double> from_cartesian(const rokae::CartesianPosition & cart) const;
  void set_status(const std::string & status);

  // 网络/实时参数、目标缓存和 SDK 资源由单节点串行 executor 管理。
  std::string robot_ip_;
  std::string local_ip_;
  std::string arm_name_;
  std::string command_topic_;
  bool connect_on_start_{false};
  double state_period_s_{0.02};
  double servo_period_s_{0.02};
  double servo_lookahead_s_{0.06};
  double servo_kp_{1.0};
  double servo_max_step_rad_{0.05};
  int rt_network_tolerance_{20};
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> latest_servo_target_;
  bool have_servo_target_{false};
  bool servo_enabled_{false};
  std::string last_status_;
  std::string cartesian_input_unit_{"mm"};
  std::string arm_servo_input_unit_{"mm"};
  std::string cartesian_output_unit_{"m"};
  double servo_command_timeout_s_{0.1};
  bool accept_empty_arm_name_{true};
  std::vector<double> joint_limits_min_;
  std::vector<double> joint_limits_max_;
  std::vector<double> vr_rot_;
  double position_scale_{1.0};
  double rotation_scale_{0.5};
  std::vector<double> workspace_min_;
  std::vector<double> workspace_max_;
  bool ik_hold_on_fail_{true};
  double ik_fail_stop_timeout_s_{1.0};
  double pose_reference_settle_s_{0.3};
  double pose_max_linear_speed_mps_{0.20};
  double pose_max_angular_speed_radps_{0.8};
  rclcpp::Time first_ik_fail_time_{0, 0, RCL_ROS_TIME};
  bool ik_failing_{false};
  bool pose_reference_valid_{false};
  tf2::Vector3 reference_vr_position_;
  tf2::Quaternion reference_vr_quat_;
  tf2::Vector3 reference_tcp_position_;
  tf2::Quaternion reference_tcp_quat_;
  double reference_tcp_elbow_{0.0};
  rclcpp::Time pose_reference_lock_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_servo_command_time_{0, 0, RCL_ROS_TIME};
  bool received_servo_command_{false};
  bool filtered_cartesian_target_valid_{false};
  tf2::Vector3 filtered_target_position_;
  tf2::Quaternion filtered_target_quat_;
  rclcpp::Time last_cartesian_filter_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<rokae::xMateErProRobot> robot_;
  std::shared_ptr<rokae::RtMotionControlCobot<kDof>> rt_controller_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_servo_target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_servo_sent_target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_raw_ik_target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_servo_error_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<vr_srv::GetRobotInfo>::SharedPtr get_info_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_operate_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr servo_enable_srv_;
  rclcpp::Service<vr_srv::JointMove>::SharedPtr joint_move_srv_;
  rclcpp::Service<vr_srv::LinearMove>::SharedPtr linear_move_srv_;
  rclcpp::Service<vr_srv::KineInverse7>::SharedPtr ik_srv_;
  rclcpp::Service<vr_srv::CalculateFK>::SharedPtr fk_srv_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_j_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_p_sub_;
  rclcpp::Subscription<vr_robot_interfaces::msg::ArmServoCommand>::SharedPtr arm_command_sub_;
  rclcpp::CallbackGroup::SharedPtr servo_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  std::mutex servo_mutex_;
};

}  // namespace xcore_gateway
