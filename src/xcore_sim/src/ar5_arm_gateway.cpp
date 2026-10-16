/*
 * xcore_sim 的 ros2_control/MoveIt 仿真适配器，不连接 xCoreSDK。
 *
 * 输入：ArmServoCommand（MODE_JOINT 或 MODE_POSE）与 ros2_control /joint_states。
 * 输出：JointTrajectory、按臂命名的关节/TCP/状态 topic。
 * 依赖：trajectory controller、TF(base->tcp) 和 MoveIt /compute_ik 由外部 launch 提供。
 *
 * MODE_POSE 的计算边界：第一次有效命令锁定 VR 与 TCP 参考位姿，之后只处理相对
 * 位移/旋转；平移经 vr_rot 映射到机器人基坐标，旋转经 SLERP 缩放后叠加到参考
 * TCP。优先异步调用 MoveIt IK；服务不可用、请求未完成或求解失败时只保持上一
 * 有效 IK 解，不再做经验线性映射。
 *
 * 可迁移到实机的部分：ROS 消息过滤、相对位姿、坐标变换、参考位姿生命周期、
 * 有限值检查和每周期步长思想。不可直接迁移的部分：fake controller 时序、MoveIt
 * IK 成功率、动力学/碰撞表现和 motion_enabled_on_start=true 的默认策略。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "vr_robot_interfaces/msg/arm_servo_command.hpp"
#include "xcore_sim/vr_pose_mapping.hpp"

using namespace std::chrono_literals;

namespace xcore_sim
{

class Ar5ArmGateway : public rclcpp::Node
{
public:
  Ar5ArmGateway(): Node("ar5_arm_gateway"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
  {
    arm_name_ = declare_parameter<std::string>("arm_name", "left");
    robot_type_ = declare_parameter<std::string>("robot_type", "AR5L");
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm_servo_command");
    // 输出轨迹
    controller_topic_ = declare_parameter<std::string>(
      "controller_topic", "/position_joint_trajectory_controller/joint_trajectory");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    // TF/轨迹限制：base->tcp 用于锁定真实的当前 TCP 参考，单位为米和弧度。
    base_frame_ = declare_parameter<std::string>("base_frame", "AR5-5_07L-W4C4A2_base");   
    // TCP 坐标系
    tcp_frame_ = declare_parameter<std::string>("tcp_frame", "AR5-5_07L-W4C4A2_tcp");
    // 单周期最大关节变化
    max_joint_step_rad_ = declare_parameter<double>("max_joint_step_rad", 0.005);
    // 轨迹持续时间
    trajectory_duration_s_ = declare_parameter<double>("trajectory_duration_s", 0.2);
    // VR 到 TCP 的几何映射。vr_rot 是 row-major 3x3 平移变换矩阵；
    // 缩放
    rotation_scale_ = declare_parameter<double>("rotation_scale", 0.5);
    position_scale_ = declare_parameter<double>("position_scale", 1.0);
    // vr到机器人基座标转换
    vr_rot_ = declare_parameter<std::vector<double>>(
      "vr_rot", 
      {0.0, 0.0, 1.0, 
      -1.0, 0.0, 0.0, 
      0.0, 1.0, 0.0});
    // IK 忙或失败时是否启用“保持上一有效关节解”兜底，默认 true。
    ik_group_name_ = declare_parameter<std::string>("ik_group_name", "AR5L_arm");
    ik_timeout_s_ = declare_parameter<double>("ik_timeout_s", 0.5);
    ik_fallback_enabled_ = declare_parameter<bool>("ik_fallback_enabled", true);
    // 兼容旧配置项；经验线性 fallback 已删除，该参数不再参与计算。
    declare_parameter<std::vector<double>>(
      "pose_to_joint_scale", {1.4, 1.2, 1.2, 0.35, 0.35, 0.35, 0.25});
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());
    // 仅仿真可考虑启动即使能；真机 gateway 不使用此参数，也不应沿用该默认策略。
    motion_enabled_ = declare_parameter<bool>("motion_enabled_on_start", false);

    // 7轴关节数检查
    if (joint_names_.size() != kJointCount) {
      RCLCPP_WARN(
        get_logger(),
        "joint_names size is %zu, expected %zu. Fall back to joint1..joint7.",
        joint_names_.size(), kJointCount);
      joint_names_ = default_joint_names();
    }
    if (max_joint_step_rad_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "max_joint_step_rad must be positive, use 0.05 rad.");
      max_joint_step_rad_ = 0.05;
    }
    if (trajectory_duration_s_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "trajectory_duration_s must be positive, use 0.2 s.");
      trajectory_duration_s_ = 0.2;
    }
    if (!std::isfinite(rotation_scale_) || rotation_scale_ < 0.0) {
      RCLCPP_WARN(get_logger(), "rotation_scale must be non-negative, use 0.5.");
      rotation_scale_ = 0.5;
    }
    if (!std::isfinite(position_scale_) || position_scale_ < 0.0) {
      RCLCPP_WARN(get_logger(), "position_scale must be non-negative, use 1.0.");
      position_scale_ = 1.0;
    }
    if (vr_rot_.size() != 9 || !is_finite_vector(vr_rot_)) {
      RCLCPP_WARN(get_logger(), "vr_rot must contain 9 finite row-major values, use default.");
      vr_rot_ = {0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    }
    if (!std::isfinite(ik_timeout_s_) || ik_timeout_s_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "ik_timeout_s must be positive, use 0.1 s.");
      ik_timeout_s_ = 0.1;
    }

    // 初始化当前关节位置为 NaN
    current_positions_.assign(kJointCount, std::numeric_limits<double>::quiet_NaN());
    last_ik_solution_.assign(kJointCount, std::numeric_limits<double>::quiet_NaN());

    // ROS 图：controller_topic 是唯一执行出口；其余三个 publisher 用于观测和录包。
    trajectory_pub_ =create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_topic_, 10);
    // xcore/left/joint_states
    joint_state_pub_ =create_publisher<sensor_msgs::msg::JointState>("/xcore/" + arm_name_ + "/joint_states", 10);
    // xcore/left/tcp_pose
    tcp_pose_pub_ =create_publisher<geometry_msgs::msg::PoseStamped>("/xcore/" + arm_name_ + "/tcp_pose", 10);
    status_pub_ =create_publisher<std_msgs::msg::String>("/xcore/" + arm_name_ + "/status", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&Ar5ArmGateway::on_joint_state, this, std::placeholders::_1));
    // 订阅arm servo command
      command_sub_ = create_subscription<vr_robot_interfaces::msg::ArmServoCommand>(
      command_topic_, 10,
      std::bind(&Ar5ArmGateway::on_arm_command, this, std::placeholders::_1));
    // /compute_ik 是 MoveIt move_group 提供的全局服务，不由本节点创建。
    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    set_motion_enabled_srv_ = create_service<std_srvs::srv::SetBool>(
      "/xcore/" + arm_name_ + "/set_motion_enabled",
      std::bind(
        &Ar5ArmGateway::on_set_motion_enabled, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/xcore/" + arm_name_ + "/stop",
      std::bind(&Ar5ArmGateway::on_stop, this, std::placeholders::_1, std::placeholders::_2));

    // 10 Hz 定时器只发布诊断/TCP 观测；轨迹发布由命令回调直接触发。
    timer_ = create_wall_timer(100ms, std::bind(&Ar5ArmGateway::on_timer, this));
    set_status(motion_enabled_ ? "gateway started, motion enabled" : "gateway started, motion disabled");
  }

private:
  static constexpr std::size_t kJointCount = 7;

  bool accepts_arm_name(const std::string & name) const
  {
    // 空名称兼容单臂全局命令；双臂上游应始终携带明确的 left/right。
    return name.empty() || name == arm_name_ || name == robot_type_;
  }

  // 关节顺序重排,缺任一关节时整帧拒绝，防止索引错位。
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    
    sensor_msgs::msg::JointState ordered;
    ordered.header = msg->header;
    ordered.name = joint_names_;
    ordered.position.resize(kJointCount);
    ordered.velocity.resize(kJointCount);
    ordered.effort.resize(kJointCount);

    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        have_joint_state_ = false;
        set_status("waiting for AR5L joint_states");
        return;
      }

      const auto source_index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (source_index >= msg->position.size()) {
        have_joint_state_ = false;
        set_status("joint_states missing position data");
        return;
      }

      ordered.position[i] = msg->position[source_index];
      ordered.velocity[i] = source_index < msg->velocity.size() ? msg->velocity[source_index] : 0.0;
      ordered.effort[i] = source_index < msg->effort.size() ? msg->effort[source_index] : 0.0;
      current_positions_[i] = ordered.position[i];
    }

    have_joint_state_ = true;
    joint_state_pub_->publish(ordered);
  }

  // 统一命令入口：先做目标臂/使能/motion/mode 检查，再分发 JOINT 或 POSE。
  void on_arm_command(const vr_robot_interfaces::msg::ArmServoCommand::SharedPtr msg)
  {
    // 手臂是否匹配-名称\型号
    if (!accepts_arm_name(msg->arm_name)) {
      return;
    }

    if (!msg->enabled) {
      clear_pose_reference();
      set_status("ArmServoCommand 下使能; 清除参考位姿");
      return;
    }
    // 启动及使能检查
    if (!motion_enabled_) {
      set_status("dry-run ArmServoCommand ignored because motion is disabled");
      return;
    }

    if (msg->mode == vr_robot_interfaces::msg::ArmServoCommand::MODE_POSE) {
      handle_pose_command(*msg);
      return;
    }
    if (msg->mode != vr_robot_interfaces::msg::ArmServoCommand::MODE_JOINT) {
      set_status("不支持的运动模式");
      return;
    }
    // 是否为7轴
    if (msg->joint_positions.size() != kJointCount) {
      set_status("非7轴关节角度");
      RCLCPP_WARN(
        get_logger(), "Reject joint command: expected %zu joints, got %zu.",
        kJointCount, msg->joint_positions.size());
      return;
    }
    // 判断是否超上下限
    const std::vector<double> target(msg->joint_positions.begin(), msg->joint_positions.end());
    if (!is_finite_vector(target)) {
      set_status("rejected joint command: non-finite value");
      return;
    }
    // 当前位置状态有效性判断
    if (!have_joint_state_ || !is_finite_vector(current_positions_)) {
      set_status("rejected joint command: no valid current joint state");
      return;
    }
    // MODE_JOINT 是新架构主路径：直接执行 mapper 自研 IK 输出的关节角。
    // 将每个周期的关节变化限制在安全步长内；大目标会由后续指令逐步逼近。
    std::vector<double> limited_target = target;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      limited_target[i] = std::clamp(
        target[i],
        current_positions_[i] - max_joint_step_rad_,
        current_positions_[i] + max_joint_step_rad_);
    }

    publish_trajectory(limited_target);
    set_status("published AR5L joint trajectory");
  }
  // 锁定当前 VR 位姿和当前机械臂 TCP 位姿，后续只计算相对变化
  bool lock_pose_reference(const geometry_msgs::msg::Pose & vr_pose)
  {
    // if (!have_joint_state_ || !is_finite_vector(current_positions_)) {
    //   set_status("cannot lock pose reference: no valid current joint state");
    //   return false;
    // }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, tcp_frame_, tf2::TimePointZero);
      // 保存vr参考位姿
      reference_vr_position_ = tf2::Vector3(
        vr_pose.position.x, vr_pose.position.y, vr_pose.position.z);
      reference_vr_quat_ = tf2::Quaternion(
        vr_pose.orientation.x, vr_pose.orientation.y,
        vr_pose.orientation.z, vr_pose.orientation.w);
      // 保存tcp参考位姿
      reference_tcp_position_ = tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      reference_tcp_quat_ = tf2::Quaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      if (reference_vr_quat_.length2() <= 0.0 || reference_tcp_quat_.length2() <= 0.0) {
        set_status("cannot lock pose reference: invalid quaternion");
        return false;
      }
      // 归一化
      reference_vr_quat_.normalize();
      reference_tcp_quat_.normalize();
      reference_joint_positions_ = current_positions_;
      pose_reference_valid_ = true;
      ++pose_reference_generation_;
      set_status("locked VR and TCP pose reference");
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot lock TCP reference: %s", ex.what());
      return false;
    }
  }
  // 位姿控制模式处理函数
  void handle_pose_command(const vr_robot_interfaces::msg::ArmServoCommand & msg)
  {
    // 1) 校验并确保参考有效；2) 计算相对位姿；3) 请求 IK 或保持上一有效解。
    const auto & pose = msg.target_pose;
    const std::vector<double> pose_values = {
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    if (!is_finite_vector(pose_values)) {
      set_status("rejected pose command: non-finite value");
      return;
    }
    if (!pose_reference_valid_ && !lock_pose_reference(pose)) {
      return;
    }
    if (!have_joint_state_ || !is_finite_vector(current_positions_)) {
      set_status("rejected pose command: no valid current joint state");
      return;
    }

    // 旋转增量 q_current * inverse(q_ref)，再按 rotation_scale 缩放并叠加到参考 TCP。
    tf2::Quaternion current_quat(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    if (current_quat.length2() <= 0.0) {
      set_status("rejected pose command: zero quaternion");
      return;
    }
    current_quat.normalize();
    tf2::Quaternion q_delta = current_quat * reference_vr_quat_.inverse();
    q_delta.normalize();
    const tf2::Quaternion scaled_delta =
      tf2::Quaternion::getIdentity().slerp(q_delta, rotation_scale_);
    tf2::Quaternion target_quat = scaled_delta * reference_tcp_quat_;
    target_quat.normalize();

    // 平移先按 position_scale 缩放，再由 vr_rot 从 VR 轴映射到机器人 base 轴。
    const tf2::Vector3 position_delta(
      (pose.position.x - reference_vr_position_.x()) * position_scale_,
      (pose.position.y - reference_vr_position_.y()) * position_scale_,
      (pose.position.z - reference_vr_position_.z()) * position_scale_);
    // vr映射到机器人坐标系
    const tf2::Matrix3x3 vr_rotation = vr_rotation_matrix(vr_rot_);
    const tf2::Vector3 target_position = reference_tcp_position_ + vr_rotation * position_delta;

    // 同一时刻只允许一个异步 IK，防止约 90 Hz VR 输入堆积请求。
    if (ik_request_pending_ || !ik_client_->service_is_ready()) {
      if (ik_fallback_enabled_) {
        if (publish_last_ik_solution_hold("IK busy; holding last solution")) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "IK busy or service not ready, holding last solution.");
        } else {
          set_status("IK busy; no last solution to hold");
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "IK busy or service not ready, but no valid last solution is available.");
        }
      }
      return;
    }

    // 上次 IK 解作为 seed 可减少冗余机械臂在相邻分支之间跳变。
    const std::vector<double> & ik_ref_positions =
      is_finite_vector(last_ik_solution_) ? last_ik_solution_ : current_positions_;

    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = ik_group_name_;
    request->ik_request.pose_stamped.header.stamp = now();
    request->ik_request.pose_stamped.header.frame_id = base_frame_;
    request->ik_request.pose_stamped.pose.position.x = target_position.x();
    request->ik_request.pose_stamped.pose.position.y = target_position.y();
    request->ik_request.pose_stamped.pose.position.z = target_position.z();
    request->ik_request.pose_stamped.pose.orientation.x = target_quat.x();
    request->ik_request.pose_stamped.pose.orientation.y = target_quat.y();
    request->ik_request.pose_stamped.pose.orientation.z = target_quat.z();
    request->ik_request.pose_stamped.pose.orientation.w = target_quat.w();
    request->ik_request.robot_state.joint_state.header.stamp = now();
    request->ik_request.robot_state.joint_state.name = joint_names_;
    request->ik_request.robot_state.joint_state.position = ik_ref_positions;
    request->ik_request.timeout = rclcpp::Duration::from_seconds(ik_timeout_s_);

    ik_request_pending_ = true;
    const auto request_generation = pose_reference_generation_;

    ik_client_->async_send_request(
      request,
      [this, request_generation](
        rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future) {
        ik_request_pending_ = false;
        if (!motion_enabled_ || !pose_reference_valid_ ||
          request_generation != pose_reference_generation_)
        {
          return;
        }
        const auto response = future.get();
        if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          if (ik_fallback_enabled_) {
            if (publish_last_ik_solution_hold("IK failed; holding last solution")) {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "IK error code %d, holding last solution.",
                response->error_code.val);
            } else {
              set_status("IK failed; no last solution to hold");
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "IK error code %d, but no valid last solution is available.",
                response->error_code.val);
            }
          } else {
            set_status("IK failed; holding the previous target");
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "IK failed with error code %d.",
              response->error_code.val);
          }
          return;
        }

        std::vector<double> target(kJointCount);
        const auto & solution = response->solution.joint_state;
        for (std::size_t i = 0; i < kJointCount; ++i) {
          const auto it = std::find(solution.name.begin(), solution.name.end(), joint_names_[i]);
          if (it == solution.name.end()) {
            set_status("IK response missing an AR5L joint");
            return;
          }
          const auto index = static_cast<std::size_t>(std::distance(solution.name.begin(), it));
          if (index >= solution.position.size()) {
            set_status("IK response missing joint position data");
            return;
          }
          target[i] = solution.position[index];
        }
        if (!is_finite_vector(target)) {
          set_status("IK solution contains non-finite values");
          return;
        }
        // 缓存成功解
        last_ik_solution_ = target;
        // 步长限幅后发布
        for (std::size_t i = 0; i < kJointCount; ++i) {
          target[i] = std::clamp(
            target[i], current_positions_[i] - max_joint_step_rad_,
            current_positions_[i] + max_joint_step_rad_);
        }
        publish_trajectory(target);
        set_status("published MODE_POSE IK trajectory");
      });
  }

  bool publish_last_ik_solution_hold(const std::string & status)
  {
    if (!is_finite_vector(last_ik_solution_) || !is_finite_vector(current_positions_)) {
      return false;
    }

    std::vector<double> target = last_ik_solution_;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      target[i] = std::clamp(
        target[i], current_positions_[i] - max_joint_step_rad_,
        current_positions_[i] + max_joint_step_rad_);
    }

    publish_trajectory(target);
    set_status(status);
    return true;
  }

  void clear_pose_reference()
  {
    // generation 使已发出的异步 IK 回调失效，避免松开扳机后旧结果继续下发。
    pose_reference_valid_ = false;
    ++pose_reference_generation_;
    reference_vr_position_ = tf2::Vector3();
    reference_vr_quat_ = tf2::Quaternion();
    reference_tcp_position_ = tf2::Vector3();
    reference_tcp_quat_ = tf2::Quaternion();
    reference_joint_positions_.clear();
    last_ik_solution_.assign(kJointCount, std::numeric_limits<double>::quiet_NaN());
  }

  void on_set_motion_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // motion gate 只控制是否接受新命令，不等价于真实控制器的上电/伺服使能。
    motion_enabled_ = request->data;
    if (!motion_enabled_) {
      clear_pose_reference();
    }
    response->success = true;
    response->message = motion_enabled_ ? "AR5L motion enabled" : "AR5L motion disabled";
    set_status(response->message);
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // fake hardware 没有硬件急停；这里只关闭软件 gate 并清理参考状态。
    // 用当前位置发一个 hold trajectory；真实急停仍应由底层控制器或硬件执行。
    if (!have_joint_state_ || !is_finite_vector(current_positions_)) {
      response->success = false;
      response->message = "no valid joint state for hold trajectory";
      set_status(response->message);
      return;
    }

    publish_trajectory(current_positions_);
    response->success = true;
    response->message = "published hold trajectory";
    set_status(response->message);
  }

  void on_timer()
  {
    publish_tcp_pose();
    const auto current_time = now();
    if ((current_time - last_status_time_).seconds() >= 1.0) {
      publish_status();
      last_status_time_ = current_time;
    }
  }

  void publish_trajectory(const std::vector<double> & positions)
  {
    // 单点轨迹。控制器在 trajectory_duration_s 内插到目标；该时序与 xCore ServoJ
    // 的固定周期实时指令不同，因此只能验证上游目标和限幅逻辑。
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities.assign(kJointCount, 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_duration_s_);
    traj.points.push_back(point);

    trajectory_pub_->publish(traj);
  }

  void publish_tcp_pose()
  {
    // TCP 位姿直接来自 TF，反映 robot_state_publisher/ros2_control 模型状态。
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, tcp_frame_, tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = tf.header.stamp;
      pose.header.frame_id = base_frame_;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      tcp_pose_pub_->publish(pose);
      have_tcp_pose_ = true;
    } catch (const tf2::TransformException & ex) {
      have_tcp_pose_ = false;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TCP transform %s -> %s: %s",
        base_frame_.c_str(), tcp_frame_.c_str(), ex.what());
    }
  }

  void set_status(const std::string & status)
  {
    last_status_ = status;
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "robot_type=" << robot_type_
        << ", arm_name=" << arm_name_
        << ", motion_enabled=" << (motion_enabled_ ? "true" : "false")
        << ", joint_state=" << (have_joint_state_ ? "ok" : "missing")
        << ", tcp_pose=" << (have_tcp_pose_ ? "ok" : "missing")
        << ", last_status=" << last_status_;
    msg.data = oss.str();
    status_pub_->publish(msg);
  }

  // 参数与运行状态分组：ROS 路由、几何映射、反馈缓存、异步 IK 生命周期。
  std::string arm_name_;
  std::string robot_type_;
  std::string command_topic_;
  std::string controller_topic_;
  std::string joint_state_topic_;
  std::string base_frame_;
  std::string tcp_frame_;
  double max_joint_step_rad_;
  double trajectory_duration_s_;
  double rotation_scale_;
  double position_scale_;
  std::vector<double> vr_rot_;
  std::string ik_group_name_;
  double ik_timeout_s_;
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  bool motion_enabled_{false};
  bool have_joint_state_{false};
  bool have_tcp_pose_{false};
  bool pose_reference_valid_{false};
  bool ik_request_pending_{false};
  std::uint64_t pose_reference_generation_{0};
  tf2::Vector3 reference_vr_position_;
  tf2::Quaternion reference_vr_quat_;
  tf2::Vector3 reference_tcp_position_;
  tf2::Quaternion reference_tcp_quat_;
  std::vector<double> reference_joint_positions_;
  std::vector<double> last_ik_solution_;
  bool ik_fallback_enabled_{true};
  std::string last_status_;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<vr_robot_interfaces::msg::ArmServoCommand>::SharedPtr command_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_motion_enabled_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace xcore_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xcore_sim::Ar5ArmGateway>());
  rclcpp::shutdown();
  return 0;
}
