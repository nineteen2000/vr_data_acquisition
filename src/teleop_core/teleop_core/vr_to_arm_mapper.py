"""
订阅vr_controller_state,发布arm_servo_command,将VR控制器状态转换为臂伺服命令.
VR 手柄状态
    ↓
订阅 /vr_controller_state
    ↓
判断左右手柄是否按下 trigger
    ↓
记录按下瞬间的参考位姿
    ↓
计算当前位姿 - 参考位姿
    ↓
根据 output_mode 生成命令

    joint 模式：
        VR 位移/旋转变化 → 7 个关节角

    pose 模式：
        VR 位移变化 → 缩放后的 Pose

    ↓
发布 ArmServoCommand
    ↓
机械臂控制节点执行运动
"""

from copy import deepcopy

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from vr_robot_interfaces.msg import ArmServoCommand, VrControllerState

from teleop_core.kinematics import forward_kinematics, solve_ik
from teleop_core.robot_model import JOINT_LIMITS_MAX, JOINT_LIMITS_MIN
from teleop_core.transforms import finite_pose, vr_axis_maps, vr_delta_to_target_tcp


class VrToArmMapper(Node):

    def __init__(self):
        super().__init__('vr_to_arm_mapper')

        self.declare_parameter('state_topic', '/vr_controller_state')
        self.declare_parameter('command_topic', '/arm_servo_command')
        self.declare_parameter('left_command_topic', '/left_arm/arm_servo_command')
        self.declare_parameter('right_command_topic', '/right_arm/arm_servo_command')
        self.declare_parameter('publish_global_topic', True)
        self.declare_parameter('publish_split_topics', True)
        # 默认关节运动
        self.declare_parameter('output_mode', 'joint')
        # 扳机按下超过0.2才会激活运动
        self.declare_parameter('deadman_threshold', 0.2)
        # ArmServoCommand 固定发布频率，和 xcore_gateway 默认 ServoJ 周期 50Hz 对齐。
        self.declare_parameter('command_publish_rate_hz', 50.0)
        # VR 输入断流时停止继续发布旧的 enabled 命令。
        self.declare_parameter('vr_state_timeout_s', 0.2)
        # 位置缩放因子
        self.declare_parameter('position_scale', 1.0)
        # 旋转缩放因子
        self.declare_parameter('rotation_scale', 0.5)
        # VR 坐标系到机器人 base 坐标系的 row-major 4x4 方向映射矩阵。
        # 当前 PICO raw: +X 左, +Y 上, +Z 后 -> robot: +Y 左, +Z 上, -X 后。
        # 旧 PICO raw(+X 右, +Y 上, +Z 前): [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.declare_parameter(
            'vr_rot',
            [
                0.0, 0.0, -1.0, -1.0,
                1.0, 0.0, 0.0, 1.0,
                0.0, 1.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        self.declare_parameter('publish_disabled_commands', True)
        self.declare_parameter('left_arm_name', 'left')
        self.declare_parameter('right_arm_name', 'right')
        # 中位关节角度：肘部弯曲、远离限位与奇异，须等于仿真/真机初始构型。
        self.declare_parameter(
            'left_neutral_joints',
            [0.0, -0.4, 0.0, 1.5, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'right_neutral_joints',
            [0.0, -0.4, 0.0, 1.5, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'joint_limits_min',
            list(JOINT_LIMITS_MIN),
        )
        self.declare_parameter(
            'joint_limits_max',
            list(JOINT_LIMITS_MAX),
        )
        # VR 位移到关节角的比例
        self.declare_parameter(
            'position_to_joint_scale',
            [1.4, 1.2, 1.2],
        )
        self.declare_parameter(
            'rotation_to_joint_scale',
            [0.35, 0.35, 0.35, 0.25],
        )
        # 读取vr_controller_state参数
        self.state_topic = self.get_parameter('state_topic').value
        self.deadman_threshold = float(self.get_parameter('deadman_threshold').value)
        self.command_publish_rate_hz = max(
            1.0,
            float(self.get_parameter('command_publish_rate_hz').value),
        )
        self.vr_state_timeout_s = float(
            self.get_parameter('vr_state_timeout_s').value
        )
        self.position_scale = float(self.get_parameter('position_scale').value)
        self.rotation_scale = float(self.get_parameter('rotation_scale').value)
        self.vr_rot = list(self.get_parameter('vr_rot').value)
        self.vr_axis_map = vr_axis_maps(self.vr_rot)
        # 转小写
        self.output_mode = str(self.get_parameter('output_mode').value).lower()
        self.publish_disabled_commands = bool(
            self.get_parameter('publish_disabled_commands').value
        )

        self.left_arm_name = str(self.get_parameter('left_arm_name').value)
        self.right_arm_name = str(self.get_parameter('right_arm_name').value)
        self.left_neutral_joints = list(self.get_parameter('left_neutral_joints').value)
        self.right_neutral_joints = list(self.get_parameter('right_neutral_joints').value)
        self.joint_limits_min = list(self.get_parameter('joint_limits_min').value)
        self.joint_limits_max = list(self.get_parameter('joint_limits_max').value)
        self.position_to_joint_scale = list(
            self.get_parameter('position_to_joint_scale').value
        )
        self.rotation_to_joint_scale = list(
            self.get_parameter('rotation_to_joint_scale').value
        )
        # 参考位姿，按下扳机记录起点，计算相对位姿
        self.left_reference_pose = None
        self.right_reference_pose = None
        # 上一帧输出关节作为 IK seed，左右臂独立维护以保证连续性。
        self.left_seed_joints = np.asarray(self.left_neutral_joints, dtype=float)
        self.right_seed_joints = np.asarray(self.right_neutral_joints, dtype=float)
        # mapper 不订阅 /joint_states，锁定瞬间只能用 FK(seed) 作为参考 TCP。
        # 这依赖 seed 与仿真真实构型一致；首次锁定则依赖仿真初始构型等于 neutral。
        self.left_ref_tcp_pos = None
        self.left_ref_tcp_quat = None
        self.right_ref_tcp_pos = None
        self.right_ref_tcp_quat = None
        self.left_ref_vr_pos = None
        self.left_ref_vr_quat = None
        self.right_ref_vr_pos = None
        self.right_ref_vr_quat = None
        self.left_was_enabled = False
        self.right_was_enabled = False
        self.latest_vr_state = None
        self.latest_vr_state_time = None
        # 全局发布
        self.global_pub = None
        if bool(self.get_parameter('publish_global_topic').value):
            self.global_pub = self.create_publisher(
                ArmServoCommand,
                str(self.get_parameter('command_topic').value),
                10,
            )
        # 左右臂单独发布
        self.left_pub = None
        self.right_pub = None
        if bool(self.get_parameter('publish_split_topics').value):
            self.left_pub = self.create_publisher(
                ArmServoCommand,
                str(self.get_parameter('left_command_topic').value),
                10,
            )
            self.right_pub = self.create_publisher(
                ArmServoCommand,
                str(self.get_parameter('right_command_topic').value),
                10,
            )

        self.state_sub = self.create_subscription(
            VrControllerState,
            self.state_topic,
            self.on_vr_state,
            10,
        )
        self.command_timer = self.create_timer(
            1.0 / self.command_publish_rate_hz,
            self.on_command_timer,
        )

        # if self.output_mode not in ('joint', 'pose'):
        #     self.get_logger().warn(
        #         "Unsupported output_mode '%s', falling back to 'joint'."
        #         % self.output_mode
        #     )
        #     self.output_mode = 'joint'

        # self.get_logger().info(
        #     "vr_to_arm_mapper publishing %s ArmServoCommand from %s"
        #     % (self.output_mode, self.state_topic)
        # )

    def on_vr_state(self, msg):
        self.latest_vr_state = deepcopy(msg)
        self.latest_vr_state_time = self.get_clock().now()

    def on_command_timer(self):
        if self.latest_vr_state is None:
            return
        if (
            self.vr_state_timeout_s > 0.0
            and self.latest_vr_state_time is not None
            and (
                self.get_clock().now() - self.latest_vr_state_time
            ).nanoseconds * 1e-9
            > self.vr_state_timeout_s
        ):
            self.publish_stale_disabled_commands()
            return
        self.publish_from_vr_state(self.latest_vr_state)

    def publish_stale_disabled_commands(self):
        msg = self.latest_vr_state
        if msg is None:
            return
        left_was_enabled = self.left_was_enabled
        right_was_enabled = self.right_was_enabled
        self.left_reference_pose = None
        self.left_ref_tcp_pos = None
        self.left_ref_tcp_quat = None
        self.left_ref_vr_pos = None
        self.left_ref_vr_quat = None
        self.right_reference_pose = None
        self.right_ref_tcp_pos = None
        self.right_ref_tcp_quat = None
        self.right_ref_vr_pos = None
        self.right_ref_vr_quat = None
        self.left_seed_joints = self.publish_arm_command(
            msg,
            arm_name=self.left_arm_name,
            pose=msg.left_pose,
            reference_pose=None,
            ref_vr_pos=None,
            ref_vr_quat=None,
            ref_tcp_pos=None,
            ref_tcp_quat=None,
            neutral_joints=self.left_neutral_joints,
            seed_joints=self.left_seed_joints,
            enabled=False,
            split_pub=self.left_pub,
            publish_disabled=left_was_enabled,
        )
        self.right_seed_joints = self.publish_arm_command(
            msg,
            arm_name=self.right_arm_name,
            pose=msg.right_pose,
            reference_pose=None,
            ref_vr_pos=None,
            ref_vr_quat=None,
            ref_tcp_pos=None,
            ref_tcp_quat=None,
            neutral_joints=self.right_neutral_joints,
            seed_joints=self.right_seed_joints,
            enabled=False,
            split_pub=self.right_pub,
            publish_disabled=right_was_enabled,
        )
        self.left_was_enabled = False
        self.right_was_enabled = False

    def publish_from_vr_state(self, msg):
        # 位姿有效 + 数据正常 + 按住扳机
        left_was_enabled = self.left_was_enabled
        right_was_enabled = self.right_was_enabled
        left_enabled = (
            msg.left_pose_valid
            and finite_pose(msg.left_pose)
            and msg.left_trigger >= self.deadman_threshold
        )
        right_enabled = (
            msg.right_pose_valid
            and finite_pose(msg.right_pose)
            and msg.right_trigger >= self.deadman_threshold
        )
        # 保证扳机按下时机械臂从当前位姿运动
        if left_enabled and not self.left_was_enabled:
            self.left_reference_pose = deepcopy(msg.left_pose)
            self.left_ref_vr_pos = self._pose_position_array(msg.left_pose)
            self.left_ref_vr_quat = self._pose_quat_array(msg.left_pose)
            if self.left_seed_joints.shape == (7,):
                self.left_ref_tcp_pos, self.left_ref_tcp_quat = forward_kinematics(
                    self.left_seed_joints
                )
        if right_enabled and not self.right_was_enabled:
            self.right_reference_pose = deepcopy(msg.right_pose)
            self.right_ref_vr_pos = self._pose_position_array(msg.right_pose)
            self.right_ref_vr_quat = self._pose_quat_array(msg.right_pose)
            if self.right_seed_joints.shape == (7,):
                self.right_ref_tcp_pos, self.right_ref_tcp_quat = forward_kinematics(
                    self.right_seed_joints
                )
        # 松开扳机时,重置参考位姿
        if not left_enabled:
            self.left_reference_pose = None
            self.left_ref_tcp_pos = None
            self.left_ref_tcp_quat = None
            self.left_ref_vr_pos = None
            self.left_ref_vr_quat = None
        if not right_enabled:
            self.right_reference_pose = None
            self.right_ref_tcp_pos = None
            self.right_ref_tcp_quat = None
            self.right_ref_vr_pos = None
            self.right_ref_vr_quat = None
        self.left_seed_joints = self.publish_arm_command(
            msg,
            arm_name=self.left_arm_name,
            pose=msg.left_pose,
            reference_pose=self.left_reference_pose,
            ref_vr_pos=self.left_ref_vr_pos,
            ref_vr_quat=self.left_ref_vr_quat,
            ref_tcp_pos=self.left_ref_tcp_pos,
            ref_tcp_quat=self.left_ref_tcp_quat,
            neutral_joints=self.left_neutral_joints,
            seed_joints=self.left_seed_joints,
            enabled=left_enabled,
            split_pub=self.left_pub,
            publish_disabled=left_was_enabled and not left_enabled,
        )
        self.right_seed_joints = self.publish_arm_command(
            msg,
            arm_name=self.right_arm_name,
            pose=msg.right_pose,
            reference_pose=self.right_reference_pose,
            ref_vr_pos=self.right_ref_vr_pos,
            ref_vr_quat=self.right_ref_vr_quat,
            ref_tcp_pos=self.right_ref_tcp_pos,
            ref_tcp_quat=self.right_ref_tcp_quat,
            neutral_joints=self.right_neutral_joints,
            seed_joints=self.right_seed_joints,
            enabled=right_enabled,
            split_pub=self.right_pub,
            publish_disabled=right_was_enabled and not right_enabled,
        )
        # 更新上一帧状态。disabled 命令只在 enabled -> disabled 边沿发布，
        # 避免 PICO 启动但未按扳机时立即关闭已使能的 ServoJ。
        self.left_was_enabled = left_enabled
        self.right_was_enabled = right_enabled

    @staticmethod
    def _pose_position_array(pose):
        # geometry_msgs/Pose 的位置转为 numpy 三维向量。
        return np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=float,
        )

    @staticmethod
    def _pose_quat_array(pose):
        # geometry_msgs/Pose 的四元数转为 numpy xyzw 顺序。
        return np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )

    # 发布arm_servo_command
    def publish_arm_command(
        self,
        state_msg,
        arm_name,
        pose,
        reference_pose,
        ref_vr_pos,
        ref_vr_quat,
        ref_tcp_pos,
        ref_tcp_quat,
        neutral_joints,
        seed_joints,
        enabled,
        split_pub,
        publish_disabled,
    ):
        if not enabled and (
            not self.publish_disabled_commands or not publish_disabled
        ):
            return seed_joints

        command = ArmServoCommand()
        command.header = state_msg.header
        command.arm_name = arm_name
        command.enabled = enabled

        if self.output_mode == 'pose':
            command.mode = ArmServoCommand.MODE_POSE
            # pose 模式直接透传 VR 原始位姿，由 gateway 统一锁参考并缩放。
            command.target_pose = deepcopy(pose)
        else:
            command.mode = ArmServoCommand.MODE_JOINT
            command.target_pose = Pose()
            # neutral 配置错误时跳过本帧，避免后续按 7 轴访问导致异常。
            if len(neutral_joints) != 7:
                self.get_logger().warn(
                    'neutral_joints must contain 7 values; skip joint command.'
                )
                return seed_joints
            # 未锁定参考时保持上一帧输出关节，松开扳机不会重置 seed。
            if reference_pose is None or ref_tcp_pos is None or ref_tcp_quat is None:
                command.joint_positions = [float(value) for value in seed_joints]
            else:
                current_vr_pos = self._pose_position_array(pose)
                current_vr_quat = self._pose_quat_array(pose)
                # VR 相对位姿先映射为目标 TCP，再由自研 IK 求 7 轴关节。
                target_pos, target_quat = vr_delta_to_target_tcp(
                    current_vr_pos,
                    current_vr_quat,
                    ref_vr_pos,
                    ref_vr_quat,
                    ref_tcp_pos,
                    ref_tcp_quat,
                    self.vr_axis_map,
                    self.position_scale,
                    self.rotation_scale,
                )
                solution, converged, _ = solve_ik(
                    target_pos,
                    target_quat,
                    seed_joints,
                    joint_limits_min=self.joint_limits_min,
                    joint_limits_max=self.joint_limits_max,
                )
                if converged:
                    seed_joints = solution
                    command.joint_positions = [
                        float(value) for value in seed_joints
                    ]
                else:
                    # IK 未收敛时保持上一帧关节，避免发布不可信的中间解。
                    self.get_logger().warn(
                        'IK not converged for %s arm, holding last joints.'
                        % arm_name,
                        throttle_duration_sec=1.0,
                    )
                    command.joint_positions = [
                        float(value) for value in seed_joints
                    ]
        # 单独发布与全局发布
        if split_pub is not None:
            split_pub.publish(command)
        if self.global_pub is not None:
            self.global_pub.publish(command)
        return seed_joints


def main(args=None):
    rclpy.init(args=args)
    node = VrToArmMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
