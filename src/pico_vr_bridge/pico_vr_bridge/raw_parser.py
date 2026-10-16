"""解析/vr_raw_data中的PICO App协议,发布/vr_controller_state."""

import math
import re

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String
from vr_robot_interfaces.msg import VrControllerState


POSE_PATTERN = re.compile(
    # 示例: LPos: (x, y, z) LRot: (roll, pitch, yaw)
    r'(?P<label>[LR])Pos:\s*\((?P<pos>[^)]*)\)\s*'
    r'(?P=label)Rot:\s*\((?P<rot>[^)]*)\)'
)


class RawVrParser(Node):
    """把PICO App的11字段文本帧转换为统一VrControllerState."""

    def __init__(self):
        super().__init__('raw_vr_parser')
        self.declare_parameter('raw_topic', '/vr_raw_data')
        self.declare_parameter('state_topic', '/vr_controller_state')

        raw_topic = self.get_parameter('raw_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(VrControllerState, state_topic, 10)
        self.subscription = self.create_subscription(String, raw_topic, self.parse_raw_data, 10)
        self.bad_frame_count = 0

    def parse_raw_data(self, raw_msg):
        """解析单帧原始字符串;解析失败只告警,不终止节点."""

        parts = [part.strip() for part in raw_msg.data.strip().split(';')]
        if len(parts) != 11:
            self.bad_frame_count += 1
            self.get_logger().warn(
                f'Invalid PICO VR frame #{self.bad_frame_count}: fields={len(parts)}'
            )
            return

        try:
            state = VrControllerState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'pico_vr'
            state.raw_message = raw_msg.data

            # 字段顺序来自已采集的App日志:
            # timestamp;LG;RG;LT;RT;LPos+LRot;RPos+RRot;X;A;Y;B
            state.left_side_button = self.parse_tf(parts[1], 'LG')
            state.right_side_button = self.parse_tf(parts[2], 'RG')
            left_trigger = self.parse_tf(parts[3], 'LT')
            right_trigger = self.parse_tf(parts[4], 'RT')
            state.left_trigger = 1.0 if left_trigger else 0.0
            state.right_trigger = 1.0 if right_trigger else 0.0

            # 当前消息中front_button为bool,先用LT/RT承载扳机按下状态.
            state.left_front_button = left_trigger
            state.right_front_button = right_trigger

            state.left_pose, state.left_pose_valid = self.parse_pose(parts[5], 'L')
            state.right_pose, state.right_pose_valid = self.parse_pose(parts[6], 'R')

            state.button_x = self.parse_tf(parts[7], 'X')
            state.button_a = self.parse_tf(parts[8], 'A')
            state.button_y = self.parse_tf(parts[9], 'Y')
            state.button_b = self.parse_tf(parts[10], 'B')
        except ValueError as exc:
            self.bad_frame_count += 1
            self.get_logger().warn(f'Invalid PICO VR frame #{self.bad_frame_count}: {exc}')
            return

        self.publisher_.publish(state)

    @staticmethod
    def parse_tf(text, name):
        """解析形如LG=T或A=F的布尔字段."""

        key, value = text.split('=', 1)
        if key.strip() != name:
            raise ValueError(f'expected {name}, got {key.strip()}')
        value = value.strip()
        if value == 'T':
            return True
        if value == 'F':
            return False
        raise ValueError(f'expected {name}=T/F, got {text}')

    def parse_pose(self, text, label):
        """解析左右手柄位姿,并用全零位姿标记tracking无效."""

        match = POSE_PATTERN.fullmatch(text)
        if match is None or match.group('label') != label:
            raise ValueError(f'expected {label} pose, got {text}')

        position = self.parse_vector3(match.group('pos'))
        rotation_deg = self.parse_vector3(match.group('rot'))
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        quat = self.euler_degrees_to_quaternion(rotation_deg)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # App退出或手柄无效时可能发全零位姿;保留数值,但valid置false.
        valid = any(abs(value) > 1e-6 for value in position + rotation_deg)
        return pose, valid

    @staticmethod
    def parse_vector3(text):
        """解析逗号分隔的三维浮点数."""

        values = [float(value.strip()) for value in text.split(',')]
        if len(values) != 3:
            raise ValueError(f'expected 3 values, got {len(values)}')
        return values

    @staticmethod
    def euler_degrees_to_quaternion(rotation_deg):
        """把App输出的度数制roll/pitch/yaw转换为ROS Pose需要的四元数."""

        roll, pitch, yaw = [math.radians(value) for value in rotation_deg]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]


def main(args=None):
    rclpy.init(args=args)
    parser = RawVrParser()

    try:
        rclpy.spin(parser)
    except KeyboardInterrupt:
        pass
    finally:
        parser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
