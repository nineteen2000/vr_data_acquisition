import time

import rclpy
from lhandpro_interfaces.srv import GetNowPosition, GetNowPositionVelocity
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vr_robot_interfaces.msg import HandCommand, HandState


class HandStatePublisherSingle(Node):


    def __init__(self):
        super().__init__("hand_state_publisher_single")

        self.state_topic = self.declare_parameter("state_topic", "/hand_state").value
        self.command_topic = self.declare_parameter("command_topic", "/hand_command").value
        
        self.service_namespace = self.declare_parameter(
            "service_namespace", "/lhandpro_service"
        ).value.rstrip("/")
        self.supported_hand_ids = tuple(
            self.declare_parameter("supported_hand_ids", [0]).value
        )
        self.left_hand_joint_ids = list(
            self.declare_parameter("left_hand_joint_ids", [1, 2, 3, 4, 5, 6]).value
        )
        # 发布频率
        self.publish_rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        self.publish_period = 1.0 / self.publish_rate_hz

        self.callback_group = ReentrantCallbackGroup()
        # 最后一次发布的preset_name指令：open\middle\close
        self.last_preset = "unknown"

        self.get_now_position_cli = self.create_client(
            GetNowPosition,
            self.service_name("get_now_position"),
            callback_group=self.callback_group,
        )
        self.get_now_position_velocity_cli = self.create_client(
            GetNowPositionVelocity,
            self.service_name("get_now_position_velocity"),
            callback_group=self.callback_group,
        )

        self.publisher = self.create_publisher(HandState, self.state_topic, 10)
        # 订阅hand_command
        self.command_subscription = self.create_subscription(
            HandCommand,
            self.command_topic,
            self.command_callback,
            10,
            callback_group=self.callback_group,
        )
        # 循环发布
        self.timer = self.create_timer(
            self.publish_period,
            self.publish_hand_state,
            callback_group=self.callback_group,
        )

    def service_name(self, name):
        return f"{self.service_namespace}/{name}"

    # 缓存左手最后的preset_name
    def command_callback(self, msg: HandCommand):
        if msg.hand_id not in self.supported_hand_ids:
            self.get_logger().warn(f"忽略不支持的 hand_id={msg.hand_id}")
            return

        self.last_preset = msg.preset_name if msg.preset_name else "custom"

    def wait_for_future(self, future):
        # service可同步执行不阻塞
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None
   
    # 获取当前灵巧手关节位置
    def read_position(self, joint_id):
        req = GetNowPosition.Request()
        req.joint_id = joint_id
        result = self.wait_for_future(self.get_now_position_cli.call_async(req))
        return None if result is None else result.position
    
    # 发送获取关节位速度req
    def read_velocity(self, joint_id):
        req = GetNowPositionVelocity.Request()
        req.joint_id = joint_id
        result = self.wait_for_future(
            self.get_now_position_velocity_cli.call_async(req)
        )
        return None if result is None else result.velocity

    # 获取每一个关节电机的位置、速度
    def read_hand_motors(self):
        positions = []
        velocities = []
        for joint_id in self.left_hand_joint_ids:
            position = self.read_position(joint_id)
            velocity = self.read_velocity(joint_id)
            if position is None or velocity is None:
                return [], []
            positions.append(position)
            velocities.append(velocity)
        return positions, velocities

    # 发布单左手状态
    def publish_hand_state(self):
        msg = HandState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand_id = 0
        msg.preset_name = self.last_preset

        service_ready = (
            self.get_now_position_cli.service_is_ready()
            and self.get_now_position_velocity_cli.service_is_ready()
        )
        if service_ready:
            msg.motor_positions, msg.motor_velocities = self.read_hand_motors()
        else:
            msg.motor_positions = []
            msg.motor_velocities = []

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandStatePublisherSingle()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
