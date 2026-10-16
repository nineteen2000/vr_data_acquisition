"""HandCommand messages to LHandPro services."""

import rclpy
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lhandpro_gateway.grasp_presets import PRESETS

# 导入服务接口
from vr_robot_interfaces.msg import HandCommand
from lhandpro_interfaces.srv import SetEnable, SetPosition, SetPositionVelocity, SetControlMode, MoveMotors, HomeMotors


class HandCommandAdapter(Node):
    def __init__(self):
        super().__init__('hand_command')
        self.command_topic = self.declare_parameter('command_topic', '/hand_command').value
        self.applied_command_topic = self.declare_parameter(
            'applied_command_topic', '/hand_applied_command'
        ).value
        self.service_namespace = self.declare_parameter('service_namespace', '/lhandpro_service').value.rstrip('/')
        self.supported_hand_ids = tuple(self.declare_parameter('supported_hand_ids', [0]).value)
        self.default_velocity = self.declare_parameter('default_velocity', 10000).value
        self.max_velocity = self.declare_parameter('max_velocity', 20000).value
        self.min_position = self.declare_parameter('min_position', 0).value
        self.max_position = self.declare_parameter('max_position', 10000).value
        self.default_hand_id = 0
        # 回调内会同步等待 service future，多线程 + 可重入回调组避免自身阻塞响应回调。
        self.callback_group = ReentrantCallbackGroup()

        self.get_hand_command = self.create_subscription(
            HandCommand, 
            self.command_topic,
            self.hand_callback,
            10,
            callback_group=self.callback_group
        )
        self.applied_command_pub = self.create_publisher(HandCommand, self.applied_command_topic, 10)
        
        # 创建客户端
        self.set_enable_cli = self.create_client(SetEnable, self.service_name('set_enable'), callback_group=self.callback_group)
        self.set_position_cli = self.create_client(SetPosition, self.service_name('set_position'), callback_group=self.callback_group)
        self.set_position_velocity_cli = self.create_client(SetPositionVelocity, self.service_name('set_position_velocity'), callback_group=self.callback_group)
        self.set_control_mode_cli = self.create_client(SetControlMode, self.service_name('set_control_mode'), callback_group=self.callback_group)
        self.move_motors_cli = self.create_client(MoveMotors, self.service_name('move_motors'), callback_group=self.callback_group)
        self.home_motors_cli = self.create_client(HomeMotors, self.service_name('home_motors'), callback_group=self.callback_group)

        self.timer = self.create_timer(1, self.timer_callback, callback_group=self.callback_group)

        self.services_ready = False
        self.pending_command = None   # 是否存在新任务
        self.busy = False

    def service_name(self, name):
        return f'{self.service_namespace}/{name}'

    def wait_for_future(self, future):
        # 节点由 MultiThreadedExecutor spin，这里只等待其它线程处理 service 响应
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        return future.result() if future.done() else None

    # 安全检查回调
    def hand_callback(self, msg:HandCommand):
        # 安全检查
        if msg.hand_id not in self.supported_hand_ids:
            self.get_logger().info('检查hand_id输入')
            return 
        if msg.preset_name:
            if msg.preset_name not in PRESETS:
                self.get_logger().info('检查preset_name输入')
                return
            positions = PRESETS[msg.preset_name]
        else:
            if len(msg.motor_positions) != 6:
                self.get_logger().info('检查电机输入')
                return
            positions = msg.motor_positions

        if len(msg.motor_velocities) not in (0, 6):
            self.get_logger().info('检查电机速度数量')
            return
        if any(p < self.min_position or p > self.max_position for p in positions):
            self.get_logger().info('检查电机位置范围')
            return

        if any(abs(v) > self.max_velocity for v in msg.motor_velocities):
            self.get_logger().info('检查电机速度')
            return 
        self.pending_command = msg

    def timer_callback(self):
        if not self.check_services_ready():
            self.get_logger().info('等待服务上线')
            return
        if self.busy:
            return
        if self.pending_command is None:
            return

        self.services_ready = True
        msg = self.pending_command
        self.pending_command = None
        self.busy = True
        try:
            success = self.process_command(msg)
            if not success:
                self.get_logger().error('手部命令下发失败')
        finally:
            self.busy = False

    # 检查服务是否启动
    def check_services_ready(self):
        clients = [
            self.set_enable_cli,
            self.set_position_cli,
            self.set_position_velocity_cli,
            self.move_motors_cli,
        ]
        return all(cli.service_is_ready() for cli in clients)
    
    # 执行
    def process_command(self,msg:HandCommand):
        joint_ids = [1, 2, 3, 4, 5, 6]
        if not msg.enabled:
            return self.set_enable(0, 0)
        if not self.set_enable(0, 1):
            return False

        if msg.preset_name:
            positions = PRESETS[msg.preset_name]
        else:
            positions = msg.motor_positions
            
        # 设置电机
        if len(msg.motor_velocities) == 0:
            velocities = [self.default_velocity] * 6
        else:
            velocities = msg.motor_velocities

        # 遍历每个关节
        for j in joint_ids:
            if not self.set_position_velocity(j, velocities[j - 1]):
                return False
            if not self.set_position(j, positions[j - 1]):
                return False

        if not self.move_motors(0):
            return False
        self.publish_applied_command(msg, positions, velocities)
        return True

    def publish_applied_command(self, source_msg, positions, velocities):
        msg = HandCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand_id = source_msg.hand_id
        msg.preset_name = source_msg.preset_name
        msg.motor_positions = list(positions)
        msg.motor_velocities = list(velocities)
        msg.enabled = source_msg.enabled
        self.applied_command_pub.publish(msg)

    def set_enable(self, joint_id, enable):
        req = SetEnable.Request()
        req.joint_id = joint_id
        req.enable = enable
        future = self.set_enable_cli.call_async(req)
        result = self.wait_for_future(future)
        if result is not None and result.result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置使能成功')
            return True
        else:
            self.get_logger().error(f'设置使能失败 (joint_id={joint_id})')
            return False    

    def set_position(self, joint_id, position):
        req = SetPosition.Request()
        req.joint_id = joint_id
        req.position = position
        future = self.set_position_cli.call_async(req)
        result = self.wait_for_future(future)
        if result is not None and result.result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置位置 {position} 成功')
            return True
        else:
            self.get_logger().error(f'设置位置失败 (joint_id={joint_id}, position={position})')
            return False

    def set_position_velocity(self, joint_id, velocity):
        req = SetPositionVelocity.Request()
        req.joint_id = joint_id
        req.velocity = velocity
        future = self.set_position_velocity_cli.call_async(req)
        result = self.wait_for_future(future)
        if result is not None and result.result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置速度 {velocity} 成功')
            return True
        else:
            self.get_logger().error(f'设置速度失败 (joint_id={joint_id}, velocity={velocity})')
            return False
    
    def set_control_mode(self, joint_id, mode):
        req = SetControlMode.Request()
        req.joint_id = joint_id
        req.mode = mode
        future = self.set_control_mode_cli.call_async(req)
        result = self.wait_for_future(future)
        if result is not None and result.result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置控制模式 {mode} 成功')
        else:
            self.get_logger().error(f'设置控制模式失败 (joint_id={joint_id}, mode={mode})')

    # 统一运动
    def move_motors(self, joint_id):
        req = MoveMotors.Request()
        req.joint_id = joint_id
        future = self.move_motors_cli.call_async(req)
        result = self.wait_for_future(future)
        if result is not None and result.result == 0:
            self.get_logger().debug(f'关节 {joint_id} 下发运动成功')
            return True
        else:
            self.get_logger().error(f'下发运动失败 (joint_id={joint_id})')
            return False

    def home_motors(self, joint_id):
        req = HomeMotors.Request()
        req.joint_id = joint_id
        future = self.home_motors_cli.call_async(req)
        result = self.wait_for_future(future)
        if result is not None and result.result == 0:
            self.get_logger().debug(f'关节 {joint_id} 下发回零成功')
        else:
            self.get_logger().error(f'下发回零失败 (joint_id={joint_id})')


def main(args=None):
    rclpy.init(args=args)
    node = HandCommandAdapter()
    # service 响应需要在 timer 回调等待期间被其它线程处理。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

        
        
        
