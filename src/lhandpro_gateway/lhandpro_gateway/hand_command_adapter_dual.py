"""Dual-hand HandCommand adapter for LHandPro services.

Current verified dual-EtherCAT layout:
  channel_indices=[1,2] in lhandpro_service
  hand_id=0 -> left hand -> global joints 1..6
  hand_id=1 -> right hand -> global joints 7..12
  hand_id=2 -> both hands with the same six-DOF preset or command
如果使用ros2启动lhandpro_service，其服务将在根命名空间下创建，使用以下参数启动此适配器：
  -p service_namespace:=/
如果lhandpro_service在/lhandpro_service命名空间下启动，请保持默认的service_namespace参数。
"""

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
        # "/" targets root services such as /set_enable. "/lhandpro_service"
        # targets namespaced services such as /lhandpro_service/set_enable.
        self.service_namespace = self.declare_parameter('service_namespace', '/lhandpro_service').value.rstrip('/')
        self.supported_hand_ids = tuple(self.declare_parameter('supported_hand_ids', [0, 1, 2]).value)
        # 2表示广播给左右手
        self.connected_hand_ids = tuple(self.declare_parameter('connected_hand_ids', [0, 1]).value)
        # 单手模式下，需要覆盖right_hand_joint_ids为[1, 2, 3, 4, 5, 6]
        self.left_hand_joint_ids = list(self.declare_parameter('left_hand_joint_ids', [1, 2, 3, 4, 5, 6]).value)
        self.right_hand_joint_ids = list(self.declare_parameter('right_hand_joint_ids', [7, 8, 9, 10, 11, 12]).value)
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

    def target_joint_groups(self, hand_id):
        if hand_id == 0 :
            return [self.left_hand_joint_ids]
        if hand_id == 1 :
            return [self.right_hand_joint_ids]
        if hand_id == 2:
            joint_groups = []
            joint_groups.append(self.left_hand_joint_ids)
            joint_groups.append(self.right_hand_joint_ids)
            return joint_groups
        return []

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
        joint_groups = self.target_joint_groups(msg.hand_id)
        if not joint_groups:
            self.get_logger().info(f'hand_id={msg.hand_id} 未配置为已接入手，忽略命令')
            return
        if any(len(joint_ids) != 6 for joint_ids in joint_groups):
            self.get_logger().info('检查hand_id对应的电机ID配置')
            return
        # 检查预设的open/middle/close命令
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
            self.process_command(msg)
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
        joint_groups = self.target_joint_groups(msg.hand_id)
        # 下使能
        if not msg.enabled:
            for joint_ids in joint_groups:
                for joint_id in joint_ids:
                    if not self.set_enable(joint_id, 0):
                        return
            return 
        # 上使能
        for joint_ids in joint_groups:
            for joint_id in joint_ids:
                if not self.set_enable(joint_id, 1):
                    return

        if msg.preset_name:
            positions = PRESETS[msg.preset_name]
        else:
            positions = msg.motor_positions
            
        # 设置电机
        if len(msg.motor_velocities) == 0:
            velocities = [self.default_velocity] * 6
        else:
            velocities = msg.motor_velocities

        for joint_ids in joint_groups:
            for index, joint_id in enumerate(joint_ids):
                if not self.set_position_velocity(joint_id, velocities[index]):
                    return
                if not self.set_position(joint_id, positions[index]):
                    return

        for joint_ids in joint_groups:
            for joint_id in joint_ids:
                if not self.move_motors(joint_id):
                    return

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

        
        
        
