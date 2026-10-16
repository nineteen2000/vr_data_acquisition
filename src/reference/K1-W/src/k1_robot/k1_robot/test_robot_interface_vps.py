import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from k1_msgs.srv import JointMove, LinearMove, KineInverse7
from std_srvs.srv import SetBool
import sys
import time
import math

class JakaRobotClient(Node):
    def __init__(self):
        super().__init__('jaka_robot_client')

        # Clients for services
        self.joint_move_client = self.create_client(JointMove, 'joint_move')
        self.linear_move_client = self.create_client(LinearMove, 'linear_move')
        self.kine_inverse_client = self.create_client(KineInverse7, 'kine_inverse7')
        self.servo_move_enable_client = self.create_client(SetBool, 'left_arm/servo_move_enable')

        # Publisher to send servo_j7 commands
        self.publisher_servo_j = self.create_publisher(JointState, 'robot_command_servo_j', 10)
        self.publisher_servo_p = self.create_publisher(JointState, 'robot_command_servo_p', 10)
        
        # Wait for the services to be available
        while not self.joint_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service joint_move not available, waiting again...')
        # while not self.linear_move_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service linear_move not available, waiting again...')
        # while not self.kine_inverse_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service kine_inverse7 not available, waiting again...')
        # while not self.servo_move_enable_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service servo_move_enable not available, waiting again...')

    def call_joint_move(self, robot_arm_index, joint_positions, move_mode, is_blocking, speed):
        request = JointMove.Request()
        request.robot_arm_index = robot_arm_index
        request.joint_positions = joint_positions
        request.move_mode = move_mode
        request.is_blocking = is_blocking
        request.speed = speed
        future = self.joint_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_linear_move(self, end_position, move_mode, is_blocking, speed):
        request = LinearMove.Request()
        request.end_position = end_position
        request.move_mode = move_mode
        request.is_blocking = is_blocking
        request.speed = speed
        future = self.linear_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_kine_inverse7(self, ref_joint_pos, cartesian_pose):
        request = KineInverse7.Request()
        request.ref_joint_pos = ref_joint_pos
        request.cartesian_pose = cartesian_pose
        future = self.kine_inverse_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_servo_move_enable(self, enable):
        request = SetBool.Request()
        request.data = enable
        future = self.servo_move_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def publish_servo_j7_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        msg.position = joint_positions
        self.publisher_servo_j.publish(msg)

    def publish_servo_p_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']   # abs without name, relative with name
        msg.position = joint_positions
        self.publisher_servo_p.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JakaRobotClient()
    action_publisher = Node('action_publisher')
    left_gripper_pub = action_publisher.create_publisher(JointState, '/left_gripper/set_pos', 10)
    right_gripper_pub = action_publisher.create_publisher(JointState, '/right_gripper/set_pos', 10)
    rclpy.spin_once(action_publisher, timeout_sec=1)
    for i in range(100):
        left_gripper_cmd = JointState()
        right_gripper_cmd = JointState()
        left_gripper_cmd.position = [0.3]
        right_gripper_cmd.position = [0.3]
        left_gripper_pub.publish(left_gripper_cmd)
        right_gripper_pub.publish(right_gripper_cmd)
    try:
        # Example usage of the client methods
        # Replace these calls with your own logic or command-line arguments
        print("before")
        # response = node.call_servo_move_enable(False)
        # print(f"Servo Move Enable Response: {response.message}")
        
       # Call joint_move service
        # response = node.call_joint_move(0, [-1.359192608283104, -0.9613273519984766, 2.034198696491911, 
                                #   -1.481068949949868, 0.3909188458616899, -0.9122486934323962, 2.582808040271289], 0, True, 1.0)
        # response = node.call_joint_move(1, [4.373533306109991, 1.05267788504786, -4.856727709524621, -1.5725765626319308,
                                #    -0.5133885994741321, 1.0164448497764575, -1.887399053106668], 0, True, 1.0)

        response = node.call_joint_move(-1, [
            -0.2973691979547939
, 0.1288402053822214
, 0.9530021314664638
, -1.440490044841
, -0.3925943619436045
, -0.9754994255246707
, 1.333745707789027
, 0.2405412775098585
, 0.1426981196430564
, -0.19867082875451453
, -1.4997614262387275
, -0.35774013678127775
, 1.0328334914526844
, 0.9893573397855057
], 0, True, 1.0)
                                        
                                        # [-1.359192608283104, -0.9613273519984766, 2.034198696491911, 
                                #   -1.481068949949868, 0.3909188458616899, -0.9122486934323962, 2.582808040271289,
                                #   4.373533306109991, 1.05267788504786, -4.856727709524621, -1.5725765626319308,
                                #    -0.5133885994741321, 1.0164448497764575, -1.887399053106668]
        time.sleep(1)
        print("after")
#       
#       print(f"Joint Move Response: {response.message}")

#        # Call linear_move service
#        response = node.call_linear_move([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0, True, 1.0)
#        print(f"Linear Move Response: {response.message}")
#
#        # Call kine_inverse7 service
#        response = node.call_kine_inverse7([-1.3644285960390872,-0.9665807930469796,2.0278806045996913,-1.4563201811565885
#                              ,0.005585053606381855,-0.6764372581954423,2.3449413107588065], 
#                              [360.0, 510.0, 250.0, -142.638/180*math.pi, 1.529/180*math.pi, -91.195/180*math.pi])
#        print(f"Kine Inverse Response: {response.message}")
#        if response.success:
#            print(f"Joint Positions: {response.joint_positions}")


        #  # Call servo_move_enable service
        response = node.call_servo_move_enable(True)
        print(f"Servo Move Enable Response: {response.message}")
 
        # # # # Publish a servo_j7 command
        # # servo_joint_pos_abs_left= [-1.359192608283104, -0.9613273519984766, 2.034198696491911, 
        # #                           -1.481068949949868, 0.3909188458616899, -0.9122486934323962, 2.582808040271289]
        # # servo_joint_pos_abs_right= [4.373533306109991, 1.05267788504786, -4.856727709524621, -1.5725765626319308,
        # #                            -0.5133885994741321, 1.0164448497764575, -1.887399053106668]
 
        # # servo_joint_pos_relative = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0080000]
        # # for i in range(20):
        # #     servo_joint_pos_abs_left[6] += servo_joint_pos_relative[6]  
        # #     servo_joint_pos_abs_right[6] += servo_joint_pos_relative[6]  
        # #     dual_abs = servo_joint_pos_abs_left+servo_joint_pos_abs_right
        # #     node.publish_servo_j7_command(dual_abs)
        # #     # node.publish_servo_j7_command(servo_joint_pos_relative)
        # #     time.sleep(0.0080)
        
        # # time.sleep(1)

        servo_pose_relative = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        servo_pose_abs_left = [300.417, 477.886, 38.860, 98.712/180*math.pi, -34.909/180*math.pi, 47.927/180*math.pi]
        servo_pose_abs_right = [322.669, -460.665, 166.374, 110.262/180*math.pi, -43.331/180*math.pi, 121.047/180*math.pi]
        for i in range(50):

            servo_pose_abs_left[2] += servo_pose_relative[2]  
            servo_pose_abs_right[2] += servo_pose_relative[2]  
            servo_pose_abs = servo_pose_abs_left + servo_pose_abs_right
            node.publish_servo_p_command(servo_pose_abs)
            # servo_pose_rel = servo_pose_relative + servo_pose_relative
            # node.publish_servo_p_command(servo_pose_rel)
            time.sleep(0.012)
        
        time.sleep(1)


        response = node.call_servo_move_enable(False)
        print(f"Servo Move Enable Response: {response.message}")


    except KeyboardInterrupt:
        pass

    finally:
        action_publisher.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
