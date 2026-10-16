import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import h5py
import numpy as np
import os
import cv2
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import time
import sys
from message_filters import ApproximateTimeSynchronizer, Subscriber
from datetime import datetime
from std_srvs.srv import SetBool

'''
data_dict = {
    # ----------ROBOT----------- 
    '/observations/qpos': Concat[ left_arm_qpos (7),          # absolute joint position
                                  left_gripper_qpos (1),  # normalized gripper position (0: close, 1: open)
                                  right_arm_qpos (7),         # absolute joint position
                                  right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)

    '/observations/qvel': Concat[ left_arm_qvel (7),          # absolute joint velocity (rad)
                                    left_gripper_qvel (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                    right_arm_qvel (7),         # absolute joint velocity (rad)
                                    right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
    # -------VR DATA--------------
    '/action': [left_arm_pose (7),             # position and quaternion for end effector                # need calculate
                left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)         
                right_arm_pose (7),            # position and quaternion for end effector
                right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)          # need calculate
    # ---------CAMERA-------------
    '/observations/images/
        render_cam': (480x640x3)        # h, w, c, dtype='uint8'
        left_shoulder_cam': (480x640x3) # h, w, c, dtype='uint8'
        right_shoulder_cam': (480x640x3)# h, w, c, dtype='uint8'
}

'''


class DataCollector(Node):
    def __init__(self):
        super().__init__('aloha_data_prepare_node')
        self.bridge = CvBridge()
        self.image_count = 0
        self.episode_idx = 0
        self.dataset_dir = './datasets'
        os.makedirs(self.dataset_dir, exist_ok=True)
        self.target_message_count = 10
        # self.camera_names = ['right_cam', 'front_cam', 'left_cam','left_fixed_cam','right_fixed_cam']
        self.camera_names = ['right_cam', 'front_cam', 'left_cam','head_cam']
        self.data_dict1 = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/action': [],
            **{f'/observations/images/{cam_name}': [] for cam_name in self.camera_names},
        }
        #  双缓冲
        # self.data_dict1 = {key: [] for key in self.data_dict}
        self.data_dict2 = {key: [] for key in self.data_dict1} 
        self.current_buffer = 1  # 1 for data_dict1, 2 for data_dict2

        self.recording = False
        self.file_saved = False

        # # robot 
        # qpos
        self.left_arm_qpos = Subscriber(self, JointState, '/left_arm/joint_states')
        self.right_arm_qpos = Subscriber(self, JointState, '/right_arm/joint_states')
        self.left_gripper_qpos = Subscriber(self, JointState, '/left_gripper/current_pos')
        self.right_gripper_qpos = Subscriber(self, JointState, '/right_gripper/current_pos')

        # # qvel
        # TODO:
        self.left_arm_qvel = Subscriber(self, JointState, '/left_arm/joint_states_vel')
        self.right_arm_qvel = Subscriber(self, JointState, '/right_arm/joint_states_vel')
        # self.left_gripper_qvel = self.left_gripper_qpos
        # self.right_gripper_qvel = self.right_gripper_qpos

        # action
        self.left_vr_pose = Subscriber(self, JointState, '/left_arm/robot_command_servo_p')
        self.right_vr_pose = Subscriber(self, JointState, '/right_arm/robot_command_servo_p')

        self.left_vr_gripper = Subscriber(self, JointState, '/left_gripper/set_pos')
        self.right_vr_gripper = Subscriber(self, JointState, '/right_gripper/set_pos')
        
        # images
        self.right_cam_sub = Subscriber(self, Image, '/right_camera/color/image_raw')
        self.front_cam_sub = Subscriber(self, Image, '/front_camera/color/image_raw')
        self.left_cam_sub = Subscriber(self, Image, '/left_camera/color/image_raw')
        self.head_cam_sub = Subscriber(self, Image, '/head_camera/color/image_raw')
        #self.right_fixed_cam_sub = Subscriber(self, Image, '/right_fixed_camera/color/image_raw')
        # Time Synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [#self.right_fixed_cam_sub,
             self.left_arm_qpos, self.right_arm_qpos, 
             self.left_gripper_qpos, self.right_gripper_qpos,
             self.left_arm_qvel,self.right_arm_qvel,
             self.left_vr_pose, self.right_vr_pose,
             self.left_vr_gripper, self.right_vr_gripper,
             self.right_cam_sub, self.front_cam_sub, self.left_cam_sub,
             self.head_cam_sub
            ],
            queue_size=1,  # 增加queue_size到50
            slop=1,       # 允许的时间偏差
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info("ApproximateTimeSynchronizer initialized and callback registered.")

        # 服务端
        self.start_recording_service = self.create_service(SetBool, 'start_recording', self.start_recording_callback)
        self.save_data_service = self.create_service(SetBool, 'save_data', self.save_data_callback)

        self.get_logger().info("ApproximateTimeSynchronizer initialized and callback registered.")
        self.get_logger().info("Services for start_recording and save_data are ready.")




    def start_recording_callback(self, request, response):
        if request.data:   
            self.recording = True
            # self.file_saved = False  
            self.get_logger().info('Start recording data.')
        response.success = True  # 服务响应对象，用于返回操作结果
        return response

    def save_data_callback(self, request, response):
        if request.data and self.recording and not self.file_saved:
            self.saving = True  # 
            self.file_saved = True

            current_dict = self.data_dict1 if self.current_buffer == 1 else self.data_dict2

            self.save_hdf5(current_dict)

            self.current_buffer = 2 if self.current_buffer == 1 else 1  # 切换缓冲区
            self.data_dict1 = {key: [] for key in self.data_dict1}  # 重置当前缓冲区
            self.data_dict2 = {key: [] for key in self.data_dict2}  # 重置另一个缓冲区
            self.saving = False

            self.recording = False
            self.file_saved = False
            # self.data_dict = {key: [] for key in self.data_dict}  # reset
            
            self.get_logger().info('Data saved and reset for next recording.')
        response.success = True
        return response

    def sync_callback(self, #right_fixed_cam_msg,
                      left_arm_qpos_msg, right_arm_qpos_msg, 
                      left_gripper_msg, right_gripper_msg,
                      left_arm_qvel_msg,right_arm_qvel_msg,
                      left_vr_pose_msg, right_vr_pose_msg, 
                      left_vr_gripper_msg,  right_vr_gripper_msg,
                      right_cam_msg, front_cam_msg, left_cam_msg,
                      head_cam_msg):
        
        if self.recording:
           
            current_dict = self.data_dict1 if self.current_buffer == 1 else self.data_dict2
            # 同步处理所有消息
            print('processing----')
            self.process_qpos(left_arm_qpos_msg, right_arm_qpos_msg,
                            left_gripper_msg, right_gripper_msg,current_dict
                            )
            self.process_qvel(left_arm_qvel_msg,right_arm_qvel_msg,
                            left_gripper_msg, right_gripper_msg,current_dict
                            )
            self.process_action(left_vr_pose_msg, right_vr_pose_msg,
                                left_vr_gripper_msg, right_vr_gripper_msg,current_dict
                                )
            self.process_images(right_cam_msg, front_cam_msg, left_cam_msg,
                                # left_fixed_cam_msg,right_fixed_cam_msg,
                                head_cam_msg,
                                current_dict)





    def process_qpos(self, left_arm_qpos_msg, right_arm_qpos_msg,
                     left_gripper_msg, right_gripper_msg,current_dict
                     ):
        print('1. process_qpos')
        left_arm_qpos = left_arm_qpos_msg.position[:7]
        right_arm_qpos = right_arm_qpos_msg.position[:7]
        left_gripper_qpos = left_gripper_msg.position[0]
        right_gripper_qpos = right_gripper_msg.position[0]
        qpos = np.concatenate([
                            left_arm_qpos,
                            np.array([left_gripper_qpos]), 
                            right_arm_qpos,
                            np.array([right_gripper_qpos]) 
                        ])
        current_dict['/observations/qpos'].append(qpos)
        print('/observations/qpos:',len(current_dict['/observations/qpos']))

    def process_qvel(self, left_arm_qvel_msg, right_arm_qvel_msg,
                     left_gripper_msg, right_gripper_msg,current_dict
                    ):
        print('2. process_qvel')
        left_arm_qvel = left_arm_qvel_msg.velocity[:7]
        right_arm_qvel = right_arm_qvel_msg.velocity[:7]
        left_gripper_qvel = left_gripper_msg.position[0]  # 假设夹爪的qvel与qpos相同
        right_gripper_qvel = right_gripper_msg.position[0]  # 假设夹爪的qvel与qpos相同
        qvel = np.concatenate([
                            left_arm_qvel,
                            np.array([left_gripper_qvel]), 
                            right_arm_qvel,
                            np.array([right_gripper_qvel]) 
                        ])
        current_dict['/observations/qvel'].append(qvel)
        print('/observations/qvel:',len(current_dict['/observations/qvel']))

    def process_action(self, left_vr_pose_msg, right_vr_pose_msg,
                       left_vr_gripper_msg, right_vr_gripper_msg,current_dict
                       ):
        print('3. process_action')
        left_vr_pose = left_vr_pose_msg.position[:6]
        right_vr_pose = right_vr_pose_msg.position[:6]
        # left_vr_gripper = left_vr_gripper_msg.position[:1]
        # right_vr_gripper = right_vr_gripper_msg.position[:1]
        left_vr_gripper = left_vr_gripper_msg.position[0]
        right_vr_gripper = right_vr_gripper_msg.position[0]
        action = np.concatenate([left_vr_pose , np.array([left_vr_gripper]) , right_vr_pose , np.array([right_vr_gripper])])
        current_dict['/action'].append(action)
        print('/action:',len(current_dict['/action']))


    def process_images(self,right_cam_msg, front_cam_msg, left_cam_msg,
                       head_cam_msg,current_dict):
        for cam_msg, cam_name in zip([right_cam_msg, front_cam_msg, left_cam_msg,
                                      head_cam_msg], self.camera_names):
            img = self.bridge.imgmsg_to_cv2(cam_msg, desired_encoding='passthrough')
            img_resized = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LANCZOS4)
            img = np.array(img_resized, dtype=np.uint8)
            current_dict[f'/observations/images/{cam_name}'].append(img)
        print('images appended!')
          
    def save_hdf5(self,current_dict):
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        dataset_path = os.path.join(self.dataset_dir, f'episode_{timestamp}')
        max_timesteps = len(current_dict['/observations/images/left_cam'])
        print(f"Saving {max_timesteps} timesteps to {dataset_path}.hdf5")
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            obs = root.create_group('observations')
            qpos = obs.create_dataset('qpos', (max_timesteps, 16), data=current_dict['/observations/qpos']) # 16
            qvel = obs.create_dataset('qvel', (max_timesteps, 16), data=current_dict['/observations/qvel']) # 16
            action = root.create_dataset('action', (max_timesteps, 14), data=current_dict['/action']) # 16
            image = obs.create_group('images')
            # print('image:',image)
            for cam_name in self.camera_names:
                image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                    data=current_dict[f'/observations/images/{cam_name}'])
            



def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    rclpy.spin(data_collector)
    data_collector.destroy_node()
    rclpy.shutdown()



class Logger(object):
    def __init__(self, file_name = 'temp.log', stream = sys.stdout) -> None:
        self.terminal = stream
        current_time = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.log_file_name = f'log/{file_name}_{current_time}.log'
        if not os.path.exists('log'):
            os.makedirs('log')
        self.log = open(self.log_file_name, "a")

    def flush(self):
        self.log.flush()


if __name__ == '__main__':
    logger = Logger()
    sys.stdout = logger
    sys.stderr = logger
    main()