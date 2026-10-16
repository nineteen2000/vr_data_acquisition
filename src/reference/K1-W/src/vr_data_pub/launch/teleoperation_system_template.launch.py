'''
Author: Ding Cheng  Email: cheng.ding@jaka.com
Date: 2025-04-15 14:47:56
LastEditors: Ding Cheng  Email: cheng.ding@jaka.com
LastEditTime: 2025-04-15 14:53:08
FilePath: /K1-W/src/vr_data_pub/launch/teleoperation_system_template.launch.py
Description: 

Copyright (c) 2025 by JAKA Robotics Co., Ltd. , All Rights Reserved. 
'''
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vr_data_pub',
            executable='vr_data_pub',
            parameters=[{'server_port': 8018}],  # 设置robot_ip参数
            name='vr_socket_server'
        ),
        Node(
            package='vr_data_pub',
            executable='vr_data_distributer',
            name='vr_data_distributor'
        ),
        Node(
            package='robohub',
            parameters=[{'robot_ip': '192.168.1.177','robot_running_mode':'teleop'}],  #teleop or act_exe 
            executable='jaka_driver_node',
            name='jaka_robot_driver'
        ),   
        # # if you have dh gripper with usb connection, uncomment this part 
         Node(
             package='k1_robot',
             executable='control_gripper',
             name='control_gripper'
         ),
        Node(
            package='vr_data_pub',
            namespace='/left_arm',
            parameters = [{'robot_name':'left'}],
            executable='vr_robot_pose_converter_30degree_servo_p',
            name='vr_robot_pose_converter'
        ),
        Node(
            package='vr_data_pub',
            namespace='/right_arm',
            parameters=[{'robot_name': 'right'}],
            executable='vr_robot_pose_converter_30degree_servo_p',
            name='vr_robot_pose_converter'
        )#,


    ])
