#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from diffusion_policy.msg import obsdata
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import requests
import cv2 as cv
from std_msgs.msg import Float32MultiArray  # 用于发布动作数据的消息类型
from frame_msgs.msg import set_servo_as
class DataSaverNode:
    def __init__(self):
        # 创建消息过滤器，订阅相应话题
        self.sub_obs_data = message_filters.Subscriber('/robot_obs_data', obsdata)
        self.sub_camera_image = message_filters.Subscriber('/camera/color/image_raw', Image)

        # 设置时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_obs_data, self.sub_camera_image],
            queue_size=10,
            slop=0.1  # 时间窗口设置为0.1秒
        )
        self.ts.registerCallback(self.callback)

        # 用于将图像消息转换为OpenCV格式
        self.bridge = CvBridge()

        # 创建一个 Publisher 用于发布 action 数据
        self.action_pub = rospy.Publisher('/main_control_servo', set_servo_as, queue_size=10)

        rospy.loginfo("DataSaverNode initialized and waiting for messages...")

    def callback(self, obs_msg, image_msg):
        """
        处理接收到的同步后的 '/robot_obs_data' 和 '/camera/color/image_raw' 话题数据
        """
        try:
            # 将机器人观测数据（arm_pos, arm_quat, gripper_pos）存储到字典中
            robot_eef_pose = np.array(obs_msg.robot_joint)  # robot_eef_pose 格式为 [x, y, z, qx, qy, qz]
            # print(robot_eef_pose)
            # 将图像数据转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            # 转换图像为目标形状 (3, 240, 320)
            camera_0_image = cv.resize(cv_image, (320, 240))  # 缩放图像至 240x320
            camera_0_image = camera_0_image.transpose(2, 0, 1)  # 转置为 (3, 240, 320)

            # 创建符合要求的字典格式
            req = {
                'obs': {
                    'camera_0': camera_0_image.astype(np.uint8).tolist(),  # 将图像数据转换为 uint8 格式并转换为列表
                    'robot_eef_pose': robot_eef_pose.tolist()  # 将 NumPy 数组转换为列表
                }
            }

            # 输出调试信息
            rospy.loginfo("Data updated in the dictionary.")

            # 将生成的 req 通过 HTTP POST 请求发送
            action = self.send_data_to_server(req)

            # 如果返回的动作不为空，进行处理
            if action is not None:
                rospy.loginfo(f"Received action from server: {action}")
                self.publish_action(action)  # 发布动作数据

        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image message to OpenCV format: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")

    def send_data_to_server(self, data):
        """
        通过 HTTP POST 请求将数据发送给服务器，并接收返回的动作。
        """
        url = "http://192.168.0.240:5000/send_data"  # 服务器的 URL
        
        try:
            # 发送 POST 请求，json 数据格式
            response = requests.post(url, json=data)
            
            # 打印服务器响应
            if response.status_code == 200:
                rospy.loginfo(f"Data successfully sent to server. Response: {response.json()}")
                
                # 从服务器的响应中提取动作数据
                response_data = response.json()
                
                # 如果服务器返回了动作数据，提取并返回
                if 'action' in response_data:
                    return response_data['action']
                else:
                    rospy.logwarn("No action returned in the server response.")
                    return None
            else:
                rospy.logwarn(f"Failed to send data. Server Response: {response.status_code} {response.text}")
                return None
        except Exception as e:
            rospy.logerr(f"Error sending data to server: {e}")
            return None

    def publish_action(self, action):
        """
        发布接收到的动作数据到 ROS 话题。
        """
        action_msg = set_servo_as()
        
        # 取出第二个元素并取反
        negated_second_element = -action[1]
        
        # 创建新的列表，将取反后的元素插入到第三个位置（索引为2）
        caition = action[:2] + [negated_second_element] + action[2:]
        
        action_msg.servo_target_angle.data = caition  # 将动作数据填充到消息中
        
        self.action_pub.publish(action_msg)  # 发布到 '/generated_action' 话题
        rospy.loginfo(f"Published caition: {caition}")

def main():
    rospy.init_node('data_saver_node', anonymous=True)
    data_saver = DataSaverNode()
    rospy.spin()

if __name__ == '__main__':
    main()




