#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from diffusion_policy_obs.msg import status_data
from diffusion_policy_obs.msg import obsdata
from diffusion_policy_obs.srv import ProcessData, ProcessDataResponse
import threading
import cv2

class DataSaverNode:
    def __init__(self, camera_index=[0]):
        """
        初始化 DataSaverNode。

        参数:
            camera_index (list): 摄像头的索引列表（默认 [0]）。
        """
        # 初始化线程锁
        self.lock = threading.Lock()

        # 初始化摄像头
        self.camera_index = camera_index
        self.caps = {idx: cv2.VideoCapture(idx) for idx in self.camera_index}  # 初始化摄像头对象
        for idx in self.camera_index:
            if not self.caps[idx].isOpened():
                rospy.logerr(f"错误：无法打开摄像头 {idx}。")
                raise RuntimeError(f"无法打开摄像头 {idx}。")

        # 初始化字典
        self.req = {
            'obs': {
                'camera_0': np.zeros((3, 240, 320), dtype=np.uint8),  # 默认图像数据
                'robot_joint': np.zeros(7, dtype=np.float32)  # 默认机器人关节数据
            }
        }

        # 订阅机器人观测数据话题
        self.obs_sub = rospy.Subscriber('/robot_obs_data', obsdata, self.obs_callback)

        # 创建一个服务服务器
        self.service = rospy.Service('process_data', ProcessData, self.handle_process_data)

        # 启动摄像头数据捕获线程
        self.camera_thread = threading.Thread(target=self.capture_camera_data)
        self.camera_thread.daemon = True
        self.camera_thread.start()

        rospy.loginfo("DataSaverNode initialized and waiting for messages...")

    def capture_camera_data(self):
        """
        从摄像头捕获数据并更新 self.req 中的图像数据。
        """
        while not rospy.is_shutdown():
            for idx in self.camera_index:
                ret, frame = self.caps[idx].read()  # 读取帧
                if not ret:
                    rospy.logerr(f"错误：无法捕获摄像头 {idx} 的帧。")
                    continue

                # 转换图像为目标形状 (3, 240, 320)
                resized_frame = cv2.resize(frame, (320, 240))  # 缩放图像至 240x320
                resized_frame = resized_frame.transpose(2, 0, 1)  # 转置为 (3, 240, 320)

                with self.lock:
                    # 更新图像数据
                    self.req['obs'][f'camera_{idx}'] = resized_frame

    def obs_callback(self, obs_msg):
        """
        处理接收到的 '/robot_obs_data' 话题数据。
        """
        try:
            robot_joint = np.array(obs_msg.robot_joint)
            with self.lock:
                self.req['obs']['robot_joint'] = robot_joint

        except Exception as e:
            rospy.logerr(f"Error processing robot observation data: {e}")

    def handle_process_data(self, request):
        """
        处理服务请求。
        """
        with self.lock:
            try:
                # 检查数据是否完整
                if 'camera_0' not in self.req['obs'] or 'robot_joint' not in self.req['obs']:
                    rospy.logwarn("Incomplete data in self.req.")
                    return ProcessDataResponse(success=False, data=status_data())

                # 将 NumPy 数组转换为 ObsData 消息
                obs_data = status_data()
                obs_data.camera_0 = self.req['obs']['camera_0'].flatten().tolist()  # 展平为一维列表
                obs_data.robot_joint = self.req['obs']['robot_joint'].tolist()

                # 打印调试信息
                rospy.loginfo(f"Camera data shape: {self.req['obs']['camera_0'].shape}")
                rospy.loginfo(f"Robot joint data: {obs_data.robot_joint}")

                return ProcessDataResponse(success=True, data=obs_data)
            except Exception as e:
                rospy.logerr(f"Error handling service request: {e}")
                return ProcessDataResponse(success=False, data=status_data())

    def __del__(self):
        """
        释放摄像头资源。
        """
        for idx in self.camera_index:
            if self.caps[idx].isOpened():
                self.caps[idx].release()


def main():
    rospy.init_node('data_saver_node', anonymous=True)
    # 列出所有可用的摄像头
    def list_cameras(max_tests=5):
        for i in range(max_tests):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"摄像头 {i} 可用。")
                cap.release()
            else:
                print(f"摄像头 {i} 不可用。")
    data_saver = DataSaverNode(camera_index=[0])  
    rospy.spin()


if __name__ == '__main__':
    main()