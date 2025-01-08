#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import zarr
import numpy as np
from diffusion_policy.msg import obsdata
from sensor_msgs.msg import Image
import message_filters
from zarr_utils import ZarrDataCreator
from std_msgs.msg import String
import time
import os
import ffmpeg
from cv_bridge import CvBridge, CvBridgeError


class DataSubscriber:
    def __init__(self):
        # 初始化数据存储属性
        self.timestamps = []  # 初始化时间戳列表
        self.robot_eef_pose = []
        self.robot_eef_pose_vel = []
        self.robot_joint = []
        self.robot_joint_vel = []
        self.camera_dimensions = None
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

        # 创建 ZarrDataCreator 实例并初始化
        self.zarr_creator = ZarrDataCreator()
        self.zarr_creator.initialize_zarr_store('./real_pusht_2025_1_7/replay_buffer.zarr')

        rospy.loginfo("Zarr store initialized at './real_pusht_2025_1_7/replay_buffer.zarr'")

        # 初始化步数计数器
        self.step_idx = 1

        # 添加保存状态标志
        self.is_saving = False

        # 订阅键盘事件话题
        rospy.Subscriber('key_event', String, self.key_callback)

        # 视频录制相关
        self.bridge = CvBridge()  # 用于将ROS图像消息转换为OpenCV格式
        self.is_recording = False
        self.output_video_root_dir = './real_pusht_2025_1_7/videos'  # 根目录

        # 确保输出目录存在
        if not os.path.exists(self.output_video_root_dir):
            os.makedirs(self.output_video_root_dir)
            rospy.loginfo(f"Created output video root directory: {self.output_video_root_dir}")

        # 视频帧率控制
        self.frame_rate = 30.0
        self.last_frame_time = None

        # 视频录制参数
        self.frame_width = 1280  # 指定视频宽度
        self.frame_height = 720  # 指定视频高度
        self.fps = 30  # 指定视频帧率

        # T 键防止重复按下处理
        self.t_key_pressed = False
        self.t_cool_down = 1.0  # T 键的冷却时间，单位为秒
        self.last_t_press_time = 0.0

    def key_callback(self, msg):
        key = msg.data.lower()
        current_time = time.time()

        if key == 'c':  # 开始保存数据并开始视频录制
            self.is_saving = True
            self.toggle_video_recording()
            rospy.loginfo("Started saving data to Zarr.")
 
        elif key == 't':
            # 确保不会在冷却时间内多次触发 't' 键事件
            if not self.t_key_pressed and (current_time - self.last_t_press_time) > self.t_cool_down:
                self.is_saving = False
                self.t_key_pressed = True
                self.last_t_press_time = current_time
                self.zarr_creator.append_meta_data(self.step_idx)
                rospy.loginfo(f"Added step {self.step_idx} to episode_ends in Zarr store.")
                # 设置一个定时器来重置 t_key_pressed 标志位
                rospy.Timer(rospy.Duration(self.t_cool_down), self.reset_t_key_pressed, oneshot=True)

                # 保存当前视频
                self.save_current_video()

    def reset_t_key_pressed(self, event):
        self.t_key_pressed = False

    def toggle_video_recording(self):
        if self.is_recording:
            self.stop_video_recording()
        else:
            if self.camera_dimensions is not None:
                self.start_video_recording()
            else:
                rospy.logwarn("Cannot start recording; no camera dimensions available yet.")

    def start_video_recording(self):
        if not self.is_recording:
            rospy.loginfo("Starting video recording...")

            # 创建新的文件夹，并以数字命名
            new_folder = self.get_next_folder()
            os.makedirs(new_folder)
            rospy.loginfo(f"Created new recording folder: {new_folder}")

            # 获取相机的宽度和高度
            if self.camera_dimensions is not None:
                width, height = self.camera_dimensions
            else:
                width, height = self.frame_width, self.frame_height  # 使用默认的宽高

            # 设置视频文件保存路径
            self.output_video_path = os.path.join(new_folder, "0.mp4")

            # 使用 ffmpeg 开始视频录制
            self.ffmpeg_process = ffmpeg.input('pipe:0', format='rawvideo', pix_fmt='bgr24',
                                               s=f'{width}x{height}', r=self.fps) \
                                  .output(self.output_video_path, vcodec='libx264', pix_fmt='yuv420p') \
                                  .overwrite_output() \
                                  .run_async(pipe_stdin=True)

            self.is_recording = True

    def stop_video_recording(self):
        if self.is_recording:
            rospy.loginfo("Stopping video recording...")
            if self.ffmpeg_process:
                self.ffmpeg_process.stdin.close()  # 关闭标准输入流
                self.ffmpeg_process.wait()  # 等待 ffmpeg 进程结束
            self.is_recording = False
            rospy.loginfo(f"Stopped recording video to {self.output_video_path}")

    def save_current_video(self):
        if self.is_recording:
            rospy.loginfo("Saving current video...")
            self.stop_video_recording()  # 确保停止时正确关闭
            rospy.loginfo(f"Video saved to: {self.output_video_path}")
        else:
            rospy.logwarn("No video recording in progress. Skipping save.")

    def get_next_folder(self):
        """
        获取下一个编号的文件夹名称。
        """
        # 获取当前根目录下的所有文件夹
        existing_folders = [f for f in os.listdir(self.output_video_root_dir) if os.path.isdir(os.path.join(self.output_video_root_dir, f))]
        
        # 找到所有数字命名的文件夹
        numbered_folders = [int(f) for f in existing_folders if f.isdigit()]
        
        # 获取下一个编号，如果没有则从 0 开始
        if numbered_folders:
            next_folder_number = max(numbered_folders) + 1
        else:
            next_folder_number = 0

        # 返回新的文件夹路径
        return os.path.join(self.output_video_root_dir, str(next_folder_number))

    def callback(self, obs_msg, image_msg):
        """
        处理接收到的同步后的 '/robot_obs_data' 和 '/camera/color/image_raw' 话题数据
        """
        # 使用 header.stamp 来同步时间戳
        obs_timestamp = obs_msg.header.stamp
        image_timestamp = image_msg.header.stamp

        # 打印时间戳对比
        # rospy.loginfo(f"ObsData Timestamp: {obs_timestamp}")
        # rospy.loginfo(f"Image Timestamp: {image_timestamp}")

        # 如果时间戳差异太大，跳过当前回调
        timestamp_diff = abs(obs_timestamp.to_sec() - image_timestamp.to_sec())
        if timestamp_diff > 1.0:
            rospy.logwarn(f"Timestamp difference too large: {timestamp_diff} seconds. Skipping this callback.")

        # 将数据存储在对应的列表中
        self.timestamps.append(obs_timestamp.to_sec())
        self.robot_eef_pose.append(list(obs_msg.robot_eef_pose))
        self.robot_eef_pose_vel.append(list(obs_msg.robot_eef_pose_vel))
        self.robot_joint.append(list(obs_msg.robot_joint))
        self.robot_joint_vel.append(list(obs_msg.robot_joint_vel))

        # 模拟 action，使用 robot_joint_vel 作为 action 的代理
        action = obs_msg.robot_joint

        try:
            # 如果 is_saving 为 True，则将数据保存到 Zarr
            if self.is_saving:
                self.zarr_creator.append_data(
                    action,
                    self.robot_eef_pose[-1],
                    self.robot_eef_pose_vel[-1],
                    self.robot_joint[-1],
                    self.robot_joint_vel[-1],
                    0,  # stage
                    self.timestamps[-1],
                )
                self.step_idx += 1
                rospy.loginfo(f"Data appended to Zarr store at step {self.step_idx}")

            # 将图像消息转换为 OpenCV 格式并处理
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                if self.camera_dimensions is None:
                    self.camera_dimensions = (cv_image.shape[1], cv_image.shape[0])  # 记录相机的宽和高
                
                # 计算帧时间间隔
                current_time = time.time()
                if self.last_frame_time is None:
                    self.last_frame_time = current_time

                # 控制帧率
                time_diff = current_time - self.last_frame_time
                if time_diff >= 1.0 / self.frame_rate:
                    if self.is_recording and self.ffmpeg_process is not None:
                        # 将当前帧写入视频文件
                        self.ffmpeg_process.stdin.write(cv_image.tobytes())
                    self.last_frame_time = current_time

                # # 显示当前帧（如果需要）
                # cv2.imshow(self.display_window_name, cv_image)
                # cv2.waitKey(1)  # 必须调用 waitKey 才能更新显示窗口

            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert image message to OpenCV format: {e}")

        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")

def main():
    rospy.init_node('data_subscriber', anonymous=True)
    subscriber = DataSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()