#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from diffusion_policy_obs.msg import obsdata
from zarr_utils import ZarrDataCreator
import cv2
import subprocess
import time
from pynput import keyboard  # 使用 pynput 监听键盘事件
import threading
import ffmpeg

class DataSubscriber:
    def __init__(self, fps=30, camera_index=0, output_videos='./real_pusht/videos',output_obs='./real_pusht/replay_buffer.zarr'):
        """
        初始化视频录制器和数据订阅器。

        参数:
            fps (int): 视频帧率（默认 30）。
            camera_index (int): 摄像头的索引（默认 0）。
            output_file (str): 保存视频的根目录（默认 './real_pusht_17/videos'）。
        """
        self.fps = fps
        self.camera_index = camera_index
        self.output_video_root_dir = output_videos
        self.is_recording = False  # 是否正在录制视频
        self.is_saving = False  # 是否正在保存数据
        self.cap = None  # 摄像头对象
        self.ffmpeg_process = None  # ffmpeg 进程
        self.last_key_press_time = 0  # 上次按键时间，用于防抖
        self.key_debounce_interval = 1  # 按键防抖时间间隔（秒）

        # 初始化数据存储属性
        self.timestamps = []  # 时间戳列表
        self.robot_eef_pose = []  # 机器人末端执行器位姿
        self.robot_eef_pose_vel = []  # 机器人末端执行器速度
        self.robot_joint = []  # 机器人关节角度
        self.robot_joint_vel = []  # 机器人关节速度
        self.action = []  # 机器人动作
        self.step_idx = 1  # 步数计数器
        self.episode = 0  # 数据保存的 episode 计数器

        # 订阅 ROS 话题
        self.sub_obs_data = rospy.Subscriber('/robot_obs_data', obsdata, self.obs_callback)
        
        self.output_obs_root_dir = output_obs
        # 创建 ZarrDataCreator 实例并初始化
        self.zarr_creator = ZarrDataCreator()
        self.zarr_creator.initialize_zarr_store(self.output_obs_root_dir)

        # 启动键盘监听器
        self.start_keyboard_listener()

    def obs_callback(self, obs_msg):
        """
        处理接收到的 '/robot_obs_data' 话题数据。
        """
        # 使用 header.stamp 来获取时间戳
        obs_timestamp = obs_msg.header.stamp

        # 将数据存储在对应的列表中
        self.timestamps.append(obs_timestamp.to_sec())
        self.robot_eef_pose.append(list(obs_msg.robot_eef_pose))
        self.robot_eef_pose_vel.append(list(obs_msg.robot_eef_pose_vel))
        self.robot_joint.append(list(obs_msg.robot_joint))
        self.robot_joint_vel.append(list(obs_msg.robot_joint_vel))

        # 模拟 action，使用 robot_joint_vel 作为 action 的代理
        self.action = obs_msg.robot_joint

        # 如果 is_saving 为 True，则将数据保存到 Zarr
        if self.is_saving:
            # 每 15 个 step_idx 保存一次
            if self.step_idx % 15 == 0:
                self.zarr_creator.append_data(
                    self.action,
                    self.robot_eef_pose[-1],
                    self.robot_eef_pose_vel[-1],
                    self.robot_joint[-1],
                    self.robot_joint_vel[-1],
                    0,  # stage
                    self.timestamps[-1],
                )
                self.episode += 1
                # print(f"Data appended to Zarr store at step {self.episode}")

            self.step_idx += 1



    def start_recording(self):
        """
        开始录制视频并显示实时画面。
        """
        try:
            # 初始化摄像头
            self.cap = cv2.VideoCapture(self.camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not self.cap.isOpened():
                rospy.logerr(f"错误：无法打开摄像头 {self.camera_index}。")
                return
            
            if not os.path.exists(self.output_video_root_dir):
                os.makedirs(self.output_video_root_dir)

            # 获取下一个编号的文件夹路径
            existing_folders = [f for f in os.listdir(self.output_video_root_dir)
                                if os.path.isdir(os.path.join(self.output_video_root_dir, f))]
            numbered_folders = [int(f) for f in existing_folders if f.isdigit()]
            next_folder_number = max(numbered_folders) + 1 if numbered_folders else 0
            output_folder = os.path.join(self.output_video_root_dir, str(next_folder_number))
            os.makedirs(output_folder, exist_ok=True)

            # 设置输出文件路径
            self.output_video_path = os.path.join(output_folder, '0.mp4')

            # 获取摄像头的默认宽度和高度
            # width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            # height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            width = 640
            height = 480
            print("宽度：",width)
            print("高度：",height)

            # 使用 ffmpeg 开始视频录制
            self.ffmpeg_process = ffmpeg.input('pipe:0', format='rawvideo', pix_fmt='bgr24',
                                              s=f'{width}x{height}', r=self.fps) \
                                      .output(self.output_video_path, vcodec='libx264', pix_fmt='yuv420p') \
                                      .overwrite_output() \
                                      .run_async(pipe_stdin=True)

            rospy.loginfo(f"视频保存到文件夹: {output_folder}")

            # 录制线程
            while self.is_recording:
                ret, frame = self.cap.read()

                height, width, channels = frame.shape
                # print(f"帧的实际分辨率: {width}x{height}, 通道数: {channels}")

                # 缩放至 640x480
                resized_frame = cv2.resize(frame, (640, 480))

                if not ret:
                    rospy.logerr("错误：无法捕获帧。")
                    break

                # 将帧写入 ffmpeg 的标准输入
                self.ffmpeg_process.stdin.write(resized_frame.tobytes())

        except Exception as e:
            rospy.logerr(f"启动录制时发生错误: {e}")
        finally:
            # 释放资源
            if self.cap:
                self.cap.release()
            if self.ffmpeg_process:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait()
            cv2.destroyAllWindows()

    def delete_current_recording(self):
        """
        删除当前正在录制的视频和数据。
        """
        try:

            last_episode_end = self.zarr_creator.delete_data()
            self.episode = last_episode_end

            # 获取当前录制的文件夹路径
            existing_folders = [f for f in os.listdir(self.output_video_root_dir)
                                if os.path.isdir(os.path.join(self.output_video_root_dir, f))]
            numbered_folders = [int(f) for f in existing_folders if f.isdigit()]
            if not numbered_folders:
                rospy.logwarn("没有找到录制的文件夹。")
                return

            # 获取最新的编号文件夹
            latest_folder_number = max(numbered_folders)
            latest_folder_path = os.path.join(self.output_video_root_dir, str(latest_folder_number))

            # 删除文件夹及其内容
            if os.path.exists(latest_folder_path):
                import shutil
                shutil.rmtree(latest_folder_path)
                rospy.loginfo(f"已删除文件夹: {latest_folder_path}")
            else:
                rospy.logwarn(f"文件夹不存在: {latest_folder_path}")

            # 释放资源
            if self.cap:
                self.cap.release()
            if self.ffmpeg_process:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait()
            cv2.destroyAllWindows()

            rospy.loginfo("当前录制的视频已删除。")
        except Exception as e:
            rospy.logerr(f"删除录制视频时发生错误: {e}")

    def stop_recording(self):
        """
        停止录制视频。
        """
        try:
            self.is_recording = False
            self.is_saving = False

            # 保存元数据
            self.zarr_creator.append_meta_data(self.episode)
            rospy.loginfo("视频录制已停止。")
        except Exception as e:
            rospy.logerr(f"停止录制时发生错误: {e}")

    def start_keyboard_listener(self):
        """
        启动键盘监听器。
        """
        listener = keyboard.Listener(on_press=self.on_key_press)
        listener.daemon = True  # 设置为守护线程
        listener.start()

    def on_key_press(self, key):
        """
        键盘按下事件回调函数。
        """
        try:
            current_time = time.time()
            if current_time - self.last_key_press_time < self.key_debounce_interval:
                return  # 防抖处理，忽略短时间内多次按键

            self.last_key_press_time = current_time

            if key.char == 'c':  # 按下 'C' 键开始录制和保存数据
                if not self.is_recording:
                    rospy.loginfo("开始录制视频并保存数据...")
                    self.is_recording = True
                    self.is_saving = True

                    # 启动录制线程
                    self.recording_thread = threading.Thread(target=self.start_recording)
                    self.recording_thread.daemon = True
                    self.recording_thread.start()

                    # self.start_recording()

            elif key.char == 't':  # 按下 'T' 键停止录制和保存数据
                if self.is_recording:
                    rospy.loginfo("停止录制视频并保存数据...")
                    self.stop_recording()

            elif key.char == 'd':  # 按下 'D' 键删除正在录制的视频
                if self.is_recording:
                    rospy.loginfo("删除正在录制的视频...")

                    self.delete_current_recording()
        except AttributeError:
            pass  # 忽略非字符键

def main():
    # 初始化 ROS 节点
    rospy.init_node('data_subscriber_node', anonymous=True)

    # 列出所有可用的摄像头
    def list_cameras(max_tests=5):
        for i in range(max_tests):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"摄像头 {i} 可用。")
                cap.release()
            else:
                print(f"摄像头 {i} 不可用。")

    list_cameras()
    # 创建 DataSubscriber 实例
    fps = 30
    camera_index = 0
    output_videos = '.DP/data/real_pusht/videos'
    output_obs = '..DP/data/real_pusht/replay_buffer.zarr'
    data_subscriber = DataSubscriber(fps=fps, camera_index=camera_index, output_videos=output_videos, output_obs=output_obs)

    # 保持主线程运行
    rospy.spin()

# 程序入口
if __name__ == "__main__":
    main()