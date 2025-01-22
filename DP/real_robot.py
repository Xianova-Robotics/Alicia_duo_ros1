#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import math
import queue
import threading
import time
from collections import deque
import cv2 as cv
import dill
import hydra
import numpy as np
import torch
import zmq
import cv2

# import sys
# # 打印当前的 Python 路径
# print("Current Python Path (sys.path):")
# sys.path.append('/home/xuanya/eto/diffusion_policy')
# for path in sys.path:
#     print(path)

import sys
import os

# 添加 diffusion_policy 的路径
diffusion_policy_path = os.path.abspath("/home/xuanya/eto/diffusion_policy")
if diffusion_policy_path not in sys.path:
    sys.path.append(diffusion_policy_path)

from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.model.common.rotation_transformer import RotationTransformer


import rospy
import numpy as np
from diffusion_policy_obs.srv import ProcessData, ProcessDataRequest
from diffusion_policy_obs.msg import ObsData
from frame_msgs.msg import set_servo_as

POLICY_CONTROL_PERIOD = 0.1  # 100 ms (10 Hz)
LATENCY_BUDGET = 0.2  # 200 ms including policy inference and communication
LATENCY_STEPS = math.ceil(LATENCY_BUDGET / POLICY_CONTROL_PERIOD)  # Up to 3 is okay, 4 is too high


class StubDiffusionPolicy:
    def reset(self):
        pass

    def step(self, obs_sequence):
        obs = obs_sequence[-1]
        act_sequence = [f'{obs + i} (inference {obs})' for i in range(8)]
        time.sleep(0.115)  # 115 ms
        return act_sequence


class DiffusionPolicy:
    def __init__(self, ckpt_path):
        # Load checkpoint
        with open(ckpt_path, 'rb') as f:
            payload = torch.load(f, pickle_module=dill)
        cfg = payload['cfg']
        cls = hydra.utils.get_class(cfg._target_)
        workspace = cls(cfg)
        workspace.load_payload(payload)

        # Load policy
        policy = workspace.model
        if cfg.training.use_ema:
            policy = workspace.ema_model
        device = torch.device('cuda')
        policy.eval().to(device)

        # Store attributes
        self.policy = policy
        self.device = device
        self.obs_shape_meta = cfg.shape_meta['obs']
        self.rotation_transformer = RotationTransformer(from_rep='rotation_6d', to_rep='quaternion')
        self.warmed_up = False

    def reset(self):
        self.policy.reset()  # 重置策略

    def step(self, obs_sequence):
        # 将观察序列转换为字典格式
        obs_dict = self._convert_obs(obs_sequence)
        with torch.no_grad():  # 禁用梯度计算
            if not self.warmed_up:  # 如果策略还没有热身
                print('Warming up policy...')  # 打印热身信息
                self.policy.predict_action(obs_dict)  # 预测动作进行热身
                self.warmed_up = True  # 设置已热身
            result = self.policy.predict_action(obs_dict)  # 获取策略输出
            action = result['action'][0].detach().to('cpu').numpy()  # 获取动作并转换为 numpy 数组
        # 将动作转换为动作序列
        # act_sequence = self._convert_action(action)
        return action  # 返回动作序列
    
    def _convert_obs(self, obs_sequence):
        obs_dict_np = {}  # 创建一个空字典来存储观察数据
        for key, value in self.obs_shape_meta.items():  # 遍历观察数据元数据
            if value.get('type') == 'rgb':  # 如果数据类型是 rgb 图像
                images = np.stack([obs[key] for obs in obs_sequence], axis=0)  # 堆叠图像
                print(images.shape)
                assert images.dtype == np.uint8  # 确保数据类型为 uint8
                images = images.astype(np.float32) / 255.0  # 转换为浮点型并归一化
                # images = np.transpose(images, (0, 3, 1, 2))  # 调整图像维度顺序
                print(f"Before transpose - images.shape: {images.shape}, expected shape: {tuple(value['shape'])}")
                assert images.shape[1:] == tuple(value['shape'])  # 确保图像形状符合预期
                obs_dict_np[key] = images  # 将处理后的图像数据存储到字典中
            else:
                obs_dict_np[key] = np.stack([obs[key] for obs in obs_sequence], axis=0).astype(np.float32)  # 处理其他类型的数据
            # print(obs_dict_np)
        obs_dict = dict_apply(obs_dict_np, lambda x: torch.from_numpy(x).unsqueeze(0).to(self.device))
        return obs_dict  # 返回转换后的观察数据字典

    def _convert_action(self, action):
        act_sequence = []  # 创建空列表来存储动作序列
        for act in action:  # 遍历每个动作
            action_dict = {
                'base_pose': act[:3],  # 基本姿势
                'arm_pos': act[3:6],  # 手臂位置
                'arm_quat': self.rotation_transformer.forward(act[6:12])[[1, 2, 3, 0]],  # (w, x, y, z) -> (x, y, z, w)
                'gripper_pos': act[12:13],  # 夹爪位置
            }
            act_sequence.append(action_dict)  # 将动作字典添加到动作序列中
        return act_sequence  # 返回动作序列



class PolicyWrapper:
    def __init__(self, policy, n_obs_steps=2, n_action_steps=8):
        # 初始化PolicyWrapper类
        # policy: 策略对象，用于生成动作
        # n_obs_steps: 观察历史的最大长度，默认为2
        # n_action_steps: 动作序列的长度，默认为8

        self.n_obs_steps = n_obs_steps  # 设置观察历史的最大长度
        self.n_action_steps = n_action_steps  # 设置动作序列的长度
        self.obs_queue = queue.Queue()  # 创建一个队列用于存储观察数据
        self.act_queue = queue.Queue()  # 创建一个队列用于存储动作数据
        self.index = 0

        self.last_publish_time = time.time()  # 记录上一次发布消息的时间

        # 启动推理循环
        # 创建一个新的线程来运行inference_loop方法
        # policy作为参数传递给inference_loop
        # daemon=True表示该线程为守护线程，主线程结束时它会自动结束
        threading.Thread(target=self.inference_loop, args=(policy,), daemon=True).start()

    def reset(self):
        # 重置方法，用于重置策略和队列
        # 向obs_queue队列中放入一个'reset'信号
        self.obs_queue.put('reset')

    # def step(self, obs):
    #     # 单步执行方法，用于处理当前的观察数据并返回动作
    #     # obs: 当前的观察数据

    #     self.obs_queue.put(obs)  # 将当前的观察数据放入obs_queue队列
    #     action = None if self.act_queue.empty() else self.act_queue.get()  # 如果act_queue不为空，则取出一个动作，否则返回None
    #     if action is None:
    #         # 如果动作队列为空，打印警告信息
    #         print('Warning: Unexpected idle action queue. Is the latency budget set too low?')
    #     return action  # 返回动作

    def step(self, obs):
        # 单步执行方法，用于处理当前的观察数据并返回动作
        # obs: 当前的观察数据

        self.obs_queue.put(obs)  # 将当前的观察数据放入obs_queue队列

        # # 打印队列中的数据
        # queue_data = list(self.obs_queue.queue)
        # print(f"队列中的数据: {queue_data}")

        if self.obs_queue.qsize() == 5:
            self.obs_queue.queue.clear()
        # 打印队列长度
        print(f"队列长度: {self.obs_queue.qsize()}")
        
        # 如果act_queue不为空，取出一个动作并清空队列
        if not self.act_queue.empty():
            action = self.act_queue.get()  # 取出一个动作
            self.act_queue.queue.clear()  # 清空队列
        else:
            action = None  # 如果act_queue为空，返回None
            print('Warning: Unexpected idle action queue. Is the latency budget set too low?')

        # current_time = time.time()
        # time_interval = current_time - self.last_publish_time
        # publish_rate = 1 / time_interval if time_interval > 0 else 0
        
        # print("循环时间间隔:", time_interval)
        # # 打印发布速率
        # print(f"消息接收速率: {publish_rate:.2f} Hz")

        # # 更新上一次发布消息的时间
        # self.last_publish_time = current_time
        return action  # 返回动作

    def inference_loop(self, policy):
        # 推理循环方法，用于持续处理观察数据并生成动作
        # policy: 策略对象，用于生成动作

        obs_history = deque(maxlen=self.n_obs_steps)  # 创建一个双端队列用于存储观察历史，最大长度为n_obs_steps
        start_of_episode = True  # 标记是否是新的一轮（episode）的开始
        while True:
            # 检查是否有新的观察数据
            if not self.obs_queue.empty():
                obs = self.obs_queue.get()  # 从obs_queue队列中取出一个观察数据

                # 如果观察数据是'reset'信号，则重置策略
                if obs == 'reset':
                    policy.reset()  # 重置策略
                    obs_history.clear()  # 清空观察历史
                    start_of_episode = True  # 标记为新的一轮的开始
                    while not self.act_queue.empty():
                        self.act_queue.get()  # 清空动作队列
                    continue  # 跳过本次循环的剩余部分

                # 将观察数据添加到观察历史中
                obs_history.append(obs)

            # 如果动作队列的大小小于LATENCY_STEPS，并且观察历史的长度等于n_obs_steps
            if self.act_queue.qsize() < LATENCY_STEPS and len(obs_history) == self.n_obs_steps:
                obs_sequence = list(obs_history)  # 将观察历史转换为列表
                # print("robot_joint 数据:")
                for obs in obs_sequence:
                    print("推理输入",obs['robot_joint'])   
                act_sequence = policy.step(obs_sequence)  # 使用策略生成动作序列
                self.index += 1
                print("生成动作序列",act_sequence,self.index)
                if not self.act_queue.empty():
                    # 如果动作队列不为空，打印警告信息
                    print('Warning: Unexpected action queue backlog. Is the latency budget set too high?')
                if start_of_episode:
                    # 如果是新的一轮的开始，截取动作序列的前n_action_steps - LATENCY_STEPS个动作
                    act_sequence = act_sequence[:self.n_action_steps - LATENCY_STEPS]
                    # act_sequence = act_sequence[:1]
                    start_of_episode = False  # 标记为不是新的一轮的开始
                    print("新的一轮的开始")
                else:
                    # 否则，截取动作序列的LATENCY_STEPS到n_action_steps之间的动作
                    act_sequence = act_sequence[LATENCY_STEPS:self.n_action_steps]
                    # act_sequence = act_sequence[:1]
                for action in act_sequence:
                    self.act_queue.put(action)  # 将动作序列中的每个动作放入动作队列

            time.sleep(1) #每次循环后休眠0.001秒，避免CPU占用过高


class PolicyServer:
    def __init__(self, policy):
        self.policy = policy
        self.req = None  # 用于存储从 ROS 服务接收到的数据
        self.latest_action = None  # 存储最新生成的动作
        self.lock = threading.Lock()  # 线程锁，用于保护 self.req


        # 初始化 ROS 发布器
        self.action_pub = rospy.Publisher('/main_control_servo', set_servo_as, queue_size=10)

    def step(self, obs):
        # 使用 policy 对象根据观察值生成动作
        action = self.policy.step(obs)
        return action

    def run(self):
        while not rospy.is_shutdown():  # 检查 ROS 是否关闭
            with self.lock:
                # 如果没有接收到有效的请求数据，继续等待
                if self.req is None:
                    time.sleep(0.1)  # 减少 sleep 时间以提高响应速度
                    continue
                # 获取观察数据
                obs = self.req['obs']
                print(f"Generated image shape: {self.req['obs']['camera_0'].shape}")
                print(f"Generated robot_joint: {self.req['obs']['robot_joint']}")

                camera_0_image = self.req['obs']['camera_0']               
                angles_text = f"Joint Angles: {self.req['obs']['robot_joint']}"
                camera_0_image_display = camera_0_image.transpose(1, 2, 0)

                # 检查图像数据类型并转换为 uint8
                if camera_0_image_display.dtype != np.uint8:
                    camera_0_image_display = camera_0_image_display.astype(np.uint8)

                # 检查图像通道顺序并转换为 BGR
                if camera_0_image_display.shape[2] == 3:  # 如果是 RGB 图像
                    camera_0_image_display = cv2.cvtColor(camera_0_image_display, cv2.COLOR_RGB2BGR)

                # 检查图像是否为空
                if camera_0_image_display is None:
                    print("Error: camera_0_image_display is None.")
                elif camera_0_image_display.size == 0:
                    print("Error: camera_0_image_display is empty.")


                print(camera_0_image_display.dtype)
                # 在图像上绘制文本
                font = cv2.FONT_HERSHEY_SIMPLEX  # 字体
                position = (10, 30)  # 文本位置 (x, y)
                font_scale = 0.8  # 字体大小
                font_color = (0, 255, 0)  # 字体颜色 (BGR 格式，这里是绿色)
                thickness = 2  # 字体粗细

                cv2.putText(camera_0_image_display, angles_text, position, font, font_scale, font_color, thickness)
                # 显示图像
                cv2.imshow('Camera 1', camera_0_image_display)
                cv2.waitKey(1)  # 等待 1 毫秒，确保窗口更新                

            # 使用 policy 生成动作
            action = self.step(obs)
            
            latest_action = np.array(action)
            # print("Generated action:", latest_action)
            if action is not None:
                # 将动作发布到 ROS 话题
                self.publish_action(latest_action)

            


            # 模拟推理延迟
            time.sleep(0.1)  # 减少 sleep 时间以提高响应速度

    def publish_action(self, latest_action):
        """
        将动作发布到 ROS 话题 '/main_control_servo'
        """
        # 创建一个 set_servo_as 消息
        action_msg = set_servo_as()

        # 将 latest_action 转换为 Python 列表
        latest_action = latest_action.tolist()
        # 取出第二个元素并取反
        negated_second_element = -latest_action[1]
        # 使用列表拼接
        action = latest_action[:2] + [negated_second_element] + latest_action[2:6]

        action_msg.servo_target_angle.data = action
        action_msg.servo_target_cycle.data = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
        action_msg.gripper_angle.data = float(latest_action[-1])

        print("控制action:", action)
        print("控制gripper_angle:", float(latest_action[-1]))
                
        # 计算发布速率
        # current_time = time.time()
        # time_interval = current_time - self.last_publish_time
        # publish_rate = 1 / time_interval if time_interval > 0 else 0
        
        # print("发布时间间隔:", time_interval)
        # # 打印发布速率
        # print(f"消息发布速率: {publish_rate:.2f} Hz")

        # # 更新上一次发布消息的时间
        # self.last_publish_time = current_time

        

    def update_req(self, req):
        """
        更新 self.req 数据
        """
        with self.lock:
            self.req = req



def call_process_data_service(server):
    """
    调用 ROS 服务并更新 server.req
    """
    # # 等待服务可用
    # rospy.loginfo("Waiting for 'process_data' service...")
    # rospy.wait_for_service('process_data')

    try:
        # 创建服务代理
        process_data = rospy.ServiceProxy('process_data', ProcessData)

        # 创建请求对象
        request = ProcessDataRequest()

        # 定期调用服务
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # 调用服务
            response = process_data(request)

            # 检查服务调用是否成功
            if response.success:

                # 获取服务返回的数据
                obs_data = response.data
                # print("Received image shape:", obs_data.camera_0)
                # 将字节数据解码为图像
                # 假设 obs_data.camera_0 是展平的 RGB 图像数据
                camera_0_flat = np.frombuffer(obs_data.camera_0, dtype=np.uint8)

                # 将展平的数据恢复为 (3, 240, 320) 的形状
                camera_0 = camera_0_flat.reshape(3, 240, 320)

                # 将 ObsData 消息转换为字典格式
                req = {
                    'obs': {
                        'camera_0': camera_0,  # 图像数据已经是列表格式
                        'robot_joint': np.array(obs_data.robot_joint)  # 将列表转换为 NumPy 数组
                    }
                }
                 

                # 更新 server.req
                server.update_req(req)

                # # 打印拼凑后的字典
                # print("Reconstructed dictionary (req):", req)

            else:
                rospy.logwarn("Service call failed: No data available.")

            # 按照指定频率休眠
            rate.sleep()

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main(ckpt_path):
    # 初始化 ROS 节点
    rospy.init_node('process_data_client', anonymous=True)

    policy = PolicyWrapper(DiffusionPolicy(ckpt_path))
    server = PolicyServer(policy)

    # 启动 ROS 服务调用线程
    ros_thread = threading.Thread(target=call_process_data_service, args=(server,), daemon=True)
    ros_thread.start()

    # 启动政策服务器
    server.run()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt-path', default='data/outputs/zhuaqu/latest.ckpt')
    args = parser.parse_args()
    main(args.ckpt_path)