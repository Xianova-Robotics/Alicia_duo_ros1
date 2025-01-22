"""
Usage:
(robodiff)$ python demo_real_robot.py -o <demo_save_dir> --robot_ip <ip_of_ur5>

Robot movement:
Move your SpaceMouse to move the robot EEF (locked in xy plane).
Press SpaceMouse right button to unlock z axis.
Press SpaceMouse left button to enable rotation axes.

Recording control:
Click the opencv window (make sure it's in focus).
Press "C" to start recording.
Press "S" to stop recording.
Press "Q" to exit program.
Press "Backspace" to delete the previously recorded episode.
"""

# %%
import time
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import scipy.spatial.transform as st
from diffusion_policy.real_world.real_env import RealEnv
from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse
from diffusion_policy.common.precise_sleep import precise_wait
from diffusion_policy.real_world.keystroke_counter import (
    KeystrokeCounter, Key, KeyCode
)

@click.command()
@click.option('--output', '-o', required=True, help="Directory to save demonstration dataset.")
@click.option('--robot_ip', '-ri', required=True, help="UR5's IP address e.g. 192.168.0.204")
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
@click.option('--init_joints', '-j', is_flag=True, default=False, help="Whether to initialize robot joint configuration in the beginning.")
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")
@click.option('--command_latency', '-cl', default=0.01, type=float, help="Latency between receiving SapceMouse command to executing on Robot in Sec.")
def main(output, robot_ip, vis_camera_idx, init_joints, frequency, command_latency):
    # 计算每个循环的时间间隔
    dt = 1/frequency
    # 使用共享内存管理器
    with SharedMemoryManager() as shm_manager:
        # 使用按键计数器、Spacemouse 和 RealEnv 环境
        with KeystrokeCounter() as key_counter, \
            Spacemouse(shm_manager=shm_manager) as sm, \
            RealEnv(
                output_dir=output, 
                robot_ip=robot_ip, 
                # 设置观察图像的录制分辨率
                obs_image_resolution=(1280,720),
                frequency=frequency,
                init_joints=init_joints,
                enable_multi_cam_vis=True,
                record_raw_video=True,
                # 设置每个相机视图的线程数以进行视频录制（H.264）
                thread_per_video=3,
                # 设置视频录制质量，数值越低质量越好（但速度越慢）
                video_crf=21,
                shm_manager=shm_manager
            ) as env:
            # 设置 OpenCV 的线程数为 1
            cv2.setNumThreads(1)

            # 设置 RealSense 相机的曝光
            env.realsense.set_exposure(exposure=120, gain=0)
            # 设置 RealSense 相机的白平衡
            env.realsense.set_white_balance(white_balance=5900)

            # 等待 1 秒
            time.sleep(1.0)
            # 打印准备就绪信息
            print('Ready!')
            # 获取机器人当前状态
            state = env.get_robot_state()
            # 获取目标 TCP 位姿
            target_pose = state['TargetTCPPose']
            # 记录循环开始时间
            t_start = time.monotonic()
            # 初始化循环索引
            iter_idx = 0
            # 初始化停止标志
            stop = False
            # 初始化录制标志
            is_recording = False
            # 主循环
            while not stop:
                # 计算当前循环的结束时间
                t_cycle_end = t_start + (iter_idx + 1) * dt
                # 计算采样时间
                t_sample = t_cycle_end - command_latency
                # 计算命令执行的目标时间
                t_command_target = t_cycle_end + dt

                # 获取当前观察数据
                obs = env.get_obs()

                # 处理按键事件
                press_events = key_counter.get_press_events()
                for key_stroke in press_events:
                    if key_stroke == KeyCode(char='q'):
                        # 按下 'q' 键退出程序
                        stop = True
                    elif key_stroke == KeyCode(char='c'):
                        # 按下 'c' 键开始录制
                        env.start_episode(t_start + (iter_idx + 2) * dt - time.monotonic() + time.time())
                        key_counter.clear()
                        is_recording = True
                        print('Recording!')
                    elif key_stroke == KeyCode(char='s'):
                        # 按下 's' 键停止录制
                        env.end_episode()
                        key_counter.clear()
                        is_recording = False
                        print('Stopped.')
                    elif key_stroke == Key.backspace:
                        # 按下退格键删除最近录制的片段
                        if click.confirm('Are you sure to drop an episode?'):
                            env.drop_episode()
                            key_counter.clear()
                            is_recording = False
                        # 删除操作
                # 获取当前阶段
                stage = key_counter[Key.space]

                # 可视化当前图像
                vis_img = obs[f'camera_{vis_camera_idx}'][-1,:,:,::-1].copy()
                # 获取当前片段 ID
                episode_id = env.replay_buffer.n_episodes
                # 设置显示的文本信息
                text = f'Episode: {episode_id}, Stage: {stage}'
                if is_recording:
                    text += ', Recording!'
                # 在图像上绘制文本
                cv2.putText(
                    vis_img,
                    text,
                    (10,30),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1,
                    thickness=2,
                    color=(255,255,255)
                )

                # 显示图像
                cv2.imshow('default', vis_img)
                # 处理 OpenCV 的键盘事件
                cv2.pollKey()

                # 精确等待到采样时间
                precise_wait(t_sample)
                # 获取 Spacemouse 的遥操作命令
                sm_state = sm.get_motion_state_transformed()
                # 计算位置变化
                dpos = sm_state[:3] * (env.max_pos_speed / frequency)
                # 计算旋转变化
                drot_xyz = sm_state[3:] * (env.max_rot_speed / frequency)
                
                if not sm.is_button_pressed(0):
                    # 如果未按下按钮 0，进入平移模式，旋转变化置零
                    drot_xyz[:] = 0
                else:
                    # 否则，位置变化置零
                    dpos[:] = 0
                if not sm.is_button_pressed(1):
                    # 如果未按下按钮 1，进入 2D 平移模式，Z 轴位置变化置零
                    dpos[2] = 0    

                # 计算旋转变化
                drot = st.Rotation.from_euler('xyz', drot_xyz)
                # 更新目标位置
                target_pose[:3] += dpos
                # 更新目标旋转
                target_pose[3:] = (drot * st.Rotation.from_rotvec(
                    target_pose[3:])).as_rotvec()

                # 执行遥操作命令
                env.exec_actions(
                    actions=[target_pose], 
                    timestamps=[t_command_target-time.monotonic()+time.time()],
                    stages=[stage])
                # 精确等待到循环结束时间
                precise_wait(t_cycle_end)
                # 增加循环索引
                iter_idx += 1

# %%
if __name__ == '__main__':
    # 如果作为主程序运行，调用 main 函数
    main()