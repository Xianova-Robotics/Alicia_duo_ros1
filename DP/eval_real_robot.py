"""
Usage:
(robodiff)$ python eval_real_robot.py -i <ckpt_path> -o <save_dir> --robot_ip <ip_of_ur5>

================ Human in control ==============
Robot movement:
Move your SpaceMouse to move the robot EEF (locked in xy plane).
Press SpaceMouse right button to unlock z axis.
Press SpaceMouse left button to enable rotation axes.

Recording control:
Click the opencv window (make sure it's in focus).
Press "C" to start evaluation (hand control over to policy).
Press "Q" to exit program.

================ Policy in control ==============
Make sure you can hit the robot hardware emergency-stop button quickly! 

Recording control:
Press "S" to stop evaluation and gain control back.
"""

# %%
import time
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import torch
import dill
import hydra
import pathlib
import skvideo.io
from omegaconf import OmegaConf
import scipy.spatial.transform as st
from diffusion_policy.real_world.real_env import RealEnv
from diffusion_policy.real_world.spacemouse_shared_memory import Spacemouse
from diffusion_policy.common.precise_sleep import precise_wait
from diffusion_policy.real_world.real_inference_util import (
    get_real_obs_resolution, 
    get_real_obs_dict)
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.cv2_util import get_image_transform


OmegaConf.register_new_resolver("eval", eval, replace=True)

@click.command()  # 使用click库定义一个命令行接口
@click.option('--input', '-i', required=True, help='Path to checkpoint')  # 添加一个必需的输入选项，用于指定检查点文件的路径
@click.option('--output', '-o', required=True, help='Directory to save recording')  # 添加一个必需的输出选项，用于指定保存录制的目录
@click.option('--robot_ip', '-ri', required=True, help="UR5's IP address e.g. 192.168.0.204")  # 添加一个必需的选项，用于指定UR5机器人的IP地址
@click.option('--match_dataset', '-m', default=None, help='Dataset used to overlay and adjust initial condition')  # 添加一个可选的选项，用于指定用于覆盖和调整初始条件的数据集
@click.option('--match_episode', '-me', default=None, type=int, help='Match specific episode from the match dataset')  # 添加一个可选的选项，用于指定从匹配数据集中匹配的特定集数
@click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")  # 添加一个可选的选项，用于指定要可视化的RealSense摄像头的索引
@click.option('--init_joints', '-j', is_flag=True, default=False, help="Whether to initialize robot joint configuration in the beginning.")  # 添加一个标志选项，用于指定是否在开始时初始化机器人关节配置
@click.option('--steps_per_inference', '-si', default=6, type=int, help="Action horizon for inference.")  # 添加一个可选的选项，用于指定推理时的动作范围
@click.option('--max_duration', '-md', default=60, help='Max duration for each epoch in seconds.')  # 添加一个可选的选项，用于指定每个周期的最大持续时间（以秒为单位）
@click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")  # 添加一个可选的选项，用于指定控制频率（以赫兹为单位）
@click.option('--command_latency', '-cl', default=0.01, type=float, help="Latency between receiving SapceMouse command to executing on Robot in Sec.")  # 添加一个可选的选项，用于指定从接收到SpaceMouse命令到在机器人上执行之间的延迟（以秒为单位）
def main(input, output, robot_ip, match_dataset, match_episode,
    vis_camera_idx, init_joints, 
    steps_per_inference, max_duration,
    frequency, command_latency):
    # 加载比赛数据集
    match_camera_idx = 0  # 设置默认的相机索引为0
    episode_first_frame_map = dict()  # 创建一个字典来存储每个episode的第一帧图像
    if match_dataset is not None:  # 如果提供了比赛数据集
        match_dir = pathlib.Path(match_dataset)  # 将数据集路径转换为Path对象
        match_video_dir = match_dir.joinpath('videos')  # 获取视频文件夹路径
        for vid_dir in match_video_dir.glob("*/"):  # 遍历视频文件夹中的每个子文件夹
            episode_idx = int(vid_dir.stem)  # 获取episode的索引
            match_video_path = vid_dir.joinpath(f'{match_camera_idx}.mp4')  # 构建视频文件路径
            if match_video_path.exists():  # 如果视频文件存在
                frames = skvideo.io.vread(
                    str(match_video_path), num_frames=1)  # 读取视频的第一帧
                episode_first_frame_map[episode_idx] = frames[0]  # 将第一帧存储到字典中
    print(f"Loaded initial frame for {len(episode_first_frame_map)} episodes")  # 打印加载的episode数量
    
    # 加载模型检查点
    ckpt_path = input  # 检查点路径
    payload = torch.load(open(ckpt_path, 'rb'), pickle_module=dill)  # 加载检查点文件
    cfg = payload['cfg']  # 获取配置文件
    cls = hydra.utils.get_class(cfg._target_)  # 根据配置文件中的目标类名获取类
    workspace = cls(cfg)  # 实例化工作空间
    workspace: BaseWorkspace  # 类型注解，表示workspace是BaseWorkspace类型
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)  # 加载模型参数

    # 根据方法类型进行特定设置
    action_offset = 0  # 动作偏移量
    delta_action = False  # 是否使用动作差值
    if 'diffusion' in cfg.name:  # 如果是扩散模型
        # 扩散模型
        policy: BaseImagePolicy  # 类型注解，表示policy是BaseImagePolicy类型
        policy = workspace.model  # 获取模型
        if cfg.training.use_ema:  # 如果使用EMA模型
            policy = workspace.ema_model  # 使用EMA模型

        device = torch.device('cuda')  # 设置设备为CUDA
        policy.eval().to(device)  # 将模型设置为评估模式并移动到CUDA设备

        # 设置推理参数
        policy.num_inference_steps = 16  # DDIM推理迭代次数
        policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1  # 计算动作步数

    elif 'robomimic' in cfg.name:  # 如果是BCRNN模型
        # BCRNN模型
        policy: BaseImagePolicy  # 类型注解，表示policy是BaseImagePolicy类型
        policy = workspace.model  # 获取模型

        device = torch.device('cuda')  # 设置设备为CUDA
        policy.eval().to(device)  # 将模型设置为评估模式并移动到CUDA设备

        # BCRNN模型的动作步数始终为1
        steps_per_inference = 1  # 设置每次推理的步数为1
        action_offset = cfg.n_latency_steps  # 设置动作偏移量
        delta_action = cfg.task.dataset.get('delta_action', False)  # 获取是否使用动作差值

    elif 'ibc' in cfg.name:  # 如果是IBC模型
        policy: BaseImagePolicy  # 类型注解，表示policy是BaseImagePolicy类型
        policy = workspace.model  # 获取模型
        policy.pred_n_iter = 5  # 设置预测迭代次数
        policy.pred_n_samples = 4096  # 设置预测样本数

        device = torch.device('cuda')  # 设置设备为CUDA
        policy.eval().to(device)  # 将模型设置为评估模式并移动到CUDA设备
        steps_per_inference = 1  # 设置每次推理的步数为1
        action_offset = 1  # 设置动作偏移量
        delta_action = cfg.task.dataset.get('delta_action', False)  # 获取是否使用动作差值
    else:
        raise RuntimeError("Unsupported policy type: ", cfg.name)  # 抛出异常，表示不支持该策略类型

    # 设置实验参数
    dt = 1/frequency  # 计算时间步长

    obs_res = get_real_obs_resolution(cfg.task.shape_meta)  # 获取观测分辨率
    n_obs_steps = cfg.n_obs_steps  # 获取观测步数
    print("n_obs_steps: ", n_obs_steps)  # 打印观测步数
    print("steps_per_inference:", steps_per_inference)  # 打印每次推理的步数
    print("action_offset:", action_offset)  # 打印动作偏移量

    with SharedMemoryManager() as shm_manager:  # 使用共享内存管理器
        with Spacemouse(shm_manager=shm_manager) as sm, RealEnv(  # 使用Spacemouse和RealEnv
            output_dir=output,  # 设置输出目录
            robot_ip=robot_ip,  # 设置机器人IP
            frequency=frequency,  # 设置频率
            n_obs_steps=n_obs_steps,  # 设置观测步数
            obs_image_resolution=obs_res,  # 设置观测图像分辨率
            obs_float32=True,  # 设置观测数据类型为float32
            init_joints=init_joints,  # 设置初始关节位置
            enable_multi_cam_vis=True,  # 启用多相机可视化
            record_raw_video=True,  # 启用原始视频录制
            # 每个相机视图的视频录制线程数（H.264）
            thread_per_video=3,
            # 视频录制质量，数值越低质量越好（但速度越慢）
            video_crf=21,
            shm_manager=shm_manager) as env:  # 使用共享内存管理器
            cv2.setNumThreads(1)  # 设置OpenCV线程数为1

            # 应与演示相同
            # 设置Realsense相机的曝光
            env.realsense.set_exposure(exposure=120, gain=0)
            # 设置Realsense相机的白平衡
            env.realsense.set_white_balance(white_balance=5900)

            print("Waiting for realsense")  # 打印等待Realsense相机的信息
            time.sleep(1.0)  # 等待1秒

            print("Warming up policy inference")  # 打印预热策略推理的信息
            obs = env.get_obs()  # 获取当前观测
            with torch.no_grad():  # 禁用梯度计算
                policy.reset()  # 重置策略
                obs_dict_np = get_real_obs_dict(
                    env_obs=obs, shape_meta=cfg.task.shape_meta)  # 获取观测字典
                obs_dict = dict_apply(obs_dict_np, 
                    lambda x: torch.from_numpy(x).unsqueeze(0).to(device))  # 将观测字典转换为张量
                result = policy.predict_action(obs_dict)  # 预测动作
                action = result['action'][0].detach().to('cpu').numpy()  # 获取动作并转换为numpy数组
                assert action.shape[-1] == 2  # 确保动作的最后一个维度为2
                del result  # 删除结果以释放内存

        print('Ready!')  # 打印 "Ready!"，表示程序已准备好
        while True:  # 进入一个无限循环，表示程序将持续运行
            # ========= human control loop ==========
            print("Human in control!")  # 打印 "Human in control!"，表示当前由人类控制
            state = env.get_robot_state()  # 获取机器人的当前状态
            target_pose = state['TargetTCPPose']  # 从状态中提取目标TCP（工具中心点）姿态
            t_start = time.monotonic()  # 记录当前时间作为循环的起始时间
            iter_idx = 0  # 初始化迭代索引为0
            while True:  # 进入另一个无限循环，表示人类控制的具体逻辑
                # calculate timing
                t_cycle_end = t_start + (iter_idx + 1) * dt  # 计算当前循环周期的结束时间
                t_sample = t_cycle_end - command_latency  # 计算采样时间，考虑命令延迟
                t_command_target = t_cycle_end + dt  # 计算命令的目标时间

                # pump obs
                obs = env.get_obs()  # 获取当前环境的观测值

                # visualize
                episode_id = env.replay_buffer.n_episodes  # 获取当前回放缓冲区中的总回合数
                vis_img = obs[f'camera_{vis_camera_idx}'][-1]  # 获取指定摄像头的最后一帧图像
                match_episode_id = episode_id  # 初始化匹配回合ID为当前回合ID
                if match_episode is not None:  # 如果指定了匹配回合
                    match_episode_id = match_episode  # 使用指定的匹配回合ID
                if match_episode_id in episode_first_frame_map:  # 如果匹配回合ID在首帧映射中存在
                    match_img = episode_first_frame_map[match_episode_id]  # 获取匹配回合的首帧图像
                    ih, iw, _ = match_img.shape  # 获取匹配图像的高度、宽度和通道数
                    oh, ow, _ = vis_img.shape  # 获取当前可视化图像的高度、宽度和通道数
                    tf = get_image_transform(
                        input_res=(iw, ih), 
                        output_res=(ow, oh), 
                        bgr_to_rgb=False)  # 获取图像变换函数，用于将匹配图像转换为与当前图像相同的分辨率
                    match_img = tf(match_img).astype(np.float32) / 255  # 对匹配图像进行变换并归一化
                    vis_img = np.minimum(vis_img, match_img)  # 将当前图像与匹配图像进行逐像素最小值操作

                text = f'Episode: {episode_id}'  # 生成显示文本，包含当前回合ID
                cv2.putText(
                    vis_img,
                    text,
                    (10,20),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.5,
                    thickness=1,
                    color=(255,255,255)
                )  # 在可视化图像上绘制文本
                cv2.imshow('default', vis_img[...,::-1])  # 显示可视化图像
                key_stroke = cv2.pollKey()  # 获取键盘输入
                if key_stroke == ord('q'):  # 如果按下 'q' 键
                    # Exit program
                    env.end_episode()  # 结束当前回合
                    exit(0)  # 退出程序
                elif key_stroke == ord('c'):  # 如果按下 'c' 键
                    # Exit human control loop
                    # hand control over to the policy
                    break  # 退出人类控制循环，将控制权交给策略

                precise_wait(t_sample)  # 精确等待到采样时间
                # get teleop command
                sm_state = sm.get_motion_state_transformed()  # 获取经过变换的运动状态
                # print(sm_state)
                dpos = sm_state[:3] * (env.max_pos_speed / frequency)  # 计算位置变化量
                drot_xyz = sm_state[3:] * (env.max_rot_speed / frequency)  # 计算旋转变化量

                if not sm.is_button_pressed(0):  # 如果按钮0未被按下
                    # translation mode
                    drot_xyz[:] = 0  # 将旋转变化量置零，进入平移模式
                else:
                    dpos[:] = 0  # 将位置变化量置零，进入旋转模式
                if not sm.is_button_pressed(1):  # 如果按钮1未被按下
                    # 2D translation mode
                    dpos[2] = 0  # 将Z轴位置变化量置零，进入2D平移模式

                drot = st.Rotation.from_euler('xyz', drot_xyz)  # 根据旋转变化量创建旋转对象
                target_pose[:3] += dpos  # 更新目标姿态的位置
                target_pose[3:] = (drot * st.Rotation.from_rotvec(
                    target_pose[3:])).as_rotvec()  # 更新目标姿态的旋转
                # clip target pose
                target_pose[:2] = np.clip(target_pose[:2], [0.25, -0.45], [0.77, 0.40])  # 对目标姿态的位置进行裁剪，确保在合理范围内

                # execute teleop command
                env.exec_actions(
                    actions=[target_pose], 
                    timestamps=[t_command_target-time.monotonic()+time.time()])  # 执行遥操作命令
                precise_wait(t_cycle_end)  # 精确等待到当前循环周期结束
                iter_idx += 1  # 迭代索引加1，进入下一个循环周期
                
# ========== 策略控制循环 ==============
                try:
                    # 开始一个 episode
                    policy.reset()  # 重置策略
                    start_delay = 1.0  # 启动延迟时间
                    eval_t_start = time.time() + start_delay  # 计算 episode 的开始时间
                    t_start = time.monotonic() + start_delay  # 记录单调时间，用于后续时间计算
                    env.start_episode(eval_t_start)  # 通知环境开始 episode
                    # 等待 1/30 秒以获取最接近的帧，减少整体延迟
                    frame_latency = 1/30  # 帧延迟时间
                    precise_wait(eval_t_start - frame_latency, time_func=time.time)  # 精确等待
                    print("Started!")  # 打印开始信息
                    iter_idx = 0  # 初始化迭代索引
                    term_area_start_timestamp = float('inf')  # 初始化终止区域开始时间戳
                    perv_target_pose = None  # 初始化前一个目标姿态
                    while True:
                        # 计算当前周期的结束时间
                        t_cycle_end = t_start + (iter_idx + steps_per_inference) * dt

                        # 获取观测数据
                        print('get_obs')  # 打印获取观测数据的提示
                        obs = env.get_obs()  # 从环境中获取观测数据
                        obs_timestamps = obs['timestamp']  # 获取观测数据的时间戳
                        print(f'Obs latency {time.time() - obs_timestamps[-1]}')  # 打印观测延迟

                        # 运行推理
                        with torch.no_grad():  # 禁用梯度计算
                            s = time.time()  # 记录推理开始时间
                            obs_dict_np = get_real_obs_dict(
                                env_obs=obs, shape_meta=cfg.task.shape_meta)  # 获取真实的观测字典
                            obs_dict = dict_apply(obs_dict_np, 
                                lambda x: torch.from_numpy(x).unsqueeze(0).to(device))  # 将观测数据转换为张量
                            result = policy.predict_action(obs_dict)  # 使用策略预测动作
                            # 这个动作从第一个观测步骤开始
                            action = result['action'][0].detach().to('cpu').numpy()  # 将动作转换为 numpy 数组
                            print('Inference latency:', time.time() - s)  # 打印推理延迟
                        
                        # 将策略动作转换为环境动作
                        if delta_action:  # 如果使用增量动作
                            assert len(action) == 1  # 确保动作长度为1
                            if perv_target_pose is None:  # 如果前一个目标姿态为空
                                perv_target_pose = obs['robot_eef_pose'][-1]  # 使用当前机器人末端执行器的姿态
                            this_target_pose = perv_target_pose.copy()  # 复制前一个目标姿态
                            this_target_pose[[0,1]] += action[-1]  # 更新目标姿态
                            perv_target_pose = this_target_pose  # 更新前一个目标姿态
                            this_target_poses = np.expand_dims(this_target_pose, axis=0)  # 扩展维度
                        else:
                            this_target_poses = np.zeros((len(action), len(target_pose)), dtype=np.float64)  # 初始化目标姿态数组
                            this_target_poses[:] = target_pose  # 填充目标姿态
                            this_target_poses[:,[0,1]] = action  # 更新目标姿态的前两个维度

                        # 处理时间
                        # 相同的步骤动作总是目标
                        action_timestamps = (np.arange(len(action), dtype=np.float64) + action_offset
                            ) * dt + obs_timestamps[-1]  # 计算动作时间戳
                        action_exec_latency = 0.01  # 动作执行延迟
                        curr_time = time.time()  # 获取当前时间
                        is_new = action_timestamps > (curr_time + action_exec_latency)  # 判断动作是否在新的时间窗口内
                        if np.sum(is_new) == 0:  # 如果没有新的动作
                            # 超出时间预算，仍然执行一些动作
                            this_target_poses = this_target_poses[[-1]]  # 使用最后一个目标姿态
                            # 安排在下个可用步骤
                            next_step_idx = int(np.ceil((curr_time - eval_t_start) / dt))  # 计算下一个步骤索引
                            action_timestamp = eval_t_start + (next_step_idx) * dt  # 计算动作时间戳
                            print('Over budget', action_timestamp - curr_time)  # 打印超出预算的时间
                            action_timestamps = np.array([action_timestamp])  # 更新动作时间戳
                        else:
                            this_target_poses = this_target_poses[is_new]  # 使用新的目标姿态
                            action_timestamps = action_timestamps[is_new]  # 使用新的动作时间戳

                        # 裁剪动作
                        this_target_poses[:,:2] = np.clip(
                            this_target_poses[:,:2], [0.25, -0.45], [0.77, 0.40])  # 裁剪目标姿态的前两个维度

                        # 执行动作
                        env.exec_actions(
                            actions=this_target_poses,
                            timestamps=action_timestamps
                        )  # 通知环境执行动作
                        print(f"Submitted {len(this_target_poses)} steps of actions.")  # 打印提交的动作步骤数

                        # 可视化
                        episode_id = env.replay_buffer.n_episodes  # 获取当前 episode 的 ID
                        vis_img = obs[f'camera_{vis_camera_idx}'][-1]  # 获取可视化图像
                        text = 'Episode: {}, Time: {:.1f}'.format(
                            episode_id, time.monotonic() - t_start
                        )  # 生成显示文本
                        cv2.putText(
                            vis_img,
                            text,
                            (10,20),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.5,
                            thickness=1,
                            color=(255,255,255)
                        )  # 在图像上绘制文本
                        cv2.imshow('default', vis_img[...,::-1])  # 显示图像


                        key_stroke = cv2.pollKey()  # 获取按键输入
                        if key_stroke == ord('s'):  # 如果按下 's' 键
                            # 停止 episode
                            # 将控制权交还给人类
                            env.end_episode()  # 通知环境结束 episode
                            print('Stopped.')  # 打印停止信息
                            break  # 退出循环

                        # 自动终止
                        terminate = False  # 初始化终止标志
                        if time.monotonic() - t_start > max_duration:  # 如果超过最大持续时间
                            terminate = True  # 设置终止标志
                            print('Terminated by the timeout!')  # 打印超时终止信息

                        term_pose = np.array([ 3.40948500e-01,  2.17721816e-01,  4.59076878e-02,  2.22014183e+00, -2.22184883e+00, -4.07186655e-04])  # 定义终止姿态
                        curr_pose = obs['robot_eef_pose'][-1]  # 获取当前机器人末端执行器的姿态
                        dist = np.linalg.norm((curr_pose - term_pose)[:2], axis=-1)  # 计算当前姿态与终止姿态的距离
                        if dist < 0.03:  # 如果距离小于 0.03
                            # 进入终止区域
                            curr_timestamp = obs['timestamp'][-1]  # 获取当前时间戳
                            if term_area_start_timestamp > curr_timestamp:  # 如果终止区域开始时间戳大于当前时间戳
                                term_area_start_timestamp = curr_timestamp  # 更新终止区域开始时间戳
                            else:
                                term_area_time = curr_timestamp - term_area_start_timestamp  # 计算在终止区域的时间
                                if term_area_time > 0.5:  # 如果超过 0.5 秒
                                    terminate = True  # 设置终止标志
                                    print('Terminated by the policy!')  # 打印策略终止信息
                        else:
                            # 离开终止区域
                            term_area_start_timestamp = float('inf')  # 重置终止区域开始时间戳

                        if terminate:  # 如果终止标志为真
                            env.end_episode()  # 通知环境结束 episode
                            break  # 退出循环

                        # 等待执行
                        precise_wait(t_cycle_end - frame_latency)  # 精确等待
                        iter_idx += steps_per_inference  # 更新迭代索引

                except KeyboardInterrupt:  # 捕获键盘中断异常
                    print("Interrupted!")  # 打印中断信息
                    # 停止机器人
                    env.end_episode()  # 通知环境结束 episode
                
                print("Stopped.")  # 打印停止信息



# %%
if __name__ == '__main__':
    main()
