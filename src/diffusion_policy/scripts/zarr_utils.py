import numpy as np
import zarr
import os

class ZarrDataCreator:
    def __init__(self):
        self.root = None
        self.data_group = None
        self.meta_group = None

    def initialize_zarr_store(self, zarr_dir='./zarr_test'):
        """
        初始化 Zarr 存储文件和数据集结构。该函数只需要调用一次。
        
        参数:
        - zarr_dir: Zarr 文件存储路径
        """
        # 创建 Zarr 存储文件夹
        if not os.path.exists(zarr_dir):
            os.makedirs(zarr_dir)

        # 创建根目录
        self.root = zarr.open(zarr_dir, mode='w')

        # 创建 data 子目录，存储不同的观测数据
        self.data_group = self.root.create_group('data')

        # 创建 meta 子目录，存储元数据
        self.meta_group = self.root.create_group('meta')

        # 初始化各个数据集
        self.create_zarr_store()

    def create_zarr_store(self):
        """
        创建 Zarr 数据集，初始化为(0, 6)或(0,)维度，随着数据写入逐步扩展
        """
        self.data_group.create_dataset('action', shape=(0, 6), dtype='f8', chunks=(100, 6))
        self.data_group.create_dataset('robot_eef_pose', shape=(0, 6), dtype='f8', chunks=(100, 6))
        self.data_group.create_dataset('robot_eef_pose_vel', shape=(0, 6), dtype='f8', chunks=(100, 6))
        self.data_group.create_dataset('robot_joint', shape=(0, 6), dtype='f8', chunks=(100, 6))
        self.data_group.create_dataset('robot_joint_vel', shape=(0, 6), dtype='f8', chunks=(100, 6))
        self.data_group.create_dataset('stage', shape=(0,), dtype='i8', chunks=(100,))
        self.data_group.create_dataset('timestamp', shape=(0,), dtype='f8', chunks=(100,))

        # 创建 meta 数据集
        self.meta_group.create_dataset('episode_ends', shape=(0,), dtype='i8', chunks=(100,))

    def append_data(self, action, robot_eef_pose, robot_eef_pose_vel, robot_joint, robot_joint_vel, stage, timestamp):
        """
        追加实时数据到 Zarr 文件。
        
        参数:
        - action: 6 维 action 数据
        - robot_eef_pose: 6 维末端执行器位姿数据
        - robot_eef_pose_vel: 6 维末端执行器速度数据
        - robot_joint: 6 维关节角度数据
        - robot_joint_vel: 6 维关节速度数据
        - stage: 整数，表示当前阶段
        - timestamp: 时间戳，单位秒
        - episode_ends: 整数数组，表示 episode 结束的索引
        """
        self.data_group['action'].append([action], axis=0)
        self.data_group['robot_eef_pose'].append([robot_eef_pose], axis=0)
        self.data_group['robot_eef_pose_vel'].append([robot_eef_pose_vel], axis=0)
        self.data_group['robot_joint'].append([robot_joint], axis=0)
        self.data_group['robot_joint_vel'].append([robot_joint_vel], axis=0)
        self.data_group['stage'].append([stage], axis=0)
        self.data_group['timestamp'].append([timestamp], axis=0)

    def append_meta_data(self, episode_end):
        """
        追加一个 episode_end 元数据到 Zarr 文件。
        
        参数:
        - episode_end: 整数，表示 episode 结束的索引
        """
        # 将新的 episode_end 数据转换为一维数组 (1,)
        new_data = np.array([episode_end], dtype='i8')
        
        # 使用 append() 方法来追加数据
        self.meta_group['episode_ends'].append(new_data)





