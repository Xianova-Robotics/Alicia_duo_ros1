#!/usr/bin/env python

# import zarr
# import numpy as np

# class ZarrDataReader:
#     def __init__(self, zarr_dir):
#         # 打开 Zarr 文件夹（存储路径）
#         self.root = zarr.open(zarr_dir, mode='r')

#         # 读取 data 和 meta 数据
#         self.data_group = self.root['data']
#         self.meta_group = self.root['meta']

#     def read_data(self):
#         # 读取并打印每个数据集
#         action = self.data_group['action'][:]
#         robot_eef_pose = self.data_group['robot_eef_pose'][:]
#         robot_eef_pose_vel = self.data_group['robot_eef_pose_vel'][:]
#         robot_joint = self.data_group['robot_joint'][:]
#         robot_joint_vel = self.data_group['robot_joint_vel'][:]
#         stage = self.data_group['stage'][:]
#         timestamp = self.data_group['timestamp'][:]
#         episode_ends = self.meta_group['episode_ends'][:]

#         # 打印数据内容
#         print(f"Action Data: {action.shape}")
#         print(action)

#         print(f"\nRobot EEF Pose Data: {robot_eef_pose.shape}")
#         print(robot_eef_pose)

#         print(f"\nRobot EEF Pose Velocity Data: {robot_eef_pose_vel.shape}")
#         print(robot_eef_pose_vel)

#         print(f"\nRobot Joint Data: {robot_joint.shape}")
#         print(robot_joint)

#         print(f"\nRobot Joint Velocity Data: {robot_joint_vel.shape}")
#         print(robot_joint_vel)

#         print(f"\nStage Data: {stage.shape}")
#         print(stage)

#         print(f"\nTimestamp Data: {timestamp.shape}")
#         print(timestamp)

#         print(f"\nEpisode Ends Data: {episode_ends.shape}")
#         print(episode_ends)

# def main():
#     zarr_dir = './zarr_folder.zarr'  # Zarr 文件存储路径

#     # 创建 ZarrDataReader 实例
#     zarr_reader = ZarrDataReader(zarr_dir)

#     # 读取并打印数据
#     zarr_reader.read_data()

# if __name__ == '__main__':
#     main()

import zarr
import numpy as np

# 打开 Zarr 存储
store_path = '/home/eto/Alicia_duo_ros/real_pusht_2025_1_7/replay_buffer.zarr'
# store_path = '/home/ubuntu/eto/diffusion_policy/example.zarr'
root = zarr.open(store_path, mode='r')

print(root.tree())

def print_array_info(group, prefix=""):
    """Print information and first few rows of each array in the group."""
    for key in group.keys():
        array = group[key]
        print(f"\n{prefix}{key} array shape: {array.shape}, dtype: {array.dtype}")
        print(f"First few rows of {prefix}{key} data:")
        
        if len(array.shape) == 1:
            # 对于一维数组，逐行打印每个元素
            for value in array[:20]:
                print(value)
        else:
            # 对于多维数组，直接打印前五行
            print(array[:20])

# 访问 data 组并打印基本信息和前几行数据
if 'data' in root:
    print("\nData Group Contents:")
    print_array_info(root['data'], prefix="data/")

# 访问 meta 组并打印基本信息和前几行数据
if 'meta' in root:
    print("\nMeta Group Contents:")
    print_array_info(root['meta'], prefix="meta/")


