import zarr
import numpy as np

# 打开 Zarr 存储
store_path = '/home/xuanya/eto/diffusion_policy/data/real_pusht_17/replay_buffer.zarr'
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
            for value in array[0:130]:
                print(value)
        else:
            # 对于多维数组，直接打印前五行
            print(array[0:130])

# 访问 data 组并打印基本信息和前几行数据
if 'data' in root:
    print("\nData Group Contents:")
    print_array_info(root['data'], prefix="data/")

# 访问 meta 组并打印基本信息和前几行数据
if 'meta' in root:
    print("\nMeta Group Contents:")
    print_array_info(root['meta'], prefix="meta/")