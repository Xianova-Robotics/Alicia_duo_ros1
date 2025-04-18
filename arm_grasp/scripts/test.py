import numpy as np
from scipy.spatial.transform import Rotation

# 输入数据
translation = np.array([0.3386691856301935, 0.010932772403258913, 0.4555244933727863])  # camera_link → arm_base
quaternion = np.array([-0.6750318965095816, -0.02152222698097017, 0.7358861331826768, 0.048376972103515964])  # x, y, z, w
obj_camera_optical = np.array([0.016, 0.014, 0.496, 1.0])  # 物体在光学帧中的坐标（齐次）

# 1. 光学帧到相机本体帧的变换（来自static_transform_publisher）
# 欧拉角: [roll=-π/2, pitch=0, yaw=-π/2]
R_optical_to_link = Rotation.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_matrix()
T_optical_to_link = np.eye(4)
T_optical_to_link[:3, :3] = R_optical_to_link  # 无平移

# 2. 相机本体帧到机械臂基坐标系的变换（来自标定数据）
rotation = Rotation.from_quat(quaternion)
R_link_to_arm = rotation.as_matrix()
T_link_to_arm = np.eye(4)
T_link_to_arm[:3, :3] = R_link_to_arm
T_link_to_arm[:3, 3] = translation

# 3. 组合变换：光学帧 → 相机本体帧 → 机械臂基坐标系
T_optical_to_arm = T_link_to_arm @ T_optical_to_link

# 4. 变换物体坐标
obj_arm = T_optical_to_arm @ obj_camera_optical

print("物体在世界坐标系中的位置 (x, y, z):", obj_arm[:3])

# import numpy as np
# from tf.transformations import quaternion_matrix

# # Calibration data
# translation = [0.3386691856301935, 0.010932772403258913, 0.4555244933727863]
# quaternion = [-0.6750318965095816, -0.02152222698097017, 0.7358861331826768, 0.048376972103515964]  # x, y, z, w

# # Build transformation matrix from camera to world
# T_world_camera = quaternion_matrix(quaternion)
# T_world_camera[:3, 3] = translation

# # Object pose in camera frame
# obj_camera = np.array([0.016, 0.014, 0.496, 1.0])

# # Transform to world frame
# obj_world = T_world_camera @ obj_camera
# x_w, y_w, z_w = obj_world[:3]

# print(f"Object in world frame:\n  x = {x_w:.4f} m\n  y = {y_w:.4f} m\n  z = {z_w:.4f} m")


################################
#  For Object Detection Test   #
################################


"""
#!/usr/bin/env python
from Obpose import ObjectPoseTransformer
import rospy

rospy.init_node('external_node', anonymous=True)

transformer = ObjectPoseTransformer()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pos = transformer.get_latest_position()
    if pos is not None:
        print("Object Position in Table Frame:", pos)
    rate.sleep()
"""