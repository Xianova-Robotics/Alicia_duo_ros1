

###############################
#    Inverse Kinematics       #
###############################
from tf.transformations import quaternion_from_euler
from Robolab_main import robolab

obj_pose = [0.22433333,  0.12029027, -0.02354876]
print("########## Inverse kinematics from URDF or MuJoCo XML files with RobotModel class ##########")
print("---------- Inverse kinematics for Franka Panda using URDF file ----------")
model_path = "/home/xuanya/demo_ws/src/alicia_duo_v5_03/urdf/v1.urdf"
q = quaternion_from_euler(3.1416, 0, 0)  # roll 180 degrees
print(q)
ee_pose = [0.22433333,  0.12029027, -0.02354876, q[0], q[1], q[2]]
export_link = "tcp_link"
robot = robolab.RobotModel(model_path, solve_engine="pytorch_kinematics", verbose=True)

# Get ik solution
ret = robot.get_ik(ee_pose, export_link)
print(ret.solutions)

# Get ik solution near the current configuration
cur_configs = [[-1.7613, 2.7469, -3.5611, -3.8847, 2.7940, 1.9055, 1.9879]]
ret = robot.get_ik(ee_pose, export_link, cur_configs=cur_configs)
print(ret.solutions)



##################################
#Object Detection In World Frame #
##################################
"""
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
"""


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