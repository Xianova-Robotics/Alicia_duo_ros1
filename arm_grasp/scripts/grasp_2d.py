#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from Obpose import ObjectPoseTransformer
from moveit_control import MoveItRobotController

class GraspController:
    def __init__(self):
        rospy.init_node('grasp_controller')
        
        # 初始化物体检测和机械臂控制
        self.object_transformer = ObjectPoseTransformer()
        self.robot_controller = MoveItRobotController()
        
        # 参数
        self.grasp_height_offset = 0.02  # 抓取高度补偿
        self.approach_distance = 0.04    # 抓取前接近高度
        self.lift_distance = 0.07         # 抬升高度
    
    def get_object_pose(self, timeout=5.0):
        """获取物体在机械臂基坐标系中的位置（不设置 orientation）"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            obj_pos = self.object_transformer.get_latest_position()
            if obj_pos is not None:
                pose = Pose()
                pose.position.x = obj_pos[0]
                pose.position.y = obj_pos[1]
                pose.position.z = obj_pos[2] + self.grasp_height_offset
                # 不设置 orientation，交由 move_group 处理默认朝向
                rospy.loginfo(f"Detected object at: {obj_pos}")
                return pose
            rospy.sleep(0.1)
        rospy.logwarn("Failed to detect object position")
        return None
    
    def execute_grasp(self):
        """执行抓取流程"""
        target_pose = self.get_object_pose()
        if not target_pose:
            return False
        
        # 预抓取位姿
        # 预抓取位姿
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = target_pose.position.x
        pre_grasp_pose.position.y = target_pose.position.y
        pre_grasp_pose.position.z = target_pose.position.z + self.approach_distance

        # 设置合法 orientation（无旋转，单位四元数）
        pre_grasp_pose.orientation.x = 0.0
        pre_grasp_pose.orientation.y = 0.0
        pre_grasp_pose.orientation.z = 0.0
        pre_grasp_pose.orientation.w = 1.0


        rospy.loginfo("Moving to pre-grasp position...")
        rospy.loginfo(f"Pre-grasp pose: {pre_grasp_pose}")

        if not self.robot_controller.move_to_pose_gripper(pre_grasp_pose):
            rospy.logerr("Failed to move to pre-grasp position")
            return False
        
        # 抓取位姿（线性下降）
        # rospy.loginfo("Descending to grasp position...")
        # if not self.robot_controller.move_to_pose_linear(target_pose):
        #     rospy.logerr("Failed to descend to grasp position")
        #     return False
        
        # rospy.loginfo("Closing gripper...")
        # if not self.robot_controller.close_gripper():
        #     rospy.logerr("Failed to close gripper")
        #     return False
        
        # 抬升
        lift_pose = Pose()
        lift_pose.position.x = target_pose.position.x
        lift_pose.position.y = target_pose.position.y
        lift_pose.position.z = target_pose.position.z + self.lift_distance

        rospy.loginfo("Lifting object...")
        if not self.robot_controller.move_to_pose_gripper(lift_pose):
            rospy.logerr("Failed to lift object")
            return False
        
        # # 移动回 home 位姿
        # rospy.loginfo("Returning to home position...")
        # if not self.robot_controller.home():
        #     rospy.logerr("Failed to return home")
        #     return False
        
        return True

if __name__ == '__main__':
    try:
        controller = GraspController()
        success = controller.execute_grasp()
        rospy.loginfo("Grasp execution: %s", "SUCCESS" if success else "FAILED")
    except rospy.ROSInterruptException:
        pass



# #!/usr/bin/env python
# import rospy
# import numpy as np
# from geometry_msgs.msg import Pose
# from Obpose import ObjectPoseTransformer
# import sys
# import rospy
# import rospkg 
# from os.path import join

# # 添加你的工作空间Python路径
# # sys.path.append(join(rospkg.RosPack().get_path('robot_control'), 'src'))
# from moveit_control import MoveItRobotController

# class GraspController:
#     def __init__(self):
#         rospy.init_node('grasp_controller')
        
#         # 初始化物体检测和机械臂控制
#         self.object_transformer = ObjectPoseTransformer()
#         self.robot_controller = MoveItRobotController()
        
#         # 预设抓取参数
#         self.grasp_height_offset = 0.02  # 抓取高度补偿（根据夹爪长度调整）
#         self.approach_distance = 0.04    # 接近距离（直线接近时使用）
        
#     def get_object_pose(self, timeout=5.0):
#         """获取物体在机械臂基坐标系中的位姿"""
#         start_time = rospy.Time.now()
#         while (rospy.Time.now() - start_time).to_sec() < timeout:
#             obj_pos = self.object_transformer.get_latest_position()
#             if obj_pos is not None:
#                 pose = Pose()
#                 pose.position.x = obj_pos[0]
#                 pose.position.y = obj_pos[1]
#                 pose.position.z = obj_pos[2] + self.grasp_height_offset
#                 pose.orientation.w = 1.0  # 默认朝向
#                 rospy.loginfo(f"Detected object at: {obj_pos}")
#                 return pose
#             rospy.sleep(0.1)
#         rospy.logwarn("Failed to detect object position")
#         return None
    
#     def execute_grasp(self):
#         """执行完整的抓取流程"""
#         # 1. 获取物体位置
#         target_pose = self.get_object_pose()
#         if not target_pose:
#             return False
        
#         # 2. 预抓取位置（从上方接近）
#         pre_grasp_pose = Pose()
#         pre_grasp_pose.position.x = target_pose.position.x
#         pre_grasp_pose.position.y = target_pose.position.y
#         pre_grasp_pose.position.z = target_pose.position.z + self.approach_distance
#         pre_grasp_pose.orientation = target_pose.orientation
        
#         # 3. 移动到预抓取位置
#         rospy.loginfo("Moving to pre-grasp position...")
#         if not self.robot_controller.move_to_pose_gripper(pre_grasp_pose):
#             rospy.logerr("Failed to move to pre-grasp position")
#             return False
        
#         # 4. 直线下降到抓取位置
#         rospy.loginfo("Descending to grasp position...")
#         if not self.robot_controller.move_to_pose_linear(target_pose):
#             rospy.logerr("Failed to descend to grasp position")
#             return False
        
#         # # 5. 闭合夹爪
#         # rospy.loginfo("Closing gripper...")
#         # if not self.robot_controller.close_gripper():
#         #     rospy.logerr("Failed to close gripper")
#         #     return False
        
#         # 6. 抬起到运输高度
#         lift_pose = Pose()
#         lift_pose.position.x = target_pose.position.x
#         lift_pose.position.y = target_pose.position.y
#         lift_pose.position.z = target_pose.position.z + 0.7
#         lift_pose.orientation = target_pose.orientation
        
#         rospy.loginfo("Lifting object...")
#         if not self.robot_controller.move_to_pose_gripper(lift_pose):
#             rospy.logerr("Failed to lift object")
#             return False
        
#         return True

# if __name__ == '__main__':
#     try:
#         controller = GraspController()
#         success = controller.execute_grasp()
#         rospy.loginfo("Grasp execution: %s", "SUCCESS" if success else "FAILED")
#     except rospy.ROSInterruptException:
#         pass