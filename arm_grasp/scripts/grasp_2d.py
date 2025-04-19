#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Pose
from Obpose import ObjectPoseTransformer
from moveit_control import MoveItRobotController
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32

class GraspController:
    def __init__(self):
        rospy.init_node('grasp_controller')
        
        # 初始化物体检测和机械臂控制
        self.object_transformer = ObjectPoseTransformer()
        self.robot_controller = MoveItRobotController()

        # 参数
        self.grasp_height_offset = 0.02  # 抓取高度补偿
        self.approach_distance = 0.07    # 抓取前接近高度
        self.lift_distance = 0.03         # 抬升高度
        self.gripper_pub = rospy.Publisher('/gripper_control', Float32, queue_size=10)

        
    def gripper_control(self, value):
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Publishing gripper control value: %f", value)
        self.gripper_pub.publish(Float32(data=value))

    def get_object_pose(self, timeout=5.0):
        """获取物体在机械臂基坐标系中的位置（不设置 orientation）"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            obj_pos = self.object_transformer.get_latest_position()
            if obj_pos is not None:
                rospy.loginfo(f"Detected pose: {obj_pos}")
                pose = Pose()
                pose.position.x = obj_pos[0]
                pose.position.y = obj_pos[1]
                pose.position.z = obj_pos[2] + self.grasp_height_offset
                return pose

            rospy.sleep(0.1)
        rospy.logwarn("Failed to detect object position")
        return None

    def execute_grasp(self):
        """执行抓取流程"""
        target_pose = self.get_object_pose()
        if not target_pose:
            return False
        time.sleep(0.2)
        self.gripper_control(0.0)
        # 预抓取位姿
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = target_pose.position.x #- 0.02
        pre_grasp_pose.position.y = target_pose.position.y #- 0.02
        pre_grasp_pose.position.z = target_pose.position.z + self.approach_distance

        q = quaternion_from_euler(np.pi, np.pi/2, 0)  # roll 180 degrees
        print(q)
        pre_grasp_pose.orientation.x = q[0] 
        pre_grasp_pose.orientation.y = q[1] 
        pre_grasp_pose.orientation.z = q[2]
        pre_grasp_pose.orientation.w = q[3]
        # time.sleep(0.2)
        self.gripper_control(0.0)
        rospy.loginfo("Moving to pre-grasp position...")
        if not self.robot_controller.move_to_pose(pre_grasp_pose):
            rospy.logerr("Failed to lift object")
            return False


        # 抓取位姿（下降）
        current_pose = self.robot_controller.get_current_pose()
        rospy.loginfo("target_pose")
        rospy.loginfo(target_pose)
        current_pose.position.z = target_pose.position.z - 0.03 # 向下移动 5cm

        # success = controller.move_to_pose(target_pose)
        if not self.robot_controller.move_to_pose(current_pose):
            rospy.logerr("Successfully moved to target pose!")
            return False
            
        # Close gripper
        # rospy.sleep(3)
        self.gripper_control(1) # 0-1
        # rospy.sleep(1)
        # rospy.loginfo("Lifting object...")
        # if not self.robot_controller.move_to_pose(lift_pose):
        #     rospy.logerr("Failed to lift object")
        #     return False
        
        # # 移动回 home 位姿
        home_joint_values = self.robot_controller.manipulator.get_named_target_values("home")
        print("Home joint values:", home_joint_values)
        if not self.robot_controller.move_to_joint_state(home_joint_values):
            rospy.logerr("Failed to go home")
            return False
        return True
        # Open gripper
        self.gripper_control(0) # 0-1

if __name__ == '__main__':
    try:
        controller = GraspController()
        success = controller.execute_grasp()
        rospy.loginfo("Grasp execution: %s", "SUCCESS" if success else "FAILED")
    except rospy.ROSInterruptException:
        pass



