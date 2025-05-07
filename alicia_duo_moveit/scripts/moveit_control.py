#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import numpy as np


class MoveItRobotController:
    def __init__(self, manipulator_group="manipulator", gripper_group="gripper", velocity=0.4):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.get_node_uri():
            rospy.init_node('moveit_robot_controller', anonymous=True)  

        # Robot and planning interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Move groups
        self.manipulator = moveit_commander.MoveGroupCommander(manipulator_group)
        self.gripper = moveit_commander.MoveGroupCommander(gripper_group)

        # Set velocity scaling (0.0 to 1.0)
        self.manipulator.set_max_velocity_scaling_factor(velocity)
        self.gripper.set_max_velocity_scaling_factor(velocity)

        self.manipulator.set_planner_id("RRTConnectkConfigDefault")  # 更快更稳定
        self.manipulator.set_planning_time(10.0)                     # 增加规划时间
        self.manipulator.set_num_planning_attempts(10)
        self.manipulator.set_goal_position_tolerance(0.01)
        self.manipulator.set_goal_orientation_tolerance(0.01)
        self.manipulator.allow_replanning(True)

        rospy.loginfo("MoveItRobotController initialized.")
        self.gripper_pub = rospy.Publisher('/gripper_control', Float32, queue_size=10)

    def gripper_control(self, value):
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Publishing gripper control value: %f", value)
        self.gripper_pub.publish(Float32(data=value))


    def move_to_pose(self, pose):
        self.manipulator.set_pose_target(pose)
        success, plan, _, _ = self.manipulator.plan(pose)
        if not success:
            rospy.logwarn("IK solution could not be found for pre-grasp pose.")
        else:
            rospy.loginfo("IK plan success.")

        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        return success


    def publish_object_tf(self, obj_pos):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"  # or "base_link" if that’s your base
        t.child_frame_id = "pre_pose"
        t.transform.translation.x =  obj_pos.position.x
        t.transform.translation.y =  obj_pos.position.y
        t.transform.translation.z =  obj_pos.position.z

        # quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = obj_pos.orientation.x 
        t.transform.rotation.y = obj_pos.orientation.y
        t.transform.rotation.z = obj_pos.orientation.z
        t.transform.rotation.w = obj_pos.orientation.w

        self.tf_broadcaster.sendTransform(t)

    
    def move_to_joint_state(self, joint_goals):
        self.manipulator.set_joint_value_target(joint_goals)
        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        return success

    def execute_trajectory(self, trajectory):
        """
        执行给定的机器人轨迹
        
        Args:
            trajectory (moveit_msgs.msg.RobotTrajectory): 要执行的轨迹对象
            
        Returns:
            bool: 执行成功返回True，否则返回False
        """
        try:
            rospy.loginfo("开始执行轨迹，共 %d 个点，时长 %.2f 秒", 
                        len(trajectory.joint_trajectory.points), 
                        trajectory.joint_trajectory.points[-1].time_from_start.to_sec())
            
            # 检查轨迹是否为空
            if not trajectory.joint_trajectory.points:
                rospy.logerr("轨迹为空，无法执行")
                return False
                
            # 验证关节名称是否与机器人模型匹配
            robot_joints = set(self.manipulator.get_active_joints())
            trajectory_joints = set(trajectory.joint_trajectory.joint_names)
            
            # 如果轨迹中包含的关节不在机器人关节列表中，发出警告
            if not trajectory_joints.issubset(robot_joints):
                rospy.logwarn("轨迹包含未知关节: %s", 
                            trajectory_joints.difference(robot_joints))
                rospy.logwarn("机器人有效关节: %s", robot_joints)
                
                # 尝试修复轨迹中的关节名称
                if len(trajectory_joints) == len(robot_joints):
                    rospy.logwarn("尝试修复轨迹关节名称...")
                    # 创建新的轨迹对象
                    fixed_trajectory = self._fix_joint_names(trajectory, list(robot_joints))
                    trajectory = fixed_trajectory
            
            # 执行轨迹
            result = self.manipulator.execute(trajectory, wait=True)
            
            # 停止移动并清理目标
            self.manipulator.stop()
            
            if result:
                rospy.loginfo("轨迹执行成功完成")
            else:
                rospy.logwarn("轨迹执行失败或被取消")
                
            return result
            
        except Exception as e:
            import traceback
            rospy.logerr("轨迹执行过程中发生错误: %s", e)
            traceback.print_exc()
            return False
            
    def _fix_joint_names(self, trajectory, correct_joint_names):
        """
        修复轨迹中的关节名称，使其与机器人模型匹配
        
        Args:
            trajectory: 原始轨迹对象
            correct_joint_names: 正确的关节名称列表
            
        Returns:
            修复后的轨迹对象
        """
        import copy
        
        # 创建新的轨迹对象
        fixed_trajectory = copy.deepcopy(trajectory)
        
        # 替换关节名称
        if len(fixed_trajectory.joint_trajectory.joint_names) == len(correct_joint_names):
            fixed_trajectory.joint_trajectory.joint_names = correct_joint_names
            rospy.loginfo("轨迹关节名称已修复")
        
        return fixed_trajectory

    def set_velocity_scaling_factor(self, factor):
        """
        设置机器人执行速度的缩放因子
        
        Args:
            factor (float): 速度缩放因子 (0.0 到 1.0)
            
        Returns:
            None
        """
        # 确保因子在有效范围内
        factor = max(0.01, min(1.0, factor))
        self.manipulator.set_max_velocity_scaling_factor(factor)
        rospy.loginfo("速度缩放因子设置为: %.2f", factor)
        
 

if __name__ == '__main__':
    controller = MoveItRobotController()

    # Example: Move to a Cartesian pose
    target_pose = Pose()
    # target_pose.orientation.w = 1.0
    # target_pose.position.x = 0.4
    # target_pose.position.y = 0.1
    # target_pose.position.z = 0.4

    # rospy.loginfo("Moving to target pose...")
    # success = controller.move_to_pose(target_pose)
    # rospy.loginfo("Move to pose: %s", "Success" if success else "Failed")

    # Print current joint values and pose
    
    ## Set pose
    
    # Gripper goal pose
    gripper_goal = Pose()
    gripper_goal.position.x = 0.25 #0.22433333
    gripper_goal.position.y = 0.0 #0.12029027
    gripper_goal.position.z = 0.09#-0.02354876
    q = quaternion_from_euler(np.pi, np.pi/2, 0)  # roll 180 degrees
    print(q)
        # ee_pose = [0.22433333,  0.12029027, -0.02354876, q[0], q[1], q[2]]
        # 设置合法 orientation（无旋转，单位四元数）
    gripper_goal.orientation.x = q[0]
    gripper_goal.orientation.y = q[1]
    gripper_goal.orientation.z = q[2]
    gripper_goal.orientation.w = q[3]


    # rospy.loginfo("Trying to move to gripper goal pose...")

    # success = controller.move_to_pose_no_gripper(gripper_goal)


    target_pose = controller.get_current_pose()
    rospy.loginfo("target_pose")
    rospy.loginfo(target_pose)
    target_pose.position.z -= 0.06  # 向下移动 5cm

    # success = controller.move_to_pose_linear(target_pose)

    success = controller.move_to_pose(target_pose)

    if success:
        rospy.loginfo("Successfully moved to gripper pose!")
    else:
        rospy.logwarn("Failed to move to gripper pose.")

    # ## Read pose
    # print("Current Joint States:", controller.get_joint_states())
    # print("Current Pose:", controller.get_current_pose())
    
    
    # gripper_pose = controller.get_current_pose_with_gripper()
    # if gripper_pose:
    #     print("Gripper Pose:")
    #     print(f"Position: {gripper_pose.position}")
    #     print(f"Orientation: {gripper_pose.orientation}")
