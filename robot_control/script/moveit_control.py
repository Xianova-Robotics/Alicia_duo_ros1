#!/usr/bin/env python
import sys
import rospy
import moveit_commander
# import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from pose_trans_link6_gripper import GripperPoseTF, GripperToLink6Transformer
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import numpy as np


class MoveItRobotController:
    def __init__(self, manipulator_group="manipulator", gripper_group="gripper", velocity=0.3):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.get_node_uri():
            rospy.init_node('moveit_robot_controller', anonymous=True)  

        # Robot and planning interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.gripper_pose_get = GripperPoseTF()
        self.pose_converter = GripperToLink6Transformer()

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

    def get_current_pose(self):
        return self.manipulator.get_current_pose().pose
    
    def get_current_pose_with_gripper(self):
        return self.gripper_pose_get.get_gripper_pose_in_base("base_link", "tcp_link")
    
    def get_joint_states(self):
        return self.manipulator.get_current_joint_values()

    def move_to_pose_no_gripper(self, pose):
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


    def move_to_pose_gripper(self, pose):
        # Convert desired gripper pose to corresponding link6 pose
        link6_pose = self.pose_converter.get_link6_pose_from_gripper_pose(pose)
        if link6_pose is None:
            rospy.logwarn("Failed to compute link6 pose from gripper pose.")
            return False
        return self.move_to_pose_no_gripper(link6_pose)
    
    def move_to_joint_state(self, joint_goals):
        self.manipulator.set_joint_value_target(joint_goals)
        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        return success



    def move_to_pose_linear(self, target_pose, eef_step=0.01, jump_threshold=0.0):
        """
        Move the end-effector in a straight line (Cartesian path) to the target pose.
        :param target_pose: geometry_msgs/Pose
        :param eef_step: resolution in meters (e.g. 0.01 means 1cm step)
        :param jump_threshold: usually 0.0 to disable joint-space jump avoidance
        :return: True if path was executed successfully
        """
        waypoints = []
        waypoints.append(self.manipulator.get_current_pose().pose)
        waypoints.append(target_pose)

        (plan, fraction) = self.manipulator.compute_cartesian_path(
            waypoints,
            eef_step,
            True  # 是否避免碰撞（bool 类型）
        )
        if fraction < 1.0:
            rospy.logwarn(f"Cartesian path planning incomplete: only {fraction*100:.1f}% achieved.")
            return False

        rospy.loginfo("Executing linear (Cartesian) path...")
        result = self.manipulator.execute(plan, wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()

        if not result:
            rospy.logerr("Cartesian execution failed.")
            return False

        rospy.loginfo("Successfully moved along Cartesian path.")
        return True



    def open_gripper(self):
        self.gripper.set_named_target("open")
        return self.gripper.go(wait=True)

    def close_gripper(self):
        self.gripper.set_named_target("close")
        return self.gripper.go(wait=True)





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

    success = controller.move_to_pose_no_gripper(target_pose)

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
