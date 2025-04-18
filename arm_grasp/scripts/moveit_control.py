#!/usr/bin/env python
import sys
import rospy
import moveit_commander
# import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from pose_trans_link6_gripper import GripperPoseTF, GripperToLink6Transformer


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


        # Move groups
        self.manipulator = moveit_commander.MoveGroupCommander(manipulator_group)
        self.gripper = moveit_commander.MoveGroupCommander(gripper_group)

        # Set velocity scaling (0.0 to 1.0)
        self.manipulator.set_max_velocity_scaling_factor(velocity)
        self.gripper.set_max_velocity_scaling_factor(velocity)

        rospy.loginfo("MoveItRobotController initialized.")

    def get_current_pose(self):
        return self.manipulator.get_current_pose().pose
    
    def get_current_pose_with_gripper(self):
        return self.gripper_pose_get.get_gripper_pose_in_base("base_link", "tcp_link")
    
    def get_joint_states(self):
        return self.manipulator.get_current_joint_values()

    def move_to_pose_no_gripper(self, pose):
        self.manipulator.set_pose_target(pose)
        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        return success

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
    gripper_goal.position.x = 0.28130450767510784
    gripper_goal.position.y = 0.011667878422168218
    gripper_goal.position.z = 0.07427088415028363
    gripper_goal.orientation.x = -0.9131221360932806
    gripper_goal.orientation.y = -0.01642308906810286
    gripper_goal.orientation.z = 0.40727551917969385
    gripper_goal.orientation.w = -0.008055941832999973

    rospy.loginfo("Trying to move to gripper goal pose...")

    success = controller.move_to_pose_gripper(gripper_goal)

    if success:
        rospy.loginfo("Successfully moved to gripper pose!")
    else:
        rospy.logwarn("Failed to move to gripper pose.")

    ## Read pose
    print("Current Joint States:", controller.get_joint_states())
    print("Current Pose:", controller.get_current_pose())
    
    
    gripper_pose = controller.get_current_pose_with_gripper()
    if gripper_pose:
        print("Gripper Pose:")
        print(f"Position: {gripper_pose.position}")
        print(f"Orientation: {gripper_pose.orientation}")
