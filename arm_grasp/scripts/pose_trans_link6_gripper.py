#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import *

class GripperPoseTF:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_gripper_pose_in_base(self, base_frame="base_link", gripper_frame="tcp_link"):
        try:
            trans = self.tf_buffer.lookup_transform(base_frame, gripper_frame, rospy.Time(0), rospy.Duration(1.0))
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            rospy.logwarn(f"Failed to lookup gripper pose: {e}")
            return None




class GripperToLink6Transformer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_link6_pose_from_gripper_pose(self, gripper_pose, base_frame="base_link", gripper_frame="tcp_link", link6_frame="Link06"):
        # Get the static transform from link6 to tcp_link
        try:
            tf_gripper_to_link6 = self.tf_buffer.lookup_transform(link6_frame, gripper_frame, rospy.Time(0), rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return None

        # Convert Pose to 4x4 matrix
        T_base_to_gripper = self.pose_to_matrix(gripper_pose)
        T_gripper_to_link6 = self.transform_to_matrix(tf_gripper_to_link6.transform)

        # T_base_to_link6 = T_base_to_gripper * inverse(T_gripper_to_link6)
        T_base_to_link6 = np.dot(T_base_to_gripper, np.linalg.inv(T_gripper_to_link6))

        # Convert back to Pose
        return self.matrix_to_pose(T_base_to_link6)

    def pose_to_matrix(self, pose):
        trans = translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        rot = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return np.dot(trans, rot)

    def transform_to_matrix(self, transform):
        trans = translation_matrix([transform.translation.x, transform.translation.y, transform.translation.z])
        rot = quaternion_matrix([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        return np.dot(trans, rot)

    def matrix_to_pose(self, mat):
        trans = translation_from_matrix(mat)
        quat = quaternion_from_matrix(mat)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = trans
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        return pose

