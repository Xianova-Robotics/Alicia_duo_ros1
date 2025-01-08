#!/usr/bin/env python

import rospy
import torch
from diffusion_policy.msg import get_angles_list, get_velocity_list, obsdata  # 引入自定义的 ObsData 消息
from std_msgs.msg import Header  # 引入标准的 Header
import numpy as np
import robolab
from std_msgs.msg import Float32MultiArray

class JacobianPublisher:
    def __init__(self):
        model_path = "/home/eto/ZLK/Robolab/assets/mjcf/alicia/Alicia_0624.xml"
        self.export_link = "Link6"

        self.robot = robolab.RobotModel(model_path, solve_engine="pytorch_kinematics", verbose=True)

        # Publisher for the obs_data dictionary
        self.pub_obs_data = rospy.Publisher('/robot_obs_data', obsdata, queue_size=10)

        self.joint_value = []
        self.joint_velocity = [0.0] * 6  # Initialize with default velocity

        # Initialize variables to store the data
        self.robot_eef_pose = [0,0,0,0,0,0]
        self.robot_eef_pose_vel = [0,0,0,0,0,0]
        self.robot_joint = [0,0,0,0,0,0]
        self.robot_joint_vel = [0.5,0.5,0.5,0.5,0.5,0.5]
        self.step_idx = 0

    def motor_angles_callback(self, data):
        data_list = list(data.data)  # Get the joint angles
        self.joint_value = data_list[:2] + data_list[3:]
        # rospy.loginfo(f"Published observation data: {self.joint_value}")
        # Compute the Jacobian
        J = self.robot.get_jacobian(self.joint_value, self.export_link)
        J_np = J.numpy()

        # Compute the end-effector velocity (linear + angular)
        velocity = J_np @ self.joint_velocity
        eef_velocity = velocity.flatten()

        # Get the forward kinematics (position and rotation)
        pos, rot, ret = self.robot.get_fk(self.joint_value, self.export_link)

        # Convert rotation from quaternion to Euler angles
        quat = rot[0].tolist()
        euler = robolab.convert_ori_format(quat, "quat", "euler")
        combined_tensor = torch.cat((pos, euler), dim=1)
        eef_pose_data = combined_tensor.squeeze().tolist()  # Flatten the tensor to a list

        # Update the variables with new data
        self.robot_eef_pose = eef_pose_data
        self.robot_eef_pose_vel = eef_velocity.tolist()
        self.robot_joint = self.joint_value
        self.robot_joint_vel = self.joint_velocity
        self.step_idx += 1  # Increment step index

    # def motor_velocity_callback(self, data):
    #     try:
    #         self.joint_velocity = list(data.velocity_list)  # Convert speed to a regular list
    #     except Exception as e:
    #         rospy.logerr(f"Error processing data: {e}")

    def publish_obs_data(self):
        """ Publishes the obs_data dictionary as an ObsData message. """
        obs_msg = obsdata()

        # Populate the message with the current data
        obs_msg.robot_eef_pose = self.robot_eef_pose
        obs_msg.robot_eef_pose_vel = self.robot_eef_pose_vel
        obs_msg.robot_joint = self.robot_joint
        obs_msg.robot_joint_vel = self.robot_joint_vel
        obs_msg.step_idx = self.step_idx

        # Set the header and timestamp
        obs_msg.header = Header()  # Create header
        obs_msg.header.stamp = rospy.Time.now()  # Set the current ROS time as the timestamp

        # Publish the message
        self.pub_obs_data.publish(obs_msg)
        
        # Print the published message for debugging
        rospy.loginfo(f"Published observation data: {obs_msg}")
        # rospy.loginfo(f"Published observation timestamp: {obs_msg.header.stamp}")

def main():
    rospy.init_node('get_obs', anonymous=True)

    # Initialize the JacobianPublisher class
    jacobian_publisher = JacobianPublisher()

    # Subscribe to motor angle and velocity topics
    rospy.Subscriber('servo_states_main', Float32MultiArray, jacobian_publisher.motor_angles_callback)
    # rospy.Subscriber('/motor_velocity_list', get_velocity_list, jacobian_publisher.motor_velocity_callback)

    rate = rospy.Rate(125)  # Set the rate to 125Hz
    while not rospy.is_shutdown():
        # Periodically publish the obs_data dictionary
        jacobian_publisher.publish_obs_data()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

