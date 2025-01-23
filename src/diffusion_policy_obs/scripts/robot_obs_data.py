#!/usr/bin/env python
# 使用ROS，将从机器人硬件收集到的数据转化为模型需要的数据格式，使得其能够用于模型的训练与验证
import rospy
import torch
from diffusion_policy_obs.msg import obsdata  # 引入自定义的 ObsData 消息
from std_msgs.msg import Header  # 引入标准的 Header
import numpy as np
from std_msgs.msg import Float32MultiArray

class JacobianPublisher:
    def __init__(self):
        self.pub_obs_data = rospy.Publisher('/robot_obs_data', obsdata, queue_size=10)

        self.joint_value = []
        self.joint_velocity = [0.0] * 6  


        self.robot_eef_pose = [0,0,0,0,0,0]
        self.robot_eef_pose_vel = [0,0,0,0,0,0]
        self.robot_joint = [0,0,0,0,0,0]
        self.joint_angle = [0,0,0,0,0,0,0]
        self.robot_joint_vel = [0.5,0.5,0.5,0.5,0.5,0.5]
        self.step_idx = 0

    def motor_angles_callback(self, data):
        data_list = list(data.data)  

        self.joint_angle = data_list[:2] + data_list[3:]
        self.joint_value = data_list[:2] + data_list[3:7]

    def publish_obs_data(self):
        """ Publishes the obs_data dictionary as an ObsData message. """
        obs_msg = obsdata()


        obs_msg.robot_eef_pose = self.robot_eef_pose
        obs_msg.robot_eef_pose_vel = self.robot_eef_pose_vel
        obs_msg.robot_joint = self.joint_angle
        obs_msg.robot_joint_vel = self.robot_joint_vel

        obs_msg.step_idx = self.step_idx


        obs_msg.header = Header()  
        obs_msg.header.stamp = rospy.Time.now()  


        self.pub_obs_data.publish(obs_msg)
        

        rospy.loginfo(f"Published observation data: {obs_msg}")

def main():
    rospy.init_node('get_obs', anonymous=True)

    jacobian_publisher = JacobianPublisher()


    rospy.Subscriber('servo_states_main', Float32MultiArray, jacobian_publisher.motor_angles_callback)


    rate = rospy.Rate(125)  
    while not rospy.is_shutdown():

        jacobian_publisher.publish_obs_data()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

