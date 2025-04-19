#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import MultiArrayDimension
from frame_msgs.msg import set_servo_as
import numpy as np
import math


class MainNode:
    def __init__(self, initial_value=0.0):
        rospy.init_node('main_node')

        self.servo_control_num = 9
        self.msg = set_servo_as()

        dim = MultiArrayDimension()
        dim.size = self.servo_control_num
        dim.stride = 1

        self.msg.servo_target_angle.layout.dim.append(dim)
        self.msg.servo_target_cycle.layout.dim.append(dim)
        self.msg.servo_target_angle.data = np.zeros(self.servo_control_num, dtype=np.float32)

        self.servo_command_pub = rospy.Publisher('main_control_servo', set_servo_as, queue_size=10)

    def send_command(self, joint_radians, gripper_deg):
        if len(joint_radians) != 6:
            rospy.logerr("Expected 6 joint values")
            return

        # Convert radians to degrees
        deg = [math.degrees(angle) for angle in joint_radians]

        # Map to servo command layout
        cmd = [0.0] * self.servo_control_num
        cmd[0] = deg[0]; cmd[1] = deg[0]
        cmd[2] = deg[1]; cmd[3] = -deg[1]
        cmd[4] = deg[2]; cmd[5] = -deg[2]
        cmd[6] = deg[3]; cmd[7] = deg[4]
        cmd[8] = deg[5]

        self.msg.servo_target_angle.data = cmd
        self.msg.servo_target_cycle.data = [1000.0] * self.servo_control_num
        self.msg.gripper_angle.data = float(gripper_deg)

        self.servo_command_pub.publish(self.msg)


def main():
    node = MainNode()
    rospy.loginfo("main_node running (command only)")
    rospy.spin()


if __name__ == '__main__':
    main()
