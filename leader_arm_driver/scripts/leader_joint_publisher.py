#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import math
import time

class ManipulatorJointPublisher:
    def __init__(self):
        rospy.init_node('manipulator_joint_publisher')
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(50)  # 50 Hz

        # Joint names must match your URDF
        self.joint_names = [
            'Joint01', 'Joint02', 'Joint03',
            'Joint04', 'Joint05', 'Joint06'
        ]

        # Initialize joint positions
        self.angle = 0.0

    def run(self):
        while not rospy.is_shutdown():
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = self.joint_names

            # Example: oscillate the first joint, keep others fixed
            js.position = [
                math.sin(self.angle),
                0.5,
                -0.3,
                0.2,
                -0.1,
                0.0
            ]

            self.pub.publish(js)

            self.angle += 0.05
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ManipulatorJointPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
