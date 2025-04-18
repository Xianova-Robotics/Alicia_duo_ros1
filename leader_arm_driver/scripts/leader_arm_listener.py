#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    rospy.loginfo("Received JointState:")
    for name, position in zip(msg.name, msg.position):
        rospy.loginfo(f"  {name}: {position:.4f}")

def main():
    rospy.init_node('leader_arm_listener')
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("Listening to /joint_states...")
    rospy.spin()

if __name__ == '__main__':
    main()
