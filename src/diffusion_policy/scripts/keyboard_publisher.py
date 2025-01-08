#!/usr/bin/env python

# import rospy
# from std_msgs.msg import String
# import sys
# import select

# def publish_key():
#     pub = rospy.Publisher('key_event', String, queue_size=10)
#     rospy.init_node('keyboard_publisher', anonymous=True)

#     rate = rospy.Rate(10)  # 10hz
#     while not rospy.is_shutdown():
#         # 读取一个字符
#         key = sys.stdin.read(1).lower()  # 直接读取一个字符
#         if key:
#             rospy.loginfo(f"Key pressed: {key}")
#             pub.publish(key)
#             if key == 'q':  # 如果按下了'q'键，退出程序
#                 break
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_key()
#     except rospy.ROSInterruptException:
#         pass

import rospy
from std_msgs.msg import String
import sys
import termios
import tty

def get_key():
    """获取键盘输入的一个字符，不需要回车"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)  # 保存当前设置
    try:
        tty.setraw(sys.stdin.fileno())  # 设置为原始模式，取消行缓冲
        ch = sys.stdin.read(1)  # 读取一个字符
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # 恢复终端设置
    return ch

def publish_key():
    pub = rospy.Publisher('key_event', String, queue_size=10)
    rospy.init_node('keyboard_publisher', anonymous=True)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        key = get_key()  # 获取按键，不需要回车
        if key:
            rospy.loginfo(f"Key pressed: {key}")
            pub.publish(key)
            if key == 'q':  # 如果按下了'q'键，退出程序
                break
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_key()
    except rospy.ROSInterruptException:
        pass

