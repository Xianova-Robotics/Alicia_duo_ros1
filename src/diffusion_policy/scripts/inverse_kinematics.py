#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray
from all_motor_control.msg import set_angles # 导入自定义的消息类型
import robolab

def main():
    # 初始化全局变量
    model_path = "/home/eto/ZLK/Robolab/assets/mjcf/alicia/Alicia_0624.xml"
    export_link = "Link6"
    cur_configs = [[-1.7613,  2.7469, -3.5611, -3.8847,  2.7940,  1.9055]]
    ee_pose = None
    ik_solution = None

    # 初始化 ROS 节点
    rospy.init_node('inverse_kinematics', anonymous=True)
    
    # 初始化机器人模型
    robot = robolab.RobotModel(model_path, solve_engine="pytorch_kinematics", verbose=True)
    
    # 创建发布者
    pub = rospy.Publisher('motor_control', set_angles, queue_size=10)
    
    # 定义回调函数
    def pose_callback(msg):
        nonlocal ee_pose, ik_solution
        action = msg.data
        rospy.loginfo(f"Received action: {action}")
        # 初始化一个6元素的姿态列表
        roboee_pose = [0, 0, 0.1, 0, 0, 0,1]
        # 将 action 的值放入前两个位置
        roboee_pose[:2] = action[:2]
        rospy.loginfo(f"Received eep: {roboee_pose}")
        # 更新 ee_pose
        ee_pose = roboee_pose
        calculate_ik()

    def calculate_ik():
        nonlocal ee_pose, ik_solution
        if ee_pose is not None:
            start_time = time.time()
            # 获取逆运动学解
            # ret = robot.get_ik(ee_pose, export_link)
            # rospy.loginfo("IK Solutions: %s", ret.solutions)
            rospy.loginfo('Time cost: %f', (time.time() - start_time))
            # 获取接近当前配置的逆运动学解
            ret_near = robot.get_ik(ee_pose, export_link, cur_configs=cur_configs)
            rospy.loginfo("IK Solutions Near Current Config: %s", ret_near.solutions)
            ik_solution = ret_near.solutions.tolist()[0][0]
            publish_solution()

    def publish_solution():
        if ik_solution is not None:
            # 创建 set_angles 消息实例
            solutions_msg = set_angles()
            solutions_msg.id = 6  # 电机ID
            solutions_msg.speed = 0.5  # 电机速度
            solutions_msg.angle_list = ik_solution  # 逆运动学解
            
            # 发布消息
            pub.publish(solutions_msg)
            rospy.loginfo("Published angles: %s", str(solutions_msg.angle_list))

    # 订阅 /catch_result 话题
    rospy.Subscriber('/get_action', Float32MultiArray, pose_callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass