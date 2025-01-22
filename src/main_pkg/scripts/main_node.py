#! /usr/bin/env python3
#coding=utf-8

## 不是3.10                      
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from frame_msgs.msg import set_servo_as
import time
import sys
import threading

## 命名空间
class MainNode:
    def __init__(self, initial_value=0.0):
        rospy.init_node('main_node') # 初始化节点

        self.servo_control_num      = 7
        self.servo_control_id_min   = 0
        self.servo_control_id_max   = 7

        self.msg = set_servo_as()                                                                   ## 定义一个多数组消息
        self.dim = MultiArrayDimension()                                                            ## 定义一个数组维度
        self.dim.size    = self.servo_control_num                                                   ## 数组长度
        self.dim.stride  = 1                                                                        ## 数组只有一个维度
        self.msg.servo_target_angle.layout.dim.append(self.dim)
        self.msg.servo_target_cycle.layout.dim.append(self.dim)

        self.servo_angle_data = [initial_value] * (self.servo_control_num + 1)
        
        # 以一定频率订阅串口数据话题
        self.sub_states = rospy.Subscriber('servo_states_main', Float32MultiArray, self.main_servo_states_callback)

        self.pub= rospy.Publisher('main_control_servo', set_servo_as, queue_size=10)    ## main_pkg 中整合

        ## 测试用
        self.servo_target_angle     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_target_angle_2   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_target_cycle     = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
        self.gripper_angle1         = 0.0
        self.gripper_angle2         = 100.0

    def main_servo_states_callback(self, servo_angle_msg):               
        """
            @brief 舵机状态数据回调函数

            @param servo_angle_msg: 舵机状态数据
        """
        if len(servo_angle_msg.data) != self.servo_control_num + 1:
            print("main中接收的舵机数量与设定的舵机数量不一致!!\n")
            return 1
        
        self.servo_angle_data = servo_angle_msg.data

        ## 显示角度
        for i in range(self.servo_control_num + 1):
            print(f"---->当前{i + 1}号电机角度为: {self.servo_angle_data[i]:.2f}")

    def main_control_servo(self, servo_angle_buf, servo_angle_cycle, gripper_angle):
        """
            @brief 发送控制舵机数据

            @param servo_angle_buf: 舵机角度数据Float32MultiArray类型
            @param servo_angle_cycle: 舵机周期数据Float32MultiArray类型
        """
        for i in range(self.servo_control_id_min, self.servo_control_id_max):
            if servo_angle_buf[i] > 180.0 or servo_angle_buf[i] < -180.0:
                print("舵机角度超出范围!!\n")
                return 1

        self.msg.servo_target_angle.data = servo_angle_buf
        self.msg.servo_target_cycle.data = servo_angle_cycle
        self.msg.gripper_angle.data      = gripper_angle

        self.pub.publish(self.msg)
    
    def dis_all_servo_angle(self):
        """
            @brief 显示所有舵机角度
        """
        ## 显示角度
        for i in range(self.servo_control_num):
            print(f"{i + 1}, {self.msg.data[i]:.2f}")

def demo_thread(node):
    try:
        while not rospy.is_shutdown():
            # node.main_control_servo(node.servo_target_angle, node.servo_target_cycle, node.gripper_angle1)
            # time.sleep(2)
            # node.main_control_servo(node.servo_target_angle_2, node.servo_target_cycle, node.gripper_angle2)
            time.sleep(2)
    except KeyboardInterrupt:
        print("程序退出ed.\n")
        sys.exit(0)

## 接收订阅舵机姿态话题数据
def main():
    try:
        node = MainNode()
        print("main_node play")

        # 创建并启动读取串口的线程
        read_thread = threading.Thread(target=demo_thread, args=(node,))
        read_thread.daemon = True  # 设置为守护线程，这样主程序结束时线程也会自动结束
        read_thread.start()

        rospy.spin()                            # 保持节点运行
    except rospy.ROSInterruptException:         # 捕获crtl+c异常
        pass

if __name__ == '__main__':
    main()
