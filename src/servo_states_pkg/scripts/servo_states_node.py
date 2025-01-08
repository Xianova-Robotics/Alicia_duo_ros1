#! /usr/bin/env python3
#coding=utf-8

## 不是3.10
import rospy
from std_msgs.msg import UInt8MultiArray, Float32MultiArray, MultiArrayDimension
import struct
import numpy as np
import time

def u8_array_to_float(u8_array):
    if len(u8_array) != 2:
        raise ValueError("Array must contain exactly two elements")
    
    # 构造16位整数
    hex_value = u8_array[0] | (u8_array[1] << 8)
    int_value = hex_value if hex_value < 0x8000 else hex_value - 0x10000
    ## 等价于
    # if hex_value < 0x8000:
    #     int_value = hex_value
    # else:
    #     int_value = hex_value - 0x10000

    # 将有符号整数转换为浮点数
    float_value = float(int_value)
    return float_value

def Hxj_angle_scale(servo_angle, scale):
    if scale:
        return servo_angle * 10.0
    else:
        return servo_angle / 10.0

## 命名空间
class ServoStatesNode:
    def __init__(self):
        rospy.init_node('servo_states_node') # 初始化节点

        self.servo_control_num      = 7
        self.servo_control_id_min   = 0
        self.servo_control_id_max   = 7
        self.msg = Float32MultiArray()                                                              ## 定义一个多数组消息
        self.dim = MultiArrayDimension()                                                            ## 定义一个数组维度
        self.dim.size    = self.servo_control_num                                                   ## 数组长度
        self.dim.stride  = 1                                                                        ## 数组只有一个维度
        self.msg.layout.dim.append(self.dim)
        self.servo_angle_data = [0.0] * self.servo_control_num
        
        self.pub_main_f = rospy.Publisher('servo_states_main', Float32MultiArray, queue_size=10)    ## main_pkg 中整合
        # 以一定频率订阅串口数据话题
        self._last_call_time = 0
        self.rate_limit = 0.001  # s级间隔
        self.subscriber = rospy.Subscriber('servo_states', UInt8MultiArray, self.servo_states_callback)

    def servo_states_callback(self, servo_states_msg):
        """
            @brief 舵机角度数据回调函数

            @param serial_msg: 串口数据
        """
        ## 节流
        # print_hex_frame(servo_states_msg.data)
        current_time = time.time()
        if current_time - self._last_call_time >= self.rate_limit:
            self._last_call_time = current_time

            ## 处理数据 is not 是用于比较是否是相同的内存地址
            if servo_states_msg.data[2]/2 != self.servo_control_num:
                print("接收的舵机数量与设定的舵机数量不一致!!\n")
                return 1
            
            ## 存储舵机角度数据
            for i in range(self.servo_control_id_min, self.servo_control_id_max): ## 不包括终止数
                self.servo_angle_data[i] = Hxj_angle_scale(u8_array_to_float(servo_states_msg.data[3+i*2:3+i*2+2]), 0)

            self.msg.data = self.servo_angle_data

            ## 显示角度
            # for i in range(self.servo_control_num):
            #     print(f"{i + 1}, {self.msg.data[i]:.2f}")

            self.pub_main_f.publish(self.msg)

## 接收订阅舵机姿态话题数据
def main():
    try:
        node = ServoStatesNode()
        print("servo_states_node play")

        rospy.spin()        # 保持节点运行
    except rospy.ROSInterruptException: # 捕获crtl+c异常
        pass

if __name__ == '__main__':
    main()
