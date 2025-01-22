#! /usr/bin/env python3
#coding=utf-8

## 不是3.10
import rospy
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from frame_msgs.msg import set_servo_as
import time

def Hxj_angle_scale(servo_angle, scale):
    if scale:
        return servo_angle * 10.0
    else:
        return servo_angle / 10.0

def float_to_u8_array(float_array, u8_array):
    for i in range(len(float_array)):
        # 将浮点值转化为有符号整数（16 位）
        int_value = int(Hxj_angle_scale(float_array[i], 1))
        
        # 确保整数在 -32768 到 32767 的范围内
        if int_value < -32768 or int_value > 32767:
            raise ValueError("Float value is out of the 16-bit signed integer range.")
        
        # 对于负数，确保转换时正确处理
        if int_value < 0:
            int_value += 0x10000  # 处理负数的有符号到无符号的转换
        
        # 拆分为高 8 位和低 8 位
        u8_array[3+2*i] = int_value & 0xFF
        u8_array[3+2*i+1] = (int_value >> 8) & 0xFF

def float_v_to_u8_array(float_value, u8_array):
    int_value = int(float_value)

    if int_value < -32768 or int_value > 32767:
        raise ValueError("Float value is out of the 16-bit signed integer range.")

    if int_value < 0:
        int_value += 0x10000

    u8_array[3+1] = int_value & 0xFF
    u8_array[3+1+1] = (int_value >> 8) & 0xFF

def sum_elements(int_array_data):
    """
        @brief 将一个整数数组中的数据, 按(玄雅科技协议)数据范围进行求和

        @param int_array_data 整数数组
        @return sum
    """
    # 确保数组至少有4个元素，并且最后一个元素不是要加的范围之内
    if len(int_array_data) < 4:
        raise ValueError("Array must contain at least 4 elements.\n")
    
    # 递归地将嵌套列表展平成一个扁平的列表
    def flatten(lst):
        for item in lst:
            if isinstance(item, list):
                yield from flatten(item)
            else:
                yield item

    return sum(flatten(int_array_data[3:-2]))

def print_hex_frame(frame_msg):
    """
        @brief 将整数数组转换为十六进制字符串并打印

        @param frame_msg 整数数组
    """
    hex_output = ""
    
    # 遍历 frame_msg 数组，并将每个 byte_int 转换为十六进制字符串格式
    for byte_int in frame_msg:
        hex_str = hex(byte_int)[2:].upper().zfill(2)

        if hex_output:  # 如果 hex_output 不是空字符串
            hex_output += " " + hex_str
        else:           # 无字符时，不添加开头空格
            hex_output += hex_str
        
    # 打印最终的输出，此时不需要再使用 lstrip() 去掉开头的空格，已经在循环中处理了
    print(hex_output)

## 命名空间
class ServoControlNode:
    def __init__(self):
        rospy.init_node('servo_control_node') # 初始化节点

        self.servo_control_num      = 7
        self.servo_control_id_min   = 0
        self.servo_control_id_max   = 7

        self.servo_control_pkg = UInt8MultiArray()
        self.dim = MultiArrayDimension()                                                            ## 定义一个数组维度
        self.dim.size    = self.servo_control_num * 2 * 2 + 5                                       ## 数组长度
        self.dim.stride  = 1                                                                        ## 数组只有一个维度
        self.dim.size    = self.servo_control_num * 2 + 5                                           ## 数组长度
        self.servo_control_pkg.layout.dim.append(self.dim)

        self.servo_angle_data = [0] * self.dim.size 
        self.servo_cycle_data = [0] * self.dim.size 
        self.servo_angle_data[0] = 0xAA
        self.servo_cycle_data[0] = 0xAA
        self.servo_angle_data[1] = 0x04
        self.servo_cycle_data[1] = 0x04
        self.servo_angle_data[2] = self.dim.size - 5
        self.servo_cycle_data[2] = self.dim.size - 5
        self.servo_angle_data[self.dim.size - 1] = 0xFF
        self.servo_cycle_data[self.dim.size - 1] = 0xFF

        self.gripper_angle = 0
        self.gripper_angle_data = [0] * (5 + 3) 
        self.gripper_angle_data[0] = 0xAA
        self.gripper_angle_data[1] = 0x02
        self.gripper_angle_data[2] = 0x03
        self.gripper_angle_data[3] = 0x01      ## 默认
        self.gripper_angle_data[5 + 3 - 1] = 0xFF
        
        # 以一定频率订阅串口数据话题
        self.rate = 1000.0  # 限定1s几次

        self.sub = rospy.Subscriber('main_control_servo', set_servo_as, self.maxf_to_min8_2callback)

        self.pub= rospy.Publisher('send_serial_data', UInt8MultiArray, queue_size=10)    ## main_pkg 中整合

    def maxf_to_min8_2callback(self, servo_control_msg):
        """
            @brief 从main_pkg中接收到的舵机浮点数据转换为舵机控制数据

            @param servo_control_msg: 舵机控制数据结构体
        """
        ### 写的暂时是只有角度，没有周期!!!!!!
        float_to_u8_array(servo_control_msg.servo_target_angle.data, self.servo_angle_data)
        float_to_u8_array(servo_control_msg.servo_target_cycle.data, self.servo_cycle_data)
        float_v_to_u8_array(servo_control_msg.gripper_angle.data, self.gripper_angle_data)

        ## 开发观测位 -- u8
        servo_control_msg.servo_target_angle_u8.layout.dim.append(self.dim)
        servo_control_msg.servo_target_cycle_u8.layout.dim.append(self.dim)
        servo_control_msg.servo_target_angle_u8.data = self.servo_angle_data
        servo_control_msg.servo_target_cycle_u8.data = self.servo_cycle_data

        # servo_control_msg.gripper_angle_u8.layout.dim.append(self.dim)
        # servo_control_msg.gripper_angle_u8.layout.dim.size = 5 + 3
        servo_control_msg.gripper_angle_u8.data = self.gripper_angle_data

        ## 检验位填写
        servo_control_msg.servo_target_angle_u8.data[self.dim.size - 2] = sum_elements(self.servo_angle_data) % 2
        servo_control_msg.servo_target_cycle_u8.data[self.dim.size - 2] = sum_elements(self.servo_cycle_data) % 2
        servo_control_msg.gripper_angle_u8.data[(5+3) - 2]              = sum_elements(self.gripper_angle_data) % 2

        # time.sleep(0.002)
        self.pub.publish(servo_control_msg.servo_target_angle_u8)
        # time.sleep(0.002)
        self.pub.publish(servo_control_msg.gripper_angle_u8)

        # self.servo_angle_data[self.dim.size - 2] = sum_elements(self.servo_angle_data) % 2
        # self.servo_cycle_data[self.dim.size - 2] = sum_elements(self.servo_cycle_data) % 2

        # servo_control_data = []
        # servo_control_data.append(0xAA)
        # servo_control_data.append(0x05)
        # servo_control_data.append(self.dim.size)
        # for i in range(self.servo_control_num):
        #     servo_control_data.append(self.servo_angle_data[3+2*i])
        #     servo_control_data.append(self.servo_angle_data[3+2*i+1])
        #     servo_control_data.append(self.servo_cycle_data[3+2*i])
        #     servo_control_data.append(self.servo_cycle_data[3+2*i+1])
        # servo_control_data.append((self.servo_angle_data[self.dim.size - 2] + self.servo_cycle_data[self.dim.size - 2]) % 2)
        # servo_control_data.append(0xFF)

        # self.servo_control_pkg.data = servo_control_data
        # self.pub.publish(servo_control_msg.self.servo_control_pkg.data)

## 接收订阅舵机姿态话题数据
def main():
    try:
        node = ServoControlNode()
        print("servo_control_node play")
        rospy.spin()                    # 保持节点运行
    except rospy.ROSInterruptException: # 捕获crtl+c异常
        pass

if __name__ == '__main__':
    main()