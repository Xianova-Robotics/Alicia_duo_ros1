#! /usr/bin/env python3
#coding=utf-8

## 不是3.10
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int32

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
class SerialReaderNode:
    def __init__(self):
        rospy.init_node('read_serial_type_node') # 初始化节点
        
        # 用 self 表明它是类变量，而不是一个局部变量或全局变量
        self.pub_2 = rospy.Publisher('gripper_angle', Int32, queue_size=10)
        self.pub_4 = rospy.Publisher('servo_states', UInt8MultiArray, queue_size=10)
        self.pub_EE = rospy.Publisher('error_frame_deal', UInt8MultiArray, queue_size=10)
        
        # 订阅串口数据话题, 对于类中的回调函数要用 self 表明，且会自动传递 self
        self.sub = rospy.Subscriber('read_serial_data', UInt8MultiArray, self.serial_data_callback)

    def serial_data_callback(self, serial_msg):
        """
            @brief 串口数据回调函数

            @param serial_msg: 串口数据
        """
        command = serial_msg.data[1] # 指令id
        
        if command == 0x02:
            gripper_angle = serial_msg.data[4] | serial_msg.data[5] << 8
            self.pub_2.publish(gripper_angle)
        elif command == 0x04:
            self.pub_4.publish(serial_msg)
            # print_hex_frame(serial_msg.data)
        elif command == 0xEE:
            self.pub_EE.publish(serial_msg)
            print("%2X 有话题，但暂时无接收处理\n")
        else:
            print("暂无该指令id %2X 的功能\n", command)

## 接收订阅串口数据话题，并判断指令id，准确为对应话题发布数据
def main():
    try:
        node = SerialReaderNode()
        print("read_serial_type_node play")
        rospy.spin() # 保持节点运行
    except rospy.ROSInterruptException: # 捕获crtl+c异常
        pass

if __name__ == '__main__':
    main()
