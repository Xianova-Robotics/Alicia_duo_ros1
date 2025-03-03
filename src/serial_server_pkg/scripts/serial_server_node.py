#! /usr/bin/env python3
#coding=utf-8

## 不是3.10
import rospy
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
import serial                       # 导入pyserial库
import serial.tools.list_ports      # 列出所有串口
import time
import sys
import numpy as np
import threading

ser = None

def find_serial_ports():
    """
        @brief 获取所有可用的串口信息函数

        @return 返回包含所有可用串口名称的列表的第一个
    """
    return [port.device for port in serial.tools.list_ports.comports()][0]

def monitor_serial_ports():
    """
        @brief monitor监控串口设备, 当有设备插入时, 自动识别并返回串口设备列表

        @return 串口设备列表
    """
    ## 捕获Ctrl+C
    try:
        ser_ports = find_serial_ports()
        
        if ser_ports:
            print(f"选择的端口：{ser_ports}\n")
        else:
            while not rospy.is_shutdown():
                print("没有任何串口设备，请插入设备...\n")
                time.sleep(1)

                # 重新查找串口设备
                ser_ports = find_serial_ports()
                if ser_ports:
                    print(f"选择的端口：{ser_ports}\n")
                    for port in ser_ports:
                        print(port)
                    break
        return ser_ports
    except KeyboardInterrupt:
        print("程序退出\n")
        sys.exit(0)

def sum_elements(int_array_data):
    """
        @brief 将一个整数数组中的数据, 按(玄雅科技协议)数据范围进行求和

        @param int_array_data 整数数组
        @return sum
    """
    # 确保数组至少有4个元素，并且最后一个元素不是要加的范围之内
    if len(int_array_data) < 4:
        raise ValueError("Array must contain at least 4 elements.\n")
    
    # 假设len(int_array_data)=10，则int_array_data[-2]==int_array_data[8]，其中包含关系是【 ）即 [start:end]
    sub_array = int_array_data[3:-2]
    
    # 计算并返回子数组中所有元素的和
    return sum(sub_array)

def serial_data_check(serial_data):
    """
        @brief 计算一段整数串口数据帧的校验位是否正确

        @param serial_data 整数串口数据
        @return True or False
    """
    check = sum_elements(serial_data) % 2
    if(serial_data[len(serial_data)-2] == check):
        return True
    else:
        return False
    
def print_hex_frame(frame_msg, num):
    """
        @brief 将整数数组转换为十六进制字符串并打印

        @param frame_msg 整数数组
    """
    if num == 0:
        hex_output = "发送数据:"
    elif num == 1:
        hex_output = "数据接收:"
    else:
        hex_output = "接收数据的一部分:"
    # 遍历 frame_msg 数组，并将每个 byte_int 转换为十六进制字符串格式
    for byte_int in frame_msg:
        hex_str = hex(byte_int)[2:].upper().zfill(2)

        if hex_output:  # 如果 hex_output 不是空字符串
            hex_output += " " + hex_str
        else:           # 无字符时，不添加开头空格
            hex_output += hex_str
        
    # 打印最终的输出，此时不需要再使用 lstrip() 去掉开头的空格，已经在循环中处理了
    print(hex_output)

def read_frame(ser, pub):
    """
        @brief 读取串口帧数据 -- 玄雅科技协议

        @param ser 串口对象
        @param pub Topic话题对象
        @return None
    """
    msg = UInt8MultiArray()                                         ## 定义一个多数组消息
    dim = MultiArrayDimension()                                     ## 定义一个数组维度
    dim.stride = 1                                                  ## 数组只有一个维度
    frame_msg = []                                                  # 整数数组，存储由接收到的“bytearray”字节数组转化成的整数
    state = True                                                    # 是否开始出现有需要的帧
    msg.layout.dim.append(dim)

    try:
        while not rospy.is_shutdown():

            if ser.in_waiting > 0:
                byte = ser.read(1)                                  # 从串行端口读取一个字节
            else:
                continue

            # 检查是否成功读取
            if byte is None:
                print("未能从串行端口读取数据..\n")
                continue                                            # 继续等待下一个字节
            
            byte_int = int.from_bytes(byte, byteorder='big')        # 将字节转换为整数, 若是 uint8_t 则多加一个 & 0xFF
            # print(hex(byte_int)[2:].upper().zfill(2))               # 以16进制格式打印，去掉'0x'前缀且有两位
            
            if state:
                if byte_int == 0xAA:                                # 检查帧开始标志
                    frame_msg.clear()                               # 清空之前的帧内容
                    frame_msg.append(byte_int)
                    state = False
            elif state == False:
                frame_msg.append(byte_int)
                if byte_int == 0xFF and len(frame_msg) >= 3:        # 确保我们有至少3个字节，且检查到帧结束标志
                    expected_length = frame_msg[2] + 5              # 数据长度+5等于帧长度
                    if len(frame_msg) == expected_length:           # 检查是否已经收到了完整的帧
                        if serial_data_check(frame_msg):            # 如果校验位正确，则认为帧接收完成
                            dim.size = len(frame_msg)               ## 数组大小为帧长度
                            msg.data = np.array(frame_msg, dtype=np.uint8) # NumPy 数组
                            msg.data = msg.data.tolist()            # 将 NumPy 数组转换为列表

                            print_hex_frame(msg.data, 1)

                            pub.publish(msg)                        ## /* 完整帧Topic传递 */
                            state = True
                            # ser.reset_input_buffer()
                    elif expected_length > 64 or len(frame_msg) > 64:    # 如果帧长度超过64字节，则认为帧错误
                        state = True
    except KeyboardInterrupt:
        if ser and ser.is_open:
            ser.close()
        print("程序退出ed.\n")
        sys.exit(0)
    finally:
        ser.close()  # 确保串口关闭
# 以下相隔20行





















def send_serial_data_callback(serial_data):
    global ser

    byte_data = bytearray(serial_data.data)

    try:
        ser.write(byte_data)
        print_hex_frame(serial_data.data, 0)
    except serial.SerialException as e:
        print(f"串口发送数据失败: {e}")
        sys.exit(1)

## 接收串口数据后发布
def main():
    try:
        global ser

        ## ros init
        rospy.init_node('serial_server_node')

        ## serial init
        ser_ports = monitor_serial_ports()
        ser_baudrate = 921600
        ser = serial.Serial(ser_ports, ser_baudrate, timeout=1)

        sub = rospy.Subscriber('send_serial_data', UInt8MultiArray, send_serial_data_callback)
        pub_r = rospy.Publisher('read_serial_data', UInt8MultiArray, queue_size=10)

        ## read_frame
        # 创建并启动读取串口的线程
        read_thread = threading.Thread(target=read_frame, args=(ser, pub_r))
        read_thread.daemon = True  # 设置为守护线程，这样主程序结束时线程也会自动结束
        read_thread.start()

        rospy.spin()

    finally:
        time.sleep(1)
        if ser is not None:
            if ser.is_open:
                ser.close()

if __name__ == '__main__':
    # try:
        main()
    # finally:
    #     if ser is not None:
    #         if ser.is_open:
    #             ser.close()
    #     print("serial_server_node exit")
    #     sys.exit(0)
