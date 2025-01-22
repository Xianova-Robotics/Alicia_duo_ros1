#! /usr/bin/env python3
#coding=utf-8

## 不是3.10
import rospy
from std_msgs.msg import UInt8MultiArray
import time

## 命名空间
class ErrorFramDealNode:
    def __init__(self):
        rospy.init_node('error_frame_deal_node') # 初始化节点

        # 以一定频率订阅串口数据话题
        self._last_call_time = 0
        self.rate_limit = 1.0  # 每秒最多调用次数的倒数，即最小间隔时间（秒）
        self.subscriber = rospy.Subscriber('error_frame_deal', UInt8MultiArray, self.error_frame_deal_callback)

    def error_frame_deal_callback(self, error_frame_deal_msg):
        """
            @brief 错误帧处理回调函数

            @param error_frame_deal_msg: 错误帧处理消息
        """
        current_time = time.time()
        if current_time - self._last_call_time >= self.rate_limit:
            self._last_call_time = current_time

            command = error_frame_deal_msg.data[3] # 错误号

            if command == 0x00:     ## 包头包尾或数据长度不对
                print("error: 发送的 error_frame_deal_msg.data[4] 帧不匹配!!\n")
            elif command   == 0x01:   ## 校验位不对
                print("error: 发送的帧检验位不匹配!!\n", error_frame_deal_msg.data[4])
            elif command == 0x02:   ## 模式不对
                print("error: 当前 F1 控制舵机模式不再允许范围, 请查看 F1 的模式按钮!!\n", error_frame_deal_msg.data[4])
            elif command == 0x03:   ## id不对
                print("error: 发送的帧中的 %d id不在可控范围!!\n", error_frame_deal_msg.data[4])

## 接收订阅 发送的数据错误话题 数据
def main():
    try:
        node = ErrorFramDealNode()
        print("error_frame_deal_node play")
        rospy.spin()        # 保持节点运行
    except rospy.ROSInterruptException: # 捕获crtl+c异常
        pass

if __name__ == '__main__':
    main()
