#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS节点：示教模式控制器

该节点用于控制机器人的示教模式，通过ROS服务提供示教模式的启动和停止功能。
发布示教模式的状态到/demonstration话题。
"""

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
import sys


class DemoModeController:
    """用于控制机器人示教模式的ROS节点类"""
    
    def __init__(self):
        """初始化示教模式控制器"""
        rospy.init_node('demo_mode_controller', anonymous=True)
        self.demo_pub = rospy.Publisher('/demonstration', Bool, queue_size=10)
        
        # 从ROS参数服务器获取初始参数
        self.demo_mode_enabled = rospy.get_param('~enable_demo', True)
        
        # 创建设置示教模式的服务
        self.set_demo_srv = rospy.Service('~set_demo_mode', SetBool, self.set_demo_callback)
        
        rospy.loginfo("示教模式控制器已初始化")
    
    def set_demo_callback(self, req):
        """
        处理设置示教模式的服务请求
        
        Args:
            req: SetBool服务请求，data字段为True表示启用示教模式，False表示禁用
            
        Returns:
            SetBoolResponse: 包含操作成功状态和描述信息的响应
        """
        self.demo_mode_enabled = req.data
        # 更新参数服务器上的值
        rospy.set_param('~enable_demo', self.demo_mode_enabled)
        self.publish_demo_state()
        
        return SetBoolResponse(
            success=True,
            message="示教模式已" + ("启动" if self.demo_mode_enabled else "停止")
        )
    
    def publish_demo_state(self):
        """发布当前示教模式的状态到相关话题"""
        # 等待发布者连接
        rospy.sleep(0.5)
        
        # 创建并设置消息
        demo_msg = Bool()
        demo_msg.data = self.demo_mode_enabled
        
        # 发布消息
        self.demo_pub.publish(demo_msg)
        
        # 记录状态变更
        if self.demo_mode_enabled:
            rospy.loginfo("示教模式已启动")
        else:
            rospy.loginfo("示教模式已停止")


def main():
    """主函数"""
    try:
        # 创建并初始化控制器
        controller = DemoModeController()
        
        # 初始发布当前状态
        controller.publish_demo_state()
        
        # 保持节点运行
        rospy.loginfo("示教模式控制器已启动，等待服务调用...")
        rospy.spin()
        
    except rospy.ROSInterruptException as e:
        rospy.logerr("示教模式控制器异常终止: %s", str(e))
    except Exception as e:
        rospy.logerr("示教模式控制器发生未预期异常: %s", str(e))


if __name__ == '__main__':
    main()
