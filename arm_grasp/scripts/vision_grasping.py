#!/usr/bin/env python
import rospy
import cv2
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
# 导入 VisionManager 类
from vision_manager import VisionManager
import numpy as np
from tf.transformations import quaternion_matrix


# 定义 GraspingDemo 类，用于处理机械臂抓取相关操作
class GraspingDemo:
    def __init__(self, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth):
        # 修改为与 vision_manager.py 相同的相机话题
        self.it_ = rospy.Subscriber("/usb_cam/image_raw", Image, self.imageCb)
        # 初始化 MoveIt! 机械臂组和夹爪组
        self.armgroup = moveit_commander.MoveGroupCommander("manipulator")
        # self.grippergroup = moveit_commander.MoveGroupCommander("hand")

        # 初始化视觉管理器，使用正确的 VisionManager 类
        self.vMng_ = VisionManager(length, breadth)

        # 直接使用手眼标定结果创建变换矩阵
        # 平移部分
        """
        translation: 
        x: 0.32655039819919396
        y: -0.12770339265371675
        z: 0.5751145270620465
        rotation: 
        x: -0.693357432462034
        y: -0.008234105027763117
        z: 0.7001609310875454
        w: 0.17018325693989253
        """

        trans = [0.32655039819919396, -0.12770339265371675, 0.5751145270620465]#x,y,z
        # 旋转部分（四元数） rot = [x, y, z, w]

        rot = [-0.693357432462034, -0.008234105027763117, 0.7001609310875454, 0.17018325693989253]

        # 创建从相机到机器人基座的变换矩阵
        self.camera_to_robot_ = self.fromTranslationRotation(trans, rot)

        rospy.loginfo("Using fixed camera to robot transformation matrix")

        # 初始化 obj_robot_frame 为 None
        self.obj_robot_frame = None
        self.grasp_running = False
        # 存储预抓取位置的坐标
        self.pregrasp_x = pregrasp_x
        self.pregrasp_y = pregrasp_y
        self.pregrasp_z = pregrasp_z

        # 初始化图像桥接器
        self.bridge = CvBridge()

        # 让机械臂到达初始位置
        rospy.loginfo("Getting into the Grasping Position....")
        # self.attainPosition(pregrasp_x, pregrasp_y, pregrasp_z)

    # 添加方法将平移和旋转转换为变换矩阵
    def fromTranslationRotation(self, trans, rot):
        """
        根据平移向量和四元数创建4x4变换矩阵
        """
        # 创建旋转矩阵
        mat = quaternion_matrix(rot)
        # 设置平移部分
        mat[0:3, 3] = trans
        return mat

    # 图像回调函数，处理相机图像
    def imageCb(self, msg):
        if not self.grasp_running:
            rospy.loginfo("Processing the Image to locate the Object...")
            try:
                # 将 ROS 图像消息转换为 OpenCV 图像
                cv_ptr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                rospy.logerr("cv_bridge exception: %s", str(e))
                return

            # 获取物体在相机坐标系下的 2D 位置
            obj_x, obj_y = self.vMng_.get2DLocation(msg)

            # 检查是否成功获取坐标
            if obj_x is None or obj_y is None:
                return

            # 创建物体在相机坐标系下的坐标
            obj_camera = np.array([obj_x, obj_y, 0.52, 1.0])

            # 使用变换矩阵将坐标从相机坐标系转换到机器人坐标系
            obj_robot = np.dot(self.camera_to_robot_, obj_camera)

            # 创建机器人坐标系下的点
            obj_robot_point = geometry_msgs.msg.Point()
            obj_robot_point.x = obj_robot[0]
            obj_robot_point.y = obj_robot[1]
            obj_robot_point.z = obj_robot[2]

            # 只在实际需要时打印坐标信息
            rospy.loginfo("Object detected in robot frame:")
            rospy.loginfo(" X: %f, Y: %f, Z: %f",
                          obj_robot_point.x,
                          obj_robot_point.y,
                          obj_robot_point.z)

            # 更新物体位置
            self.obj_robot_frame = obj_robot_point
            self.grasp_running = True

    # 让机械臂到达指定位置
    def attainPosition(self, x, y, z):
        # 获取当前机械臂的位姿
        currPose = self.armgroup.get_current_pose()
        # 创建目标位姿
        target_pose1 = geometry_msgs.msg.Pose()
        target_pose1.orientation = currPose.pose.orientation
        target_pose1.position.x = x
        target_pose1.position.y = y
        target_pose1.position.z = z
        # 设置目标位姿
        self.armgroup.set_pose_target(target_pose1)
        # 执行运动规划，添加wait=True参数并设置planning_time
        self.armgroup.go(wait=True)
        # 确保没有残留目标
        self.armgroup.clear_pose_targets()

    # 让机械臂靠近物体
    def attainObject(self):
        # 让机械臂移动到物体上方一定距离处
        self.attainPosition(self.obj_robot_frame.x, self.obj_robot_frame.y, self.obj_robot_frame.z + 0.1)
        rospy.sleep(1.0)
        # 打开夹爪
        # self.grippergroup.set_named_target("open")
        # self.grippergroup.go()

        # 获取当前机械臂的位姿
        currPose = self.armgroup.get_current_pose()
        target_pose1 = geometry_msgs.msg.Pose()
        target_pose1.orientation = currPose.pose.orientation
        target_pose1.position = currPose.pose.position
        # 让机械臂下降到物体抓取位置
        target_pose1.position.z = self.obj_robot_frame.z - 0.1
        self.armgroup.set_pose_target(target_pose1)
        self.armgroup.go()

    # 执行抓取操作
    def grasp(self):
        rospy.sleep(1.0)
        # 关闭夹爪
        # self.grippergroup.set_named_target("close")
        # self.grippergroup.go()

    # 提起物体
    def lift(self):
        # 获取当前机械臂的位姿
        currPose = self.armgroup.get_current_pose()
        target_pose1 = geometry_msgs.msg.Pose()
        target_pose1.orientation = currPose.pose.orientation
        target_pose1.position = currPose.pose.position

        # 随机选择在 Y 轴上移动的方向
        if random.randint(0, 1):
            target_pose1.position.y = target_pose1.position.y + 0.1
        else:
            target_pose1.position.y = target_pose1.position.y - 0.1

        # 设置目标位姿并执行运动规划
        self.armgroup.set_pose_target(target_pose1)
        self.armgroup.go()

        rospy.sleep(1.0)
        # 打开夹爪
        # self.grippergroup.set_named_target("open")
        # self.grippergroup.go()

        # 让机械臂上升一定距离
        target_pose1.position.z = target_pose1.position.z + 0.1
        self.armgroup.set_pose_target(target_pose1)
        self.armgroup.go()

    # 让机械臂回到初始位置
    def goHome(self):
        #使用预抓取位置作为初始位置
        self.attainPosition(self.pregrasp_x, self.pregrasp_y, self.pregrasp_z)

    # 添加方法检查是否已检测到物体
    def object_detected(self):
        return self.obj_robot_frame is not None

    # 修改 initiateGrasping 方法
    def initiateGrasping(self):
        rospy.sleep(3.0)
        # 记录初始位置
        self.homePose = self.armgroup.get_current_pose()

        # 检查是否已检测到物体
        if not self.object_detected():
            rospy.logwarn("No object detected yet. Waiting for object detection...")
            return

        rospy.loginfo("Approaching the Object....")
        # 靠近物体
        self.attainObject()

        rospy.loginfo("Attempting to Grasp the Object now..")
        # 执行抓取操作
        self.grasp()

        rospy.loginfo("Lifting the Object....")
        # 提起物体
        self.lift()

        rospy.loginfo("Going back to home position....")
        # 回到初始位置
        self.goHome()

        self.grasp_running = False


if __name__ == '__main__':
    try:
        # 初始化 ROS 节点
        rospy.init_node('simple_grasping')

        # 从参数服务器获取指定名称的参数，使用rospy.get_param
        length = rospy.get_param("probot_grasping/table_length", 0.34)
        breadth = rospy.get_param("probot_grasping/table_breadth", 0.315)
        pregrasp_x = rospy.get_param("probot_grasping/pregrasp_x", 0.20)
        pregrasp_y = rospy.get_param("probot_grasping/pregrasp_y", -0.17)
        pregrasp_z = rospy.get_param("probot_grasping/pregrasp_z", 0.28)

        # 创建 GraspingDemo 类的实例
        simGrasp = GraspingDemo(pregrasp_x, pregrasp_y, pregrasp_z, length, breadth)
        rospy.loginfo("Waiting for eight seconds..")
        rospy.sleep(8.0)

        # 等待对象被检测到
        rospy.loginfo("Waiting for object detection...")
        rate = rospy.Rate(1)  # 1Hz检查频率
        waiting_time = 0
        timeout = 30  # 30秒超时

        while not rospy.is_shutdown() and not simGrasp.object_detected():
            rate.sleep()
            waiting_time += 1
            if waiting_time > timeout:
                rospy.logerr("Timeout waiting for object detection!")
                break

        # 如果检测到物体，启动抓取流程
        if simGrasp.object_detected():
            print("Object detected")
            # while not rospy.is_shutdown():
                # rate = rospy.Rate(0.5)  # 每2秒尝试一次抓取
                # simGrasp.initiateGrasping()
                # rate.sleep()
        else:
            rospy.loginfo("No object detected, exiting...")

    except rospy.ROSInterruptException:
        pass