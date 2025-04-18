#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class VisionManager:
    def __init__(self, length, breadth):
        self.table_length = length
        self.table_breadth = breadth
        self.bridge = CvBridge()

        # 订阅输入视频流并发布处理后的图像
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.imageCb)
        self.image1_pub = rospy.Publisher('/table_detect', Image, queue_size=1)
        self.image2_pub = rospy.Publisher('/object_detect', Image, queue_size=1)
        self.position_pub = rospy.Publisher('/detected_object_position', Point, queue_size=1)

        # 像素到毫米的转换比例
        self.pixels_permm_x = None
        self.pixels_permm_y = None

        # 增加一个计数器，用于处理连续检测失败的情况
        self.failure_count = 0

    def get2DLocation(self, msg):
        tablePos = self.detectTable(msg)
        if tablePos is None:
            self.failure_count += 1
            if self.failure_count >= 5:
                rospy.logwarn("Table not detected for 5 consecutive frames!")
                return None, None
        else:
            self.failure_count = 0

        pixel_x, pixel_y = self.detect2DObject(msg, tablePos)
        if pixel_x == 0 and pixel_y == 0:
            self.failure_count += 1
            if self.failure_count >= 5:
                rospy.logwarn("Object not detected for 5 consecutive frames!")
                return None, None
        else:
            self.failure_count = 0

        x, y = self.convertToMM(pixel_x, pixel_y, msg.width, msg.height)
        return x, y

    def detectTable(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge exception: %s", str(e))
            return None

        # 转换为灰度图
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 高斯模糊减少噪声
        gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # 自适应光照补偿
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_image = clahe.apply(gray_image)

        # 动态调整阈值
        _, binary_image = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 调试：显示二值化图像
        cv2.imshow("Binary Image", binary_image)
        cv2.waitKey(1)

        # 查找轮廓
        contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 找到最大轮廓（假设桌面是图像中最大的白色区域）
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            bbox = cv2.boundingRect(largest_contour)

            # 计算中心点
            center_x = bbox[0] + bbox[2] // 2
            center_y = bbox[1] + bbox[3] // 2
            center = (center_x, center_y)

            # 在图像上绘制边框和中心点
            cv2.rectangle(cv_image, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 2)
            cv2.circle(cv_image, center, 4, (0, 0, 255), -1)

            # 计算像素到毫米的转换比例
            self.pixels_permm_y = bbox[3] / self.table_length
            self.pixels_permm_x = bbox[2] / self.table_breadth

            # 发布处理后的图像
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image1_pub.publish(ros_image)

            return bbox
        else:
            rospy.logwarn("No table detected in the image!")
            return None

    def detect2DObject(self, msg, tablePos):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge exception: %s", str(e))
            return 0, 0

        # 将 BGR 图像转换为 HSV 颜色空间
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 深绿色HSV阈值
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        binaryImage = cv2.inRange(hsv_image, lower_green, upper_green)

        # 限制检测区域在桌子边界内
        mask = np.zeros_like(binaryImage)
        x1, y1, w, h = tablePos
        mask[y1 + 3:y1 + h - 3, x1 + 3:x1 + w - 3] = 255
        binaryImage = cv2.bitwise_and(binaryImage, mask)

        # 膨胀操作
        kernel = np.ones((3, 3), np.uint8)
        binaryImage = cv2.dilate(binaryImage, kernel)
        # 新增腐蚀操作
        binaryImage = cv2.erode(binaryImage, kernel)

        # 查找非零像素点
        nonZeroPoints = cv2.findNonZero(binaryImage)
        if nonZeroPoints is None:
            rospy.logwarn("No object detected on the table!")
            return 0, 0

        # 轮廓筛选
        min_contour_area = 10
        valid_contours = []
        contours, _ = cv2.findContours(binaryImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_contour_area:
                valid_contours.append(contour)
        if valid_contours:
            bbox = cv2.boundingRect(valid_contours[0])
            pixel_x = bbox[0] + bbox[2] // 2
            pixel_y = bbox[1] + bbox[3] // 2
            pt = (pixel_x, pixel_y)
            cv2.circle(cv_image, pt, 4, (0, 0, 255), -1)

            # 绘制筛选后的轮廓（用于调试）
            cv2.drawContours(cv_image, valid_contours, -1, (255, 0, 0), 3)
        else:
            rospy.logwarn("No valid object contour detected on the table!")
            return 0, 0

        # 发布处理后的图像
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image2_pub.publish(ros_image)

        return pixel_x, pixel_y

    def convertToMM(self, x, y, img_width, img_height):
        if self.pixels_permm_x is None or self.pixels_permm_y is None:
            rospy.logerr("Pixel to mm conversion ratio not set!")
            return 0, 0

        # 动态计算图像中心点
        img_centre_x = img_width // 2
        img_centre_y = img_height // 2

        # 从像素坐标转换为世界坐标
        x = (x - img_centre_x) / self.pixels_permm_x
        y = (y - img_centre_y) / self.pixels_permm_y
        return x, y

    def imageCb(self, msg):
        x, y = self.get2DLocation(msg)

        if x is not None and y is not None:
            # 创建并发布位置消息
            position_msg = Point()
            position_msg.x = x
            position_msg.y = y
            position_msg.z = 0.5351145270620465  # 需要测量摄像头距离物体高度
            self.position_pub.publish(position_msg)


if __name__ == '__main__':
    try:
        # 添加节点初始化
        rospy.init_node('vision_manager_node', anonymous=True)

        rospy.loginfo("Starting Vision Manager...")
        # 识别的桌子尺寸单位为米
        length = rospy.get_param("~table_length", 0.34)
        breadth = rospy.get_param("~table_breadth", 0.315)

        vm = VisionManager(length, breadth)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass