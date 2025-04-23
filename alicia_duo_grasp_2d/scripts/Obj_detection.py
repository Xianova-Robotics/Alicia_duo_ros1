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

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.imageCb)
        self.image1_pub = rospy.Publisher('/table_detect', Image, queue_size=1)
        self.image2_pub = rospy.Publisher('/object_detect', Image, queue_size=1)
        self.position_pub = rospy.Publisher('/detected_object_position', Point, queue_size=1)

        self.pixels_permm_x = None
        self.pixels_permm_y = None
        self.failure_count = 0

    def imageCb(self, msg):
        x, y = self.get2DLocation(msg)
        if x is None or y is None:
            return

        position_msg = Point()
        position_msg.x = x
        position_msg.y = y
        position_msg.z = 0.4955244933727863  # Set from calibration
        self.position_pub.publish(position_msg)

    def get2DLocation(self, msg):
        bbox = self.detectTable(msg)
        if bbox is None:
            return self._handle_failure("Table not detected")

        pixel_x, pixel_y = self.detect2DObject(msg, bbox)
        if pixel_x == 0 and pixel_y == 0:
            return self._handle_failure("Object not detected")

        self.failure_count = 0
        return self.convertToMM(pixel_x, pixel_y, msg.width, msg.height)

    def detectTable(self, msg):
        image = self._convert_to_cv(msg)
        if image is None:
            return None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)

        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cv2.imshow("Binary Image", binary)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logwarn("No contours found for table.")
            return None

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        center = (x + w // 2, y + h // 2)

        # Draw overlay and compute scale
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, center, 4, (0, 0, 255), -1)
        self.image1_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        self.pixels_permm_x = w / self.table_breadth
        self.pixels_permm_y = h / self.table_length

        return (x, y, w, h)

    def detect2DObject(self, msg, table_bbox):
        image = self._convert_to_cv(msg)
        if image is None:
            return 0, 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([35, 50, 50]), np.array([85, 255, 255]))

        x, y, w, h = table_bbox
        region_mask = np.zeros_like(mask)
        region_mask[y+3:y+h-3, x+3:x+w-3] = 255
        mask = cv2.bitwise_and(mask, region_mask)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > 10]

        if not valid:
            rospy.logwarn("No valid contours for object.")
            return 0, 0

        x, y, w, h = cv2.boundingRect(valid[0])
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        cv2.drawContours(image, valid, -1, (255, 0, 0), 2)
        self.image2_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        return cx, cy

    def convertToMM(self, x, y, width, height):
        if self.pixels_permm_x is None or self.pixels_permm_y is None:
            rospy.logerr("Conversion scale not available.")
            return 0, 0

        center_x = width // 2
        center_y = height // 2
        return (x - center_x) / self.pixels_permm_x, (y - center_y) / self.pixels_permm_y

    def _convert_to_cv(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return None

    def _handle_failure(self, reason):
        self.failure_count += 1
        if self.failure_count >= 5:
            rospy.logwarn(f"{reason} for 5 consecutive frames!")
        return None, None


if __name__ == '__main__':
    try:
        rospy.init_node('vision_manager_node')
        rospy.loginfo("Starting Vision Manager...")

        length = rospy.get_param("~table_length", 0.34)
        breadth = rospy.get_param("~table_breadth", 0.315)

        vm = VisionManager(length, breadth)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
