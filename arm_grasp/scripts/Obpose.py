#!/usr/bin/env python
import rospy
import yaml
import numpy as np
from geometry_msgs.msg import Point
from tf.transformations import quaternion_matrix
import os
import tf2_ros
import geometry_msgs.msg
import tf_conversions

class ObjectPoseTransformer:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('object_pose_transformer', anonymous=True)

        # Load transformation from YAML
        transform_matrix = self.load_transform_from_yaml()
        self.transform_matrix = transform_matrix
        self.latest_pose = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/detected_object_position", Point, self.callback)
        rospy.loginfo("ObjectPoseTransformer initialized. Waiting for detected object position...")

    def load_transform_from_yaml(self):
        yaml_path = os.path.expanduser("~/.ros/easy_handeye/Alicia_usb_handeyecalibration_eye_on_base.yaml")

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                trans = [
                    data['transformation']['x'],
                    data['transformation']['y'],
                    data['transformation']['z']
                ]
                quat = [
                    data['transformation']['qx'],
                    data['transformation']['qy'],
                    data['transformation']['qz'],
                    data['transformation']['qw']
                ]
                rospy.loginfo("Loaded calibration transformation from YAML.")
                return self._create_transform(trans, quat)
        except Exception as e:
            rospy.logerr("Failed to load transformation from YAML: %s", str(e))
            rospy.signal_shutdown("Failed to load calibration data.")
            return np.identity(4)


    def publish_object_tf(self, obj_pos):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"  # or "base_link" if that’s your base
        t.child_frame_id = "detected_object"

        t.transform.translation.x = obj_pos[0]
        t.transform.translation.y = obj_pos[1]
        t.transform.translation.z = obj_pos[2]

        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


    def _create_transform(self, trans, rot):
        """Create 4x4 homogeneous transform matrix from translation and quaternion."""
        mat = quaternion_matrix(rot)
        mat[0:3, 3] = trans
        return mat

    # def callback(self, msg):
    #     """Transform the 3D point from camera frame to robot base frame."""
    #     obj_camera = np.array([msg.x, msg.y, msg.z, 1.0])
    #     # rospy.loginfo("obj_camera = [%.3f, %.3f, %.3f, %.1f]", msg.x, msg.y, msg.z, 1.0)

    #     obj_robot = np.dot(self.transform_matrix, obj_camera)
    #     self.latest_pose = obj_robot[:3]
    #     rospy.loginfo_throttle(1.0, "Camera frame: x=%.3f y=%.3f z=%.3f", msg.x, msg.y, msg.z)
    #     rospy.loginfo_throttle(1.0, "Robot frame:  x=%.3f y=%.3f z=%.3f", *obj_robot[:3])


    def callback(self, msg):
        """Transform the 3D point from camera optical frame to robot base frame."""
        # 1. 物体在相机光学帧中的坐标（齐次坐标）
        obj_optical = np.array([msg.x, msg.y, msg.z, 1.0])
        
        # 2. 光学帧到相机本体帧的变换矩阵（来自static_transform_publisher）
        # 参数: args="0.0 0.0 0.0 -1.570796 0 -1.570796 camera_link camera_rgb_optical_frame 100"
        # 含义: 欧拉角 [roll=-π/2, pitch=0, yaw=-π/2]（单位：弧度）
        R_optical_to_link = np.array([
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])  # 旋转矩阵 + 无平移
        
        # 3. 将物体从光学帧转换到相机本体帧
        obj_link = np.dot(R_optical_to_link, obj_optical)
        
        # 4. 从相机本体帧转换到机械臂基坐标系（使用标定的变换矩阵）
        obj_robot = np.dot(self.transform_matrix, obj_link)
        
        # 更新并记录结果
        self.latest_pose = obj_robot[:3]
        self.publish_object_tf(self.latest_pose)  # pubulish object tf

        # rospy.loginfo_throttle(1.0, "Optical frame: x=%.3f y=%.3f z=%.3f", msg.x, msg.y, msg.z)
        # rospy.loginfo_throttle(1.0, "Robot frame:  x=%.3f y=%.3f z=%.3f", *obj_robot[:3])
        
    def get_latest_position(self):
            """Returns the latest object position in robot frame."""
            return self.latest_pose


if __name__ == '__main__':
    try:
        ObjectPoseTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

