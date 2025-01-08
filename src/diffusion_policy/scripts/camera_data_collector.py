#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import os
import ffmpeg
from std_msgs.msg import String

class ImageToVideoNode:
    def __init__(self):
        rospy.init_node('image_to_video', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        
        self.key_sub = rospy.Subscriber('key_event', String, self.key_callback)

        self.frame_width = 1280
        self.frame_height = 720
        self.fps = 30
        self.recording = False
        self.folder_index = 0
        self.base_directory = './real_pusht_2024_12_13/videos'
        self.cooldown_time = 2
        self.last_stop_time = time.time() - self.cooldown_time

        if not os.path.exists(self.base_directory):
            os.makedirs(self.base_directory)

        rospy.loginfo("Waiting for camera info...")

    def camera_info_callback(self, camera_info):
        self.frame_width = camera_info.width
        self.frame_height = camera_info.height
        rospy.loginfo(f"Received camera info: width={self.frame_width}, height={self.frame_height}")
        self.camera_info_sub.unregister()

    def start_recording(self):
        if not self.recording:
            folder_name = os.path.join(self.base_directory, str(self.folder_index))
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)

            video_path = os.path.join(folder_name, '0.mp4')
            rospy.loginfo(f"Start recording video: {video_path}")
            
            self.ffmpeg_process = ffmpeg.input('pipe:0', format='rawvideo', pix_fmt='bgr24',
                                               s=f'{self.frame_width}x{self.frame_height}', r=self.fps) \
                                    .output(video_path, vcodec='libx264', pix_fmt='yuv420p') \
                                    .overwrite_output() \
                                    .run_async(pipe_stdin=True)
            self.recording = True
            self.folder_index += 1

    def stop_recording(self):
        if self.recording:
            rospy.loginfo("Stop recording")
            if self.ffmpeg_process is not None:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait()
                self.ffmpeg_process = None
            self.recording = False
            self.last_stop_time = time.time()

    def image_callback(self, data):
        if self.recording:
            try:
                if self.ffmpeg_process is None:
                    rospy.logwarn("FFmpeg process is not running, skipping frame.")
                    return

                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.ffmpeg_process.stdin.write(cv_image.tobytes())

            except Exception as e:
                rospy.logerr("Error processing image: %s" % str(e))

    def key_callback(self, msg):
        key = msg.data.lower()
        if key == 'c':
            self.start_recording()
        elif key == 't':
            self.stop_recording()

    def cleanup(self):
        if self.recording:
            self.stop_recording()
        rospy.loginfo("Video recording stopped.")

if __name__ == '__main__':
    try:
        node = ImageToVideoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()







