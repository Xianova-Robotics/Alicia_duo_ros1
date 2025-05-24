import numpy as np
import pyrealsense2 as rs
import cv2
import time

class Camera(object):

    def __init__(self, width=640, height=480, fps=30):
        self.im_height = height
        self.im_width = width
        self.fps = fps
        self.intrinsics = None
        self.scale = None
        self.pipeline = None
        self.connect()

    def connect(self):
        self.pipeline = rs.pipeline()
        config = rs.config()

        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError("未检测到RealSense相机")

        print(f"发现 {len(devices)} 个RealSense设备:")
        for dev in devices:
            print(f"  - {dev.get_info(rs.camera_info.name)} (S/N: {dev.get_info(rs.camera_info.serial_number)})")
            print("正在重置相机...")
            dev.hardware_reset()  # <-- 关键重置动作

        # 等待设备重新连接（USB重启需要几秒）
        time.sleep(5)

        # 重新获取设备上下文
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError("重置后仍未检测到RealSense相机")

        config.enable_stream(rs.stream.depth, self.im_width, self.im_height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, self.fps)

        print("正在连接D405相机...")
        profile = self.pipeline.start(config)

        device = profile.get_device()
        self.scale = device.first_depth_sensor().get_depth_scale()
        print("相机深度比例:", self.scale)

        rgb_profile = profile.get_stream(rs.stream.color)
        self.intrinsics = self.get_intrinsics(rgb_profile)
        print("D405已连接...")


    def get_data(self):
        # 增加重试机制，避免帧获取失败
        for _ in range(3):  # 尝试获取三次
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=10000)  # 增加超时时间
                if frames:
                    break
            except Exception as e:
                print(f"等待帧超时或错误: {e}")
                time.sleep(1)  # 暂停1秒后再试

        # 对齐
        align = rs.align(align_to=rs.stream.color)
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # 将图像转换为numpy数组
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_image

    def plot_image(self):
        color_image, depth_image = self.get_data()
        # 在深度图像上应用颜色映射（图像必须先转换为每像素8位）
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # 如果深度和颜色分辨率不同，调整颜色图像大小以匹配深度图像以供显示
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        # 显示图像
        cv2.namedWindow('D405 Camera View', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('D405 Camera View', images)
        key = cv2.waitKey(100)
        return key
        
    def close(self):
        # 关闭相机连接
        if self.pipeline:
            self.pipeline.stop()
            print("D405相机已断开连接")

    def get_intrinsics(self, rgb_profile):
        raw_intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()
        print("相机内参:", raw_intrinsics)
        # 相机内参形式如下：
        #[[fx,0,ppx],
        # [0,fy,ppy],
        # [0,0,1]]
        intrinsics = np.array([raw_intrinsics.fx, 0, raw_intrinsics.ppx, 
                              0, raw_intrinsics.fy, raw_intrinsics.ppy, 
                              0, 0, 1]).reshape(3, 3)
        return intrinsics
        
if __name__== '__main__':
    try:
        # 创建D405相机实例，使用D405支持的分辨率
        mycamera = Camera(width=640, height=480, fps=30)
        print("相机内参矩阵：")
        print(mycamera.intrinsics)
        
        # 实时显示相机画面，按ESC键退出
        print("正在显示相机画面，按ESC键退出...")
        while True:
            key = mycamera.plot_image()
            if key == 27:  # ESC键
                break
                
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 确保相机连接被正确关闭
        if 'mycamera' in locals():
            mycamera.close()
        cv2.destroyAllWindows()
