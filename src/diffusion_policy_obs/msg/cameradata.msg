# cameraData.msg
std_msgs/Header header
uint8[] rgb          # 扁平化后的图像数据
uint32 T                # 帧数
uint32 H                # 图像高度
uint32 W                # 图像宽度
uint32 C                # 颜色通道数

