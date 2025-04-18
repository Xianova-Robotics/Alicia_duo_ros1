#include "serial_server_node/serial_server_node.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>

SerialServerNode::SerialServerNode() : 
    baudrate_(921600),
    timeout_ms_(1000),
    is_running_(false),
    debug_mode_(false)  // 默认关闭调试模式
{
    // 初始化ROS发布者和订阅者
    pub_serial_data_ = nh_.advertise<std_msgs::UInt8MultiArray>("/read_serial_data", 10);
    sub_send_data_ = nh_.subscribe("/send_serial_data", 10, &SerialServerNode::sendSerialDataCallback, this);
    
    // 从参数服务器读取调试模式参数
    nh_.param<bool>("debug_mode", debug_mode_, false);
    ROS_INFO_STREAM("Debug mode: " << (debug_mode_ ? "enabled" : "disabled"));

    // 使用正确的语法设置关闭回调
    ros::NodeHandle n;
    n.setParam("/serial_server_node/shutdown_requested", false);
    
    // 连接串口
    connectSerial();
}

// 析构函数
SerialServerNode::~SerialServerNode() {
    // 确保线程和串口在析构时被正确关闭
    shutdownCallback();
}

// 运行节点
void SerialServerNode::run() {
    ROS_INFO("serial_server_node is running");
    ros::spin();
}

// 关闭回调
void SerialServerNode::shutdownCallback() {
    is_running_ = false;
    
    // 等待线程结束
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    
    // 关闭串口
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_.isOpen()) {
        serial_port_.close();
        ROS_INFO("Serial port closed");
    }
}

// 连接串口
bool SerialServerNode::connectSerial() {
    try {
        std::string port = "/dev/ttyUSB0";
        if (port.empty()) {
            ROS_ERROR("No serial port found");
            return false;
        }
        
        ROS_INFO_STREAM("Connecting to port: " << port);
        // 设置串口参数
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
        
        serial_port_.setPort(port);
        serial_port_.setBaudrate(baudrate_);
        
        // 修复setTimeout调用
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_port_.setTimeout(timeout);
        
        serial_port_.open();
        if (!serial_port_.isOpen()) {
            ROS_ERROR("Failed to open serial port");
            return false;
        }
        
        ROS_INFO("Serial port opened successfully");
        
        // 启动读取线程
        is_running_ = true;
    
        read_thread_ = std::thread(&SerialServerNode::readFrameThread, this);
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception while connecting to serial port: " << e.what());
        
        // 设置定时器进行重连
        ros::Timer timer = nh_.createTimer(
            ros::Duration(5.0),
            [this](const ros::TimerEvent&) { connectSerial(); },
            true  // 仅执行一次
        );
        
        return false;
    }
}
// 查找可用的串口
std::string SerialServerNode::findSerialPort() {
    std::vector<serial::PortInfo> ports = serial::list_ports();
    
    std::stringstream ss;
    ss << "Found " << ports.size() << " serial ports:";
    for (const auto& port : ports) {
        ss << " " << port.port;
    }
    ROS_INFO_STREAM(ss.str());
    
    if (ports.empty()) {
        return "";
    }
    
    // 尝试选择第二个端口（如果有），否则使用第一个
    if (ports.size() > 1) {
        return ports[1].port;
    }
    return ports[0].port;
}

// 发送数据回调
void SerialServerNode::sendSerialDataCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // 将ROS消息转换为字节数组
    std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
    
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (!serial_port_.isOpen()) {
            ROS_WARN("Serial port is not open, attempting to reconnect");
            if (!connectSerial()) {
                ROS_ERROR("Failed to reconnect to serial port");
                return;
            }
        }
        
        // 写入数据

        size_t bytes_written = serial_port_.write(data);
        if (bytes_written != data.size()) {
            ROS_WARN_STREAM("Only wrote " << bytes_written << " of " << data.size() << " bytes");
        }
        
        printHexFrame(data, 0);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception while sending data: " << e.what());
        // 尝试重新连接而不是退出程序
        connectSerial();
    }
}
// 读取数据线程
void SerialServerNode::readFrameThread() {
    std::vector<uint8_t> frame_buffer;
    bool wait_for_start = true;
    
    while (is_running_ && ros::ok()) {
        try {
            // 检查串口是否打开
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (!serial_port_.isOpen()) {
                    ROS_WARN("Serial port closed unexpectedly, attempting to reconnect");
                    if (!connectSerial()) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        continue;
                    }
                }
            }
            
            // 读取一个字节
            uint8_t byte = 0;
            size_t bytes_read = 0;
            
            {   // 锁定串口以确保线程安全
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (serial_port_.available() > 0) {
                    bytes_read = serial_port_.read(&byte, 1);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            // 检查读取的字节数
            if (bytes_read == 0) {
                continue;
            }
            
            // 帧解析逻辑
            if (wait_for_start) {
                // 查找帧开始标记
                if (byte == 0xAA) {
                    frame_buffer.clear();
                    frame_buffer.push_back(byte);
                    wait_for_start = false;
                }
            } else {
                // 构建帧
                frame_buffer.push_back(byte);
                
                // 检查是否找到帧结束标记
                if (byte == 0xFF && frame_buffer.size() >= 3) {
                    uint8_t expected_length = frame_buffer[2] + 5;  // 数据长度+5等于帧长度
                    
                    if (frame_buffer.size() == expected_length) {
                        // 验证校验和
                        if (serialDataCheck(frame_buffer)) {
                            // 构建ROS消息并发布
                            std_msgs::UInt8MultiArray msg;
                            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                            msg.layout.dim[0].size = frame_buffer.size();
                            msg.layout.dim[0].stride = 1;
                            msg.data = frame_buffer;
                            
                            printHexFrame(frame_buffer, 1);
                            pub_serial_data_.publish(msg);
                        } else {
                            ROS_WARN("Frame checksum validation failed");
                        }
                        wait_for_start = true;
                    } else if (expected_length > 64 || frame_buffer.size() > 64) {
                        // 帧太长，认为帧错误
                        ROS_WARN("Frame too long, discarding");
                        wait_for_start = true;
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception in read thread: " << e.what());
            wait_for_start = true;
            
            // 尝试重新连接
            std::this_thread::sleep_for(std::chrono::seconds(1));
            connectSerial();
        }
    }
}
// 检查数据的校验和
bool SerialServerNode::serialDataCheck(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        return false;
    }
    
    uint8_t calculated_check = sumElements(data) % 2;
    uint8_t received_check = data[data.size() - 2];
    
    return (calculated_check == received_check);
}

// 计算数据的校验和
uint8_t SerialServerNode::sumElements(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        ROS_ERROR("Data array too small for checksum calculation");
        return 0;
    }
    
    // 计算从第3个字节到倒数第2个字节之前的所有元素的和
    uint32_t sum = 0;
    for (size_t i = 3; i < data.size() - 2; ++i) {
        sum += data[i];
    }
    
    return sum % 2;
}
// 打印十六进制数据
void SerialServerNode::printHexFrame(const std::vector<uint8_t>& data, int type) {
    // 如果不是调试模式，直接返回
    if (!debug_mode_) {
        return;
    }
    
    std::stringstream ss;
    
    // 根据类型选择前缀
    if (type == 0) {
        ss << "发送数据: ";
    } else if (type == 1) {
        ss << "数据接收: ";
    } else {
        ss << "接收数据的一部分: ";
    }
    
    // 添加每个字节的十六进制表示
    for (const auto& byte : data) {
        ss << std::uppercase << std::setfill('0') << std::setw(2) 
           << std::hex << static_cast<int>(byte) << " ";
    }
    
    ROS_INFO_STREAM(ss.str());
}