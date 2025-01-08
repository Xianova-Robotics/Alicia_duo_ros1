import requests

def send_data():
    url = "http://localhost:5000/send_data"
    
    # 示例数据
    data = {
        "obs": {
            "camera_0": [[1, 2], [3, 4]],
            "robot_eef_pose": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        },
        "action": [1.0, -1.0]
    }
    
    # 发送 POST 请求
    response = requests.post(url, json=data)
    
    # 打印服务器响应
    print(f"Server Response: {response.json()}")

if __name__ == "__main__":
    send_data()

