# 灵动 Alicia-D 系列机械臂

[English Version](README_en.md) | [中文版](README.md) | [官方淘宝店](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [灵动 Alicia-D 产品手册](https://tcnqzgyay0jb.feishu.cn/wiki/ElDUwERlNilPLWkJ2e2cYGyZncb?fromScene=spaceOverview)

![Alicia-D](images/Alicia_Duo_V5_4.png)

## 简介

**【灵动 Alicia-D】系列机械臂是一套高性价比、功能完善的遥操作数据采集以及模仿学习IL、强化学习RL、VLA等先进机器人算法复现平台。**
**本repo是灵动系列单操作臂的ROS1代码及其例程。**

[淘宝购买链接](https://e.tb.cn/h.h6jfG5QfQVm5Ndq?tk=7T1q4aGvWz1)

## 安装

### 系统建议

目前只在如下系统中配置中进行了测试

- Ubuntu 20.04
- ROS Noetic

### 安装流程

运行以下代码

```
sh install/alicia_amd64_install.sh
```

## Repo结构

该仓库包含多个目录，每个目录包含与 Alicia-D 系列机械臂相关的 ROS 包和资源。以下是仓库的结构：

```
Alicia_duo_ros1
├── alicia_duo_descriptions
├── alicia_duo_drag_teaching
├── alicia_duo_driver
├── alicia_duo_grasp_2d
├── alicia_duo_grasp_6d
├── alicia_duo_moveit
├── alicia_duo_ros_control
```

以下是操作 Alicia-D 机械臂所需的核心 ROS 包：

- **`alicia_duo_driver`**: 提供与机械臂的底层控制和通信。
- **`alicia_duo_moveit`**: 配置机械臂以使用 MoveIt 进行运动规划和控制。
- **`alicia_duo_ros_control`**: 实现机械臂的 ROS 控制接口。
- **`alicia_duo_descriptions`**: 包含机械臂的 URDF 和网格文件，用于在 RViz 中可视化。
- **`alicia_duo_calibration`**: 提供手眼标定和其他校准工具。

以下功能包是基于核心包提供的例程：

- **`alicia_duo_drag_teaching`**: 实现拖动示教功能。
- **`alicia_duo_grasp_2d`**: 展示机械臂的 2D 抓取能力。
- **`alicia_duo_grasp_6d`**: Alicia-D 机械臂 6D 抓取示例。
- **examples**: 包含学习和测试的示例脚本和演示：
  - **`alicia_duo_zero_calibration.py`**: 用于机械臂零点校准的 Python 脚本。

## 链接

- **淘宝店铺**: [灵动 Alicia-D 官方淘宝店](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2)
- **产品手册**: [灵动 Alicia-D 产品手册](https://tcnqzgyay0jb.feishu.cn/wiki/ElDUwERlNilPLWkJ2e2cYGyZncb?fromScene=spaceOverview)
