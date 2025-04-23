#!/usr/bin/env bash

# 更新源
sudo apt update
sudo apt -y autoremove

# 安装依赖
sudo apt -y install python3-serial
# 安装机械臂相关包
ALICIA_WS=~/alicia_ws
if [ ! -d "$ALICIA_WS/src" ]; then
  echo "Installing Alicia Duo ROS packages..."
  mkdir -p "$ALICIA_WS/src"
  cd "$ALICIA_WS/src"
  git clone https://gitee.com/xuanyatech/Alicia_duo_ros.git
  cd "$ALICIA_WS"
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  if ! grep -Fxq "source $ALICIA_WS/devel/setup.bash" ~/.bashrc; then
    echo "source $ALICIA_WS/devel/setup.bash" >> ~/.bashrc
  fi
else
  echo "Alicia Duo ROS packages already installed!"
fi
source "$ALICIA_WS/devel/setup.bash"

