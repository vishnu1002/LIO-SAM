#!/bin/bash
set -e

# Universe repository is enabled
sudo apt update && sudo apt upgrade -y

sudo apt install -y software-properties-common
sudo add-apt-repository universe

# ROS 2 GPG key
sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repo to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop

# Environment setup
if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source /opt/ros/humble/setup.bash

# Install development tools
sudo apt install -y ros-dev-tools

# Install required ROS2 packages
sudo apt install -y ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-vision-opencv ros-humble-xacro

# Install GTSAM
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# Set up ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vishnu1002/LIO-SAM.git
cd LIO-SAM
git checkout ros2
cd ~/ros2_ws

# Build
colcon build 

# Source the workspace
source ~/ros2_ws/install/setup.bash 