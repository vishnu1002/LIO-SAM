#!/bin/bash
set -e

sudo apt update && sudo apt upgrade -y

# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS key
sudo apt update
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

# Install ROS Noetic Desktop Full
sudo apt install -y ros-noetic-desktop-full

# Add ROS to bashrc
if ! grep -Fxq "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
source /opt/ros/noetic/setup.bash

# Install ROS dependencies
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Install required ROS packages
sudo apt-get install -y ros-noetic-navigation ros-noetic-robot-localization ros-noetic-robot-state-publisher

# Install GTSAM
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

# Set up workspace
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src
git clone https://github.com/vishnu1002/LIO-SAM.git
cd ~/ros1_ws

# Build
catkin_make -j1 

# Source the workspace
source ~/ros1_ws/devel/setup.bash 