#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'
BG_GREEN='\033[42m'
BG_RED='\033[41m'
BG_YELLOW='\033[43m'

# Function to print status messages
print_status() {
    echo -e "${BOLD}${BLUE}[INFO]${NC} $1"
}

# Function to print success messages
print_success() {
    echo -e "${BOLD}${GREEN}[SUCCESS]${NC} $1"
}

# Function to print error messages
print_error() {
    echo -e "${BOLD}${RED}[ERROR]${NC} $1"
}

# Function to print progress
print_progress() {
    echo -e "${BOLD}${YELLOW}[PROGRESS]${NC} $1"
}

# Function to show progress bar
progress_bar() {
    local duration=$1
    local steps=50
    local step_delay=$(echo "scale=3; $duration/$steps" | bc)
    
    echo -ne "${BOLD}${CYAN}["
    for ((i=0; i<=steps; i++)); do
        echo -ne "="
        sleep $step_delay
    done
    echo -ne "]${NC}\r"
    echo -ne "\n"
}

# Function to check if a command succeeded
check_status() {
    if [ $? -eq 0 ]; then
        print_success "$1"
    else
        print_error "$2"
        exit 1
    fi
}

# Clear screen and show welcome message
clear
echo -e "${BOLD}${MAGENTA}=============================================${NC}"
echo -e "${BOLD}${MAGENTA}           LIO-SAM Installation Script        ${NC}"
echo -e "${BOLD}${MAGENTA}=============================================${NC}\n"

# Update and upgrade system
print_status "Updating and upgrading system..."
sudo apt update && sudo apt upgrade -y
check_status "System updated and upgraded" "Failed to update and upgrade system"

# Start installation
print_status "Starting LIO-SAM installation...\n"

# 1. Install ROS Noetic
print_progress "Installing ROS Noetic..."
progress_bar 2
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
check_status "Added ROS repository" "Failed to add ROS repository"

print_progress "Installing curl..."
progress_bar 1
sudo apt install curl -y
check_status "Installed curl" "Failed to install curl"

print_progress "Adding ROS key..."
progress_bar 1
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
check_status "Added ROS key" "Failed to add ROS key"

print_progress "Updating package list..."
progress_bar 1
sudo apt update
check_status "Updated package list" "Failed to update package list"

print_progress "Installing ROS Noetic Desktop Full..."
progress_bar 5
sudo apt install ros-noetic-desktop-full -y
check_status "Installed ROS Noetic Desktop Full" "Failed to install ROS Noetic"

print_progress "Setting up ROS environment..."
progress_bar 1
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
check_status "Added ROS to bashrc" "Failed to add ROS to bashrc"

print_progress "Installing ROS dependencies..."
progress_bar 2
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
check_status "Installed ROS dependencies" "Failed to install ROS dependencies"

print_progress "Initializing rosdep..."
progress_bar 2
sudo rosdep init
rosdep update
check_status "Initialized rosdep" "Failed to initialize rosdep"

# 2. Install Required ROS Packages
print_progress "Installing required ROS packages..."
progress_bar 2
sudo apt-get install -y ros-noetic-navigation ros-noetic-robot-localization ros-noetic-robot-state-publisher
check_status "Installed ROS packages" "Failed to install ROS packages"

# 3. Install GTSAM
print_progress "Installing GTSAM..."
progress_bar 3
sudo add-apt-repository ppa:borglab/gtsam-release-4.0 -y
sudo apt update
check_status "Added GTSAM repository" "Failed to add GTSAM repository"

print_progress "Installing GTSAM libraries..."
progress_bar 2
sudo apt install libgtsam-dev libgtsam-unstable-dev -y
check_status "Installed GTSAM" "Failed to install GTSAM"

# 4. Set Up Workspace
print_progress "Setting up workspace..."
progress_bar 1
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src
check_status "Created workspace" "Failed to create workspace"

print_progress "Cloning LIO-SAM repository..."
progress_bar 3
git clone https://github.com/vishnu1002/LIO-SAM.git
check_status "Cloned LIO-SAM repository" "Failed to clone LIO-SAM repository"

cd ~/ros1_ws
print_progress "Building workspace (this may take a while)..."
progress_bar 10
catkin_make -j1
check_status "Built workspace" "Failed to build workspace"

# Add workspace to bashrc
print_progress "Setting up environment..."
progress_bar 1
echo "source ~/ros1_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
check_status "Added workspace to bashrc" "Failed to add workspace to bashrc"

echo -e "\n${BOLD}${GREEN}=============================================${NC}"
echo -e "${BOLD}${GREEN}    LIO-SAM Installation Completed Successfully!${NC}"
echo -e "${BOLD}${GREEN}=============================================${NC}\n"

print_status "To run LIO-SAM, use: ${BOLD}roslaunch lio_sam run.launch${NC}"
print_status "To play a bag file, use: ${BOLD}rosbag play your-bag.bag -r 3${NC}"