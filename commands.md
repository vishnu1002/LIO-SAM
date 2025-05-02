# LIO-SAM Installation and Usage Guide

This guide provides step-by-step instructions for installing and running LIO-SAM on Ubuntu 20.04 with ROS Noetic, optimized for WSL2 on Windows 11.

## System Requirements

- Ubuntu 20.04 (WSL2 on Windows 11)
- Minimum 8GB RAM (16GB recommended)
- AMD Ryzen 5 3550H or equivalent processor
- NVIDIA GTX 1650 or equivalent graphics card
- X Server (VcXsrv or X410) for visualization

## Installation Steps

### Quick Installation (Recommended)

For a quick and automated installation, you can use the provided installation script:

```bash
# Download the install.sh script to your workspace root
# Make it executable
chmod +x install.sh

# Run the installation script
./install.sh
```

The script will automatically handle all the installation steps below. Make sure to run it from your workspace root directory.

### Manual Installation

If you prefer to install manually, follow these steps:

### 1. Install ROS Noetic

```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS key
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package list
sudo apt update

# Install ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full -y

# Add ROS to bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ROS dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init && rosdep update
```

### 2. Install Required ROS Packages

```bash
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
```

### 3. Install GTSAM

```bash
# Add GTSAM repository
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update

# Install GTSAM
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### 4. Set Up Workspace

```bash
# Create workspace
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src

# Clone LIO-SAM repository
git clone https://github.com/vishnu1002/LIO-SAM.git

# Build workspace
cd ~/ros1_ws
catkin_make -j1

# Clean build (if needed)
rm -rf build devel

# Source the workspace
echo "source ~/ros1-ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Running LIO-SAM

### 1. Start LIO-SAM

```bash
roslaunch lio_sam run.launch
```

### 2. Play Bag File

```bash
rosbag play bag.bag -r 3
```

## Troubleshooting

### Common Issues

1. **Memory Issues During Build**

   - Use `catkin_make -j1` to reduce memory usage
   - Increase WSL2 memory limit in `.wslconfig`

2. **RViz Crashes**

   - Ensure X Server is running
   - Check DISPLAY environment variable
   - Use software rendering if needed

3. **Build Errors**
   - Clean build directory and rebuild
   - Check ROS and package versions
   - Verify all dependencies are installed

## Notes

- This setup is optimized for WSL2 on Windows 11
- Memory settings may need adjustment based on your system
- X Server configuration is required for visualization
- Performance may vary based on hardware specifications

## Support

For issues or questions, please refer to:

- LIO-SAM GitHub repository
- ROS documentation
- WSL2 documentation
