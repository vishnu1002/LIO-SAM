# Contents

- [1. LIOSAM ROS](#1-liosam-ros)
  - [Setup Ubuntu 20.04 Focal Fossa](#setup-ubuntu-2004-focal-fossa)
  - [Setup ROS](#setup-ros)

- [2. LIO-SAM ROS 2](#2-lio-sam-ros-2)
  - [Setup Ubuntu 22.04 Jammy Jellyfish](#setup-ubuntu-2204-jammy-jellyfish)
  - [Setup ROS 2](#setup-ros-2)
- Dataset [link](https://drive.google.com/drive/folders/1nU2QaPu0172DG_03LlEZBaHeRXACY1eU?usp=sharing)

## 1. LIOSAM ROS 

### Setup Ubuntu 20.04 Focal Fossa

- Ubuntu 20.04 | WSL [setup](https://github.com/vishnu1002/cmd-help/blob/main/wsl--help.md)

### Setup ROS 

#### 1. Install Noetic Ninjemys | [doc](https://wiki.ros.org/noetic/Installation/Ubuntu)

```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS key
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

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

#### 2. Install Required ROS Packages

```bash
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
```

#### 3. Install GTSAM

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

#### 4. Set Up Workspace

```bash
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws/src

git clone https://github.com/vishnu1002/LIO-SAM.git

cd ~/ros1_ws
```

#### 5. Build
```bash
catkin_make -j1
```

#### 6. Source the workspace
```bash
echo "source ~/ros1_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 7. Running LIO-SAM (ROS 1)

```bash
roslaunch lio_sam run.launch
```

#### 8. Play Bag File

```bash
rosbag play bag.bag -r 3
```

### Troubleshooting (ROS1)

#### Common Issues

1. **Memory Issues During Build**

   - Use `catkin_make -j1` to reduce memory usage
   - Increase WSL2 memory limit in `.wslconfig` in root folder

2. **RViz Crashes**

   - Ensure X Server is running
   - Check DISPLAY environment variable
   - Use software rendering if needed

3. **Build Errors**
   - Clean build directory and rebuild
   - Check ROS and package versions
   - Verify all dependencies are installed

---

## 2. LIO-SAM ROS 2

### Setup Ubuntu 22.04 Jammy Jellyfish

- Ubuntu 22.04 | WSL [setup](https://github.com/vishnu1002/cmd-help/blob/main/wsl--help.md)

### Setup ROS 2

#### 1. Install ROS 2 Humble Hawksbill | [doc](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

```bash
# Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repo to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install development tools
sudo apt install ros-dev-tools
```

#### 2. Install Required ROS2 Packages

```bash
sudo apt install ros-humble-perception-pcl \
  	   ros-humble-pcl-msgs \
  	   ros-humble-vision-opencv \
  	   ros-humble-xacro
```

#### 3. Install GTSAM

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

#### 4. Set Up ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/vishnu1002/LIO-SAM.git
cd LIO-SAM
git checkout ros2
```

#### 5. Build
```bash
cd ~/ros2_ws
colcon build
```

#### 6. Source the workspace
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Running LIO-SAM (ROS 2)

#### Method 1: Visualize with LIOSAM

##### 1. Start LIO-SAM

```bash
ros2 launch lio_sam run.launch.py
```

##### 2. Play Bag File

```bash
ros2 bag play your-bag.bag
```
#### Method 2: Visualize without LIOSAM

If you only have point cloud data without IMU, you can still visualize it:

###### Terminal 1: Play the bag
```bash
ros2 bag play ~/rosbag2_2025_04_17-13_52_08_0.db3
```

###### Terminal 2: Launch RViz2
```bash
rviz2
```
###### View bag info
Make sure the bag is running
```bash
ros2 run tf2_tools view_frames
ros2 topic list
```
#### RViz2 Configuration

- Set **Fixed Frame** to `rslidar` (or the appropriate frame_id)
- Add a **PointCloud2** display and set topic to `/rslidar_points`
