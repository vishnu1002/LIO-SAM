# Visualizing Point Cloud in RViz2 (Without LIOSAM)

This guide explains how to visualize point cloud data from a ROS 2 bag file in RViz2, without using LIOSAM.

## Prerequisites
- Ubuntu 22 Jammy Jellyfish | WSL setup
- ROS 2 Humble [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- `.db3` pointcloud data [here](https://drive.google.com/drive/folders/1nU2QaPu0172DG_03LlEZBaHeRXACY1eU?usp=sharing)

## Steps

### 1. Source Your ROS 2 Environment
Open Terminal 1 and run:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Play the Bag File
In the same Terminal 1, play the data:
```bash
ros2 bag play ~/dataset/rosbag2_2025_04_17-13_52_08_0.db3
```

### 3. Open RViz2
Open a new Terminal 2, start RViz2:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 4. Configure RViz2
- Set **Fixed Frame** to `rslidar`
- Click **Add** in the Displays panel
- Select **PointCloud2**
- Set the topic to `/rslidar_points`
- Adjust point size, color transformer, and other settings as needed

### 5. (Optional) Save Your RViz2 Configuration
Once you have the visualization set up, you can save the configuration for future use:
- Go to `File > Save Config As...` in RViz2

### View data info
Make sure the data is running in background
```bash
ros2 run tf2_tools view_frames
ros2 topic list
```
