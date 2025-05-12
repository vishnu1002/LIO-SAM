# Visualizing Point Cloud in RViz2 (Without LIOSAM)

This guide explains how to visualize point cloud data from a ROS 2 bag file in RViz2, without using LIOSAM.

## Prerequisites
- ROS 2 Humble installed
- Your bag file (e.g., `rosbag2_2025_04_17-13_52_08_0.db3`) is located in the `dataset/` folder
- The point cloud topic in your bag is `/rslidar_points` (type: `sensor_msgs/msg/PointCloud2`)

## Steps

### 1. Source Your ROS 2 Environment
Open a terminal and run:
```bash
source /opt/ros/humble/setup.bash
```

### 2. Play the Bag File
In the same terminal, play the bag file:
```bash
ros2 bag play ~/ros2_ws/dataset/rosbag2_2025_04_17-13_52_08_0.db3
```

### 3. Open RViz2
Open a new terminal, source ROS 2, and start RViz2:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 4. Configure RViz2
- Set **Fixed Frame** to `rslidar` (or try `base_link` if `rslidar` does not exist)
- Click **Add** in the Displays panel
- Select **PointCloud2**
- Set the topic to `/rslidar_points`
- Adjust point size, color transformer, and other settings as needed

### 5. (Optional) Save Your RViz2 Configuration
Once you have the visualization set up, you can save the configuration for future use:
- Go to `File > Save Config As...` in RViz2

## Notes
- This method only visualizes the raw point cloud. No mapping or odometry will be available without IMU data and LIOSAM.
- If you later obtain a bag file with IMU data, you can use LIOSAM for mapping and odometry.
