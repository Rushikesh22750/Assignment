# swaayatt_rushikesh

## Overview
This ROS package implements a complete pipeline for object detection, tracking, and visualization. It includes the following nodes:

1. **Image Subscriber** from Realsense Camera (C++)
2. **Object Detection** using YOLOv5s (Python/ONNXRuntime)
3. **Object Tracking** using Kalman Filter (C++)
4. **Visualization** node using OpenCV (C++)

## Dependencies
- **ROS Version:** Noetic
- **ROS Packages:**
  - `roscpp`
  - `rospy`
  - `std_msgs`
  - `sensor_msgs`
  - `message_generation`
  - `message_runtime`
  - `cv_bridge`
  - `image_transport`
- **Other Libraries:**
  - OpenCV (C++)
  - Python 3
  - NumPy
  - ONNXRuntime (for YOLO model inference)
  - Intel RealSense ROS Driver ([realsense-ros](https://github.com/IntelRealSense/realsense-ros))

## Package Structure
