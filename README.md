# Swaayatt Assignment

## Overview
This is a ROS package for object detection, tracking, and visualization using YOLOv5s and Kalman Filter. It includes the following nodes:

1. **Image Subscriber** taking input from Realsense Camera (C++)
2. **Object Detection** using YOLOv5s (Python/ONNXRuntime)
3. **Object Tracking** using Kalman Filter (C++)
4. **Visualization** node using OpenCV (C++)

##  ROS Topics

| **Topic**                     | **Description**                                        |
|------------------------------|--------------------------------------------------------|
| `/camera/color/image_raw`    | Original RGB images from RealSense camera             |
| `/processed_image`           | Preprocessed images (converted to BGR)                |
| `/object_detection`          | YOLO detections (class labels, confidence, boxes)     |
| `/object_tracking`           | Kalman-filtered tracking data with object IDs         |
| `/visualization/image`       | Annotated output with detection and tracking overlays |


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


## Installation and Setup

### Step 1: Clone the repository into your ROS workspace:
```bash
cd ~/swaayatt_ws/src
git clone https://github.com/Rushikesh22750/Assignment.git swaayatt_rushikesh
