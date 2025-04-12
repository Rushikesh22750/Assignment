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



---

## 🔧 Installation and Setup

```bash
# Step 1: Clone the Repository
cd ~/swaayatt_ws/src
git clone https://github.com/Rushikesh22750/Assignment.git swaayatt_rushikesh

# Step 2: Install Dependencies
cd ~/swaayatt_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install numpy onnxruntime

# Step 3: Build the Workspace
catkin_make
source devel/setup.bash

# 🚀 Running the Nodes

# Run Full Pipeline (Recommended)
roslaunch swaayatt_rushikesh pipeline.launch

# Run Individual Nodes (for Debugging)

# 1. RealSense Camera Node
roslaunch realsense2_camera rs_camera.launch

# 2. Image Subscriber & Pre-processing Node
rosrun swaayatt_rushikesh image_processor

# 3. YOLO Object Detection Node
rosrun swaayatt_rushikesh yolo_detector.py

# 4. Object Tracking Node
rosrun swaayatt_rushikesh object_tracker

# 5. Visualization Node
rosrun swaayatt_rushikesh visualization_node
```


## 🔧 Installation and Setup

### 📁 Step 1: Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/Rushikesh22750/Assignment.git swaayatt_rushikesh
```

Step 2: Install Dependencies

```bash
cd ~/swaayatt_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install numpy onnxruntime
```

Step 3: Build the Workspace
```bash
catkin_make
source devel/setup.bash
```
Run Full Pipeline 
```bash
roslaunch swaayatt_rushikesh pipeline.launch

```
Run Individual Nodes
1. RealSense Camera Node

```bash
roslaunch realsense2_camera rs_camera.launch


```

2. Image Subscriber Node
```bash
rosrun swaayatt_rushikesh image_processor


```

3. YOLO Object Detection Node
```bash
rosrun swaayatt_rushikesh yolo_detector.py

```

4. Object Tracking Node
```bash
rosrun swaayatt_rushikesh object_tracker
```

5. Visualization Node
```bash
rosrun swaayatt_rushikesh visualization_node

```
