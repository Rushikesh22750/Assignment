# Swaayatt Assignment

## Overview
This is a ROS package for object detection, tracking, and visualization using YOLOv5s and Kalman Filter. It includes the following nodes:

1. **Image Subscriber** taking input from Realsense Camera (C++)
2. **Object Detection** using YOLOv5s (Python)
3. **Object Tracking** using Kalman Filter (C++)
4. **Visualization** node using OpenCV (C++)

##  ROS Topics

| **Topic**                     | **Description**                                        |
|------------------------------|--------------------------------------------------------|
| `/camera/color/image_raw`    | RGB images from RealSense camera             |
| `/processed_image`           | Preprocessed Images               |
| `/object_detection`          | YOLO detections    |
| `/object_tracking`           | Kalman filter tracking data        |
| `/visualization/image`       | Annotated outputs with detection and tracking overlays |


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
 ```bash
  
sudo apt-get install ros-noetic-realsense2-camera
```
## 🔧 Installation and Setup

Step 1: Clone the Repository

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
Run Full Pipeline with ROS launch
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
rosrun swaayatt_rushikesh image_subscriber
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
