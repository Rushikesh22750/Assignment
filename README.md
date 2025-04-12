# swaayatt_rushikesh

## Overview
This ROS package demonstrates a **real-time object detection and tracking pipeline** using:
1. **Image Subscriber & Pre-processing (C++)**  
2. **YOLOv5 Object Detection (Python, ONNX)**  
3. **Object Tracking (Kalman Filter, C++)**  
4. **Visualization Node (C++)** for overlaying detections and trajectories on images.  

---

## Dependencies

1. **ROS Noetic** with:
   - `roscpp`, `rospy`
   - `sensor_msgs`, `std_msgs`
   - `cv_bridge`, `image_transport`
   - `message_generation`, `message_runtime`
2. **OpenCV** (for visualization and Kalman filtering in C++).
3. **Python libraries**:
   - `onnxruntime` (for YOLO inference):  
     ```bash
     pip3 install onnxruntime
     ```
   - `numpy`:  
     ```bash
     pip3 install numpy
     ```
4. **Camera driver** (optional). If you’re using an Intel RealSense camera:
   ```bash
   sudo apt-get install ros-noetic-realsense2-camera
