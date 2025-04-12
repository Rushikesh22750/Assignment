cmake_minimum_required(VERSION 3.0.2)
project(swaayatt_rushikesh)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjects.msg
  TrackedObject.msg            # new
  TrackedObjects.msg           # new
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(image_subscriber src/image_subscriber.cpp)
target_link_libraries(image_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


# Visualization Node
add_executable(visualization_node src/visualization_node.cpp)
target_link_libraries(visualization_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(object_tracker src/object_tracker.cpp)
target_link_libraries(object_tracker ${catkin_LIBRARIES})
add_dependencies(object_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


