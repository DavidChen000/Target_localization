cmake_minimum_required(VERSION 2.8.3)
project(yolov5_new)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   VisionCarBoardPos.msg
   YoloRes.msg
   DetectBox.msg
 )

## Generate services in the 'srv' folder
## add_service_files(
##   FILES
##  )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
  #DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/detect.py scripts/target_detect_node.py scripts/video_cap_node.py scripts/miniA8_detect_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

