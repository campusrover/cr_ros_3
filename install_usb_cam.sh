#!/bin/bash
cs
git clone https://github.com/ros-drivers/usb_cam.git
rosdep install usb_cam
cd ~/.ros
mkdir -p camera_info
cd camera_info
cat ~/catkin_ws/src/cr_ros_3/head_camera.yaml >> head_camera.yaml