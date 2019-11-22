#!/bin/bash
source ~/catkin_ws/devel/setup.bash
roscd cr_ros_3 && rosrun dynamic_reconfigure dynparam load raspicam_node camera.yaml