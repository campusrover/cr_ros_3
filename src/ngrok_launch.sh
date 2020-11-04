#!/bin/bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/cr_ros_3/ && ./ngrok http -subdomain=campusrover 5000
