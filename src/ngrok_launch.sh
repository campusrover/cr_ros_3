#!/bin/bash
source ~/catkin_ws/devel/setup.bash
cd ~ && ./ngrok http -subdomain=campusrover 5000
