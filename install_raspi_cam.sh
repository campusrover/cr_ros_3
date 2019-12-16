#!/bin/bash
sudo apt update
sudo apt upgrade
sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
sudo apt update
sudo apt upgrade
sudo apt install ros-kinetic-raspicam-node
cs 
git clone https://github.com/UbiquityRobotics/raspicam_node.git
cm