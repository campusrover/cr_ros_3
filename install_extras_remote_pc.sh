#!/bin/bash
cs 
echo "[Cloning github repos]"
git clone https://github.com/campusrover/cr_web.git
git clone https://github.com/UbiquityRobotics/fiducials.git
echo "[Installing python modules]"
pip install Flask
echo "[Building catkin workspace]"
cm
cd ~
echo "[Setting evironment variables]"
sh -c "echo \"export WEB_BROWSER=google-chrome\" >> ~/.bashrc"
echo "[Done!]"