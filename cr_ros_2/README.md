Welcome to the campus rover repository, version 2. This repo contains the base code needed to run mutant for gen3's demo.

to get started with this repo, you will need a few things installed on your laptop:
* the fiducials package, which contains `aruco_detect`. It can be [cloned from github](https://github.com/UbiquityRobotics/fiducials) or installed with apt-get (command is in the lab notebook)
* flask python module - google how to install it with pip
* google chrome - there are plenty of tutorials on the internet which will guide you through installing chrome on ubuntu. If you really prefer firefox or some other browser, then **A**: what is wrong with you? and **B**: you will have to edit a shell script in cr_web. speaking of that...
* [the cr_web package](https://github.com/campusrover/cr_web/tree/mutant), which should be on the branch `mutant`, not `master`
* it is entirely possible that there are other dependencies that aren't listed. use this package at your own risk!

This package should be installed on both the remote pc AND the robot. The robot has it's own set of required dependencies, most of which are python modules which will throw errors if not installed. Check out the lab notebook page on mutant setup for better instructions.

This package consists of both an offboard and onboard bringup. The goal is that basic nodes run on the weak robot cpu, while more resource-heavy nodes run on the remote pc.

to launch onboard (on the robot - use ssh)
```
roslaunch cr_ros_2 mtnt_onb_rpicam.launch
```
**the above launch file includes turtlebot_bringup! You do not need to bring up the robot in addition to launching this package**
and to launch offboard (on remote pc):
```
roslaunch cr_ros_2 mutant_offboard_rpicam.launch
```

It is highly suggested that you set up aliases for both of these commands on your robot and your remote pc. 
