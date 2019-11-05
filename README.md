# Welcome to the campus rover repository, version 3. This repo contains python code for the campus rover project

to get started with this repo, you will need a few things installed on your laptop:
* the fiducials package, which contains `aruco_detect`. It can be [cloned from github](https://github.com/UbiquityRobotics/fiducials) or installed with apt-get (command is in the lab notebook)
* flask python module - google how to install it with pip
* google chrome - there are plenty of tutorials on the internet which will guide you through installing chrome on ubuntu. If you really prefer firefox or some other browser, then **A**: what is wrong with you? and **B**: you will have to edit a shell script in cr_web. speaking of that...
* [the cr_web package](https://github.com/campusrover/cr_web/tree/mutant)
* it is entirely possible that there are other dependencies that aren't listed. use this package at your own risk!

This package should be installed on both the remote pc AND the robot. The robot has it's own set of required dependencies, most of which are python modules which will throw errors if not installed. Check out the lab notebook page on mutant setup for better instructions.

This package consists of both an offboard and onboard bringup. The goal is that basic nodes run on the weak robot cpu, while more resource-heavy nodes run on the remote pc.

to launch onboard (on the robot - use ssh)
```
roslaunch cr_ros_3 mutant_onboard_rpicam.launch
```
**the above launch file includes turtlebot_bringup! You do not need to bring up the robot in addition to launching this package**
and to launch offboard (on remote pc):
```
roslaunch cr_ros_3 mutant_offboard_rpicam.launch
```
to disable fiducials, do the following:
```
roslaunch cr_ros_3 mutant_offboard_rpicam.launch fids:=FALSE
```

It is highly suggested that you set up aliases for both of these commands on your robot and your remote pc. 

## How to add a new node
If you are adding a node that will affect the robot's behavior, then it should this package's state manager

Currently, there are 7 states:
* Waiting - the campus rover knows it's location and has no current objective. It is motionless. 
* Navigating - the campus rover has recieved a nav goal and is currently en route to the destination
* Teleop - Someone is remotely controlling the campus rover via the web app
* Lost - an event has caused the robot to have bad localization data (it was just turned on, or was picked up and moved) - it's current pose cannot be trusted. It needs a new pose estimate before it can resume operation
* Flying - sensor data indicates that the campus rover has been picked up off the floor. 
* Stolen Package - sensor data indicates that the package the campus rover was carrying has been removed prematurely (deprecated state on MUTANT until a touch/ weight sensor is added to the top of the robot)
* Search - the campus rover is lookng for a person using CV. (State added by gen3 for a specific purpose)

Always ensure: 
1. that your new node will only perform actions while in the appropriate state
2. if your node detects a reason to change states, then it requests the appropriate state change
3. if you need a new state, add it and it's legal state changes to src/all_state.py

examples: 
* you create a node that suspends navigation when a certain hand gesture is recognized. It should only execute critical code while state = NAVIGATING. If the gesture is recognized, then a state changed should be requested. 
* you make a new node that makes the robot look like it is dancing. add a DANCING state to src/all_states.py and all legal state changes both out and in to DANCING (e.g. if you can dance after waiting, then add DANCING to WAITING's legal state change list)

When adding a node, you will also need to add it to the appropriate launch file - either onboard or offboard. 
* ONBOARD nodes should be lightweight and/or have some sort of hardware requirement associated with it - for instance, if the node publishes pictures directly from the camera, then it should be running onboard. 
* OFFBOARD nodes can be all other nodes. Remember, ROS is distributed - if this package ever becomes too computationally intense, the offboard components can be distributed amongst many launch files, each one running on a different remote PC. (e.g. there should probably only be one CV/ML node per remote PC...)

## Facial recognition
Two different attempts have been made at facial recognition so far. 

#### Gen 2
see `greet.py`

#### Gen 3
there is another repositiory in the campusrover group called [Robotic Computer Vision](https://github.com/campusrover/Robotics_Computer_Vision)
running the `detect.py` script in that package will begin publishing facial recognition information. **NB** `detect.py` is not a ROS node but it will publish a ROS topic. Therefore you can run `detect.py` in the terminal by using `python`, not `rosrun` 
`detect.py` communicates with `go_to_person.py` in cr_ros_3. 
`detect.py` requires a discrete GPU, and should probably not be run alongside other nodes on the same remote PC. 

## Hand gestures
*We are currently in process of retrieving the gen 3 source code for hand gesture publishing from it's author*

## State of the package report
#### NOTE TO CONTRIBUTORS: please update this section after every sizable commit!
11/5/19
* Haofan's Hand gesture node is AWOL. We've reached out to him to get the code, awaiting response
* Facial recognition node also AWOl. 
* Both facets above should be replaced, removed or redone. 
* Some changes have been made regarding the Alexa/voice integration, but has not been tested
* improving documentation of all nodes - each node should have a clear statement of purpose at the top. 
* launch files now have greater modularity. related nodes have been grouped into launch files. 
* some files have been renamed for clarity. these include: 
    * `detect_pickup.py` --> `pickup_detector.py`
    * `lost_and_found.py` --> `pickup_recovery.py` These two nodes are connected, so their names now reflect that. 
    * `message_switch.py` --> `talk_queue.py` "message switch" was ambiguous. didn't really tell you what the node did. "talk queue" perfectly captures the function: it is a queue for the talk service. 
    * `delivery.py` --> `voice_delivery.py`

Things that should/need to be done:
* nodes that currently directly interface talk_srv should publish to /things_to_say instead, let talk_queue handle talk service interface
* hand gesture + facial recognition need to be tracked down and added
* continue improving documentation
* finalize launch modularity - what should go where?
