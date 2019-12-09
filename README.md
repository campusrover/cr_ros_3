
# Welcome to the campus rover repository, version 3. This repo contains python code for the campus rover project

## [Please Familiarize Yourself with the ARL Notebook, it is a Useful Resource](https://campus-rover.gitbook.io/lab-notebook/)

## Setting Up your Remote PC

After cloning this repo to your catkin workspace src directory, run the following command in a terminal:

``` sh
roscd cr_ros_3 && source install_extras_remote_pc.sh
```

This will add everything* needed on the remote PC for this package. After running the install script, you may need to edit your .bashrc file. You will notice two new export variables at the bttom of the file, `WEB_BROWSER` and `CR_MODEL`.

1. Set `WEB_BROWSER` to your favorite web browser, like `google-chrome`, `firefox` or `dillo`
2. Set `CR_MODEL` to `MUTANT` if you are using the Mutant in the lab, or a similar robot that runs ROS kinetic on Ubuntu 16.04 with a raspberry Pi camera. Set `CR_MODEL` to `ALIEN` if you are using the Alien in the lab, or a similar robot that runs ROS Melodic on Ubuntu 18.04 with a usb web camera.

*The remote pc install script does not yet add ngrok to the remote pc.

## Setting up the Robot

After cloning this repo to your catkin workspace src directory, run the following command in a terminal:

``` sh
roscd cr_ros_3 && source install_extras_rover.sh
```

This will add MOST of what is needed to run the package on the robot. The exception is adding the software needed to use the camera.

Just like the Remote PC setup, you will need to set `CR_MODEL` in .bashrc to accurately reflect the robot characteristics described above.

For Mutant-like robots:

1. [Follow the instructions under the first section titled "Installation" of the README](https://github.com/UbiquityRobotics/raspicam_node). Be sure to click the link and follow the instructions within, or else you will not be able to install via apt.
2. `git clone https://github.com/UbiquityRobotics/raspicam_node.git` into `catkin_ws/src`.

For Alien-like robots:

1. `git clone https://github.com/ros-drivers/usb_cam.git` into `catkin_ws/src`.
2. `rosdep install usb_cam`
3. From your home directory, input the following commands in order
4. `cd .ros`
5. `mkdir camera_info` (if the camera_info directory does not already exist)
6. `nano head_camera.yaml`
7. Now, copy and paste the contents of cr_ros_3/head_camera.yaml into your nano editor.
8. save your file.
9. catkin make

## Using the package

1. ssh into your robot
2. launch the onboard launch file.
3. let it complete.
4. launch the offboard launch on your remote pc
5. give it a minute - it may take some time before everything is ready to go.

The offboard launch is modular, and has modules that can disabled from the command line. The variables are:

1. `fids` for fiducials
2. `voice` for alexa related voice control
3. `waypoints` for action narration (turns the robot into a chatty cathy)

Example: To disable fiducials, do the following:

``` sh
roslaunch cr_ros_3 mutant_offboard_rpicam.launch fids:=FALSE
```

## How to add a new node

Read [the state interface guide](HOW_TO_USE_STATE_INTERFACE.md) to learn the zen of interfacing with the state manage and what tools are available to you in this package.

## Popular Topics

### Facial recognition

Two different attempts have been made at facial recognition so far.

#### Gen 2

See `greet.py` in cr_ros_2. It has been removed from cr_ros_3

#### Gen 3

there is another repositiory in the campusrover group called [Robotic Computer Vision](https://github.com/campusrover/Robotics_Computer_Vision)
running the `detect.py` script in that package will begin publishing facial recognition information. **NB** `detect.py` is not a ROS node but it will publish a ROS topic. Therefore you can run `detect.py` in the terminal by using `python`, not `rosrun`
`detect.py` communicates with `go_to_person.py` in cr_ros_2. `go_to_person.py` has been removed from cr_ros_3.  
`detect.py` requires a discrete GPU, and should probably not be run alongside other nodes on the same remote PC.

### Hand gestures

*We are currently in process of retrieving the gen 3 source code for hand gesture publishing from it's author*

## State of the package report

### NOTE TO CONTRIBUTORS: please update this section after every sizable commit!

12/9/19

* This package is now 100% Alien compatible.
* Alien launch files can now be used as universal launch files
* Install scripts added to make it easier to begin using this package from scratch
* README overhauled to accomdate new users and new procedures
* cr_web can now use a diiferent web browser from .bashrc rather than editing web_launcher.sh.  

Things that should/need to be done:

* continue improving documentation - especially with regards to the alexa voice integration (instructions for launch, setup, etc)
* remove mutant launches and rename mutant_navigation to rover_navigation
