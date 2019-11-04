### Campus Rover Mark 2!!!!

TODO: convert all mark 1 nodes to this mark 2 repo

EC 3/7:
Currently, it may not work -- the launch files are depending on files that exist in the turtlebot3_navigation directory.
So we may need to replicate all the files that are in those directories. Just didn't get a chance to test.

ND 3/17:
* greet.py and offboard_launch.launch need camera image topic updated to work w/ mutant
* compressed images from the kinect camera might be needed for above, not currently being published
* all laptops planning to work with mutant should get aruco_detect added to their catkin_ws (for fiducials)
* working on removing everything related to turtlebot2 docking
* working on updating dependent file paths from turtlebot to turtlebot3
* deciding which nodes do not get included - currently anything related to speech is getting culled b/c mutant does not have a speaker (yet)
* need to find a new way for lost_and_found.py to detect that the robot has been lifted (suggestion: add an IR sensor to mutant that points at the ground, that should be readable from the topic /mutant/sensor_state.cliff)

ND 3/19:
* entire lab experiencing internet problems...
* cr_ros_2 repo needs to be cloned to mutant
* aruco detect needs to be installed on mutant
* (roadblock: why the last two haven't been done: mutant is having other problems that need to be solved first...)

ND 3/26
* *This is the week that we finish this awful job*
*  goal for this week: ([!] denotes that there are incompatibilities in the node which need to be amended)
an onboard launch file which launches nodes:

[nodes that have been tested already]
cpu_checker
scan_filter
pickup detector

[nodes that need testing]
talk
state [!] (all_states)
rover_controller
(recording sender)

an offboard launch file which launches:
mesasge switch
teleop [!]
lost and found [!]
process fid transoforms
package sender
package handler

clean up launch files

ND 3/29:
* all files namespaced
* cr_web has new branch mutant w/ namespacing to communicate w/ move_base_mutant
* package handler and package sender will be disabled until mutant has a button/ sensor
installed that can detect stolen packages
* BIG NOTE: anyone who would like to use gen2's software on mutant should have the fiducials
package and cr_web (mutant branch) cloned into their catkin workspace
* updates have not been pulled to mutant yet - that needs to be done

ND 4/19
Update on the last few weeks of work -
* raspi camera has been decided as the camera of choice. all kinect related launch files have been removed
* raspi camera image is throttled properly with topic tools
* aruco detect subscribes to throttled camera topic, published fid_transforms to a non-namespaced topic (will change soon) *NB* use of this package requires that the aruco detect/ fiducials package is installed on your remote pc
* mutant_navigation and move_base_mutant have been cleaned up significantly, made to resemble turtlebot3_navigation defaults. *NB* use of this package requires that turtlebot3_navigation is installed on your remote pc.
* mutant_navigation has been tested as a stand-alone offboard launch. mutant_navigation is included in mutant_offboard, but mutant_offboard has not been tested with navigation yet.
* rviz_settings is hard-coded to work with mutant. . . perhaps in the future we can find a way for rviz_settings to namespace properly?? (side note: in the final version, rviz shouldn't need to run... it's just useful for debugging)
* cr_web launches web app alongside offboard launch. *NB* use of this package requires that cr_web and google chrome are installed on your remote pc, along with the python flask module.
* cr_web mutant branch is almost, if not, fully operational - last thing to check is robot pose on the live map
* no button on the robot means that package handler and package sender have not been updated for mutant/ gen3 use yet.
* pickup detector is behaving poorly - requires more debugging/ improvements.
