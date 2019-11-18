#!/usr/bin/env python

# description: a node meant to replace the kobuki base wheel event present on turtlebot 2.
# uses openCR accelerometer to detect whether the robot has been picked up or not,
# cannot guarantee that the robot has been placed on a different level (e.g. if it is picked up
# off the floor and then put on a table), but neither could the kobuki wheel event.

import rospy
import time
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


def imu_cb(msg):
    global z_acceleration
    z_acceleration = msg.linear_acceleration.z


rospy.init_node("pickup_detector")
imu_sub = rospy.Subscriber("imu", Imu, imu_cb)
airborne_pub = rospy.Publisher('airborne', Bool, queue_size=5) # true if roboot is in air, false if not
min_acc_z = 100  # start min high initially

rate = rospy.Rate(10)
airborne_pub.publish(False) # initially on the ground
flying = False
prev_flying = False
# continue reading data, print direction that the robot thinks it is going
start_time = None
z_acceleration = 9.8
# important note: z_acceleration picks up acceleration due to gravity (~9.8) normally. acceleration > ~9.8 mean the robot is falling faster than gravity.
# acceleration < 9.8 means the robot is either slowing it's fall - or accelerating upwards
while not rospy.is_shutdown():
    curr_z = z_acceleration
    if curr_z > 11: # 11 replaced max_acc_z + diff
        if flying:
            rospy.loginfo("down: {}".format(curr_z))
        start_time = rospy.Time.now()  # reset start time
        #airborne_pub.publish(True)
    elif curr_z < 7.5:  # 7.5 replaced max_acc_z + diff
        rospy.loginfo("up: {}".format(curr_z))
        if not start_time is None and not flying:
            flying = True
        start_time = rospy.Time.now()
        if curr_z < min_acc_z:
            min_acc_z = curr_z
            rospy.loginfo('new low acceleration: {}'.format(min_acc_z)) # log low values - used to debug the acceleration experienced when the robot drives over bumps on the carpet.

    # test to see if the robot is grounded again
    if not start_time is None:
        if rospy.Time.now() > start_time + rospy.Duration(3): # if 3 seconds have passed and no up or down movements detected, assum that you have returned to the ground. 
            print("I think I'm back on the ground now")
            start_time = None
            if flying:
                flying = False
    if not prev_flying == flying:  # if the flying status has changed
        airborne_pub.publish(flying)
    prev_flying = flying
    rate.sleep()
