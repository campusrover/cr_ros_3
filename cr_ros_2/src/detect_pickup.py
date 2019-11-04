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
airborne_pub = rospy.Publisher('airborne', Bool, queue_size=1) # true if roboot is in air, false if not
min_acc_z = 100  # start min high initially
""" removed "calibration" in favor of hard-coded thresholds
# gloabl variables for acceleration in the z direction
global min_acc_z
global max_acc_z
global avg_acc_z  # average is admittedly not used, but it might be nice to have around.

z_acceleration = 69  # this seems to be the ballpark that the IMU normally reads

max_acc_z = 0
avg_acc_z = 0
samples = 0

time.sleep(4)  # wait for imu to begin publishing - wait time is an estimate, and not consistant
start_time = rospy.Time.now()
print("[PICKUP DETECTOR]: Do not move robot! calibrating pickup")
# spend 7 seconds to generate an idea of the normal range of values the IMU reads for z acceleration (it's not constant)
while rospy.Time.now() < start_time + rospy.Duration(4):
    print('curr z acc: ', z_acceleration)
    if not z_acceleration == 69:
        curr_z = z_acceleration
    #print(curr_z, rospy.Time.now() - start_time)
        if curr_z < min_acc_z:
            min_acc_z = curr_z
        if curr_z > max_acc_z:
            max_acc_z = curr_z
        if not avg_acc_z == 0:
            avg_acc_z = (avg_acc_z * samples + curr_z)/(samples + 1)
            samples += 1
        else:
            avg_acc_z = curr_z

diff = (avg_acc_z - min_acc_z)*1.5 # increases the dead zone range
print("[PICKUP DETECTOR]: finished calibration: min: {}, avg: {} max: {} diff{}".format(min_acc_z, avg_acc_z, max_acc_z, diff))
"""
rate = rospy.Rate(10)
airborne_pub.publish(False) # initially on the ground
flying = False
# continue reading data, print direction that the robot thinks it is going
start_time = None
# important note: z_acceleration picks up acceleration due to gravity (~9.8) normally. acceleration > ~9.8 mean the robot is falling faster than gravity.
# acceleration < 9.8 means the robot is either slowing it's fall - or accelerating upwards
while not rospy.is_shutdown():
    curr_z = z_acceleration
    if curr_z > 11: # 11 replaced max_acc_z + diff
        if flying:
            print("down: ", curr_z)
        start_time = rospy.Time.now()  # reset start time
        #airborne_pub.publish(True)
    elif curr_z < 7.5:  # 7.5 replaced max_acc_z + diff
        print("up: ", curr_z)
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
    airborne_pub.publish(flying)
    rate.sleep()
