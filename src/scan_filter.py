#!/usr/bin/env python

# This file replaces the 'nan' value in the /scan topic with 9.90 and publishes to /scan_filtered
import rospy
from sensor_msgs.msg import LaserScan
import math

rospy.init_node('scan_filter')

publisher = rospy.Publisher('scan_filter', LaserScan, queue_size=1)


def filterRange(ranges, intensities):
    ranges = list(ranges)
    intensities = list(intensities)
    for i,value in enumerate(ranges):
        if value < 0.2:
            ranges[i] = 0
    for i in range(125, 145):
        ranges[i] = 0
        intensities[i] = 0

    for i in range(35, 55):
        ranges[i] = 0
        intensities[i] = 0

    for i in range(305, 325):
        ranges[i] = 0
        intensities[i] = 0

    for i in range(215, 235):
        ranges[i] = 0
        intensities[i] = 0

    return ranges, intensities


def subCallback(msg):
    msg.ranges, msg.intensities = filterRange(msg.ranges, msg.intensities)
    publisher.publish(msg)


subscriber = rospy.Subscriber('scan', LaserScan, subCallback)

rospy.spin()
