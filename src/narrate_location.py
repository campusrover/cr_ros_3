#!/usr/bin/env python

# description: listens to /nearest_waypoint (published by whereabouts.py) and
# publishes what they are doing (from state manager) along with the location
# to the things to say topic for message_switch to say

import rospy
from std_msgs.msg import String
from cr_ros_3.msg import ThingsToSay
from all_states import *
from state_tools import talker
import time

def update_location(msg):
    global location
    location = msg.data

loc_sub = rospy.Subscriber('/nearest_waypoint', String, update_location)
talker_pub = rospy.Publisher('/things_to_say', ThingsToSay, queue_size=1)

location = None
prev_location = None
prev_doing = None

rospy.init_node('location_narration')
rate = rospy.Rate(.5)

while not rospy.is_shutdown():
    doing = str(get_state()).strip('States.').replace('_', ' ').lower()
    if location is None or (location == prev_location and doing == prev_doing):  # if the combination of state + waypoint has not changed, then do nothing
        continue
    else:
        
        rospy.loginfo('Robot is {0} near {1}'.format(doing, location))  # log behavior
        talker('I am {0} near {1}'.format(doing, location), talker_pub)   # declare behavior
        prev_location = location  # update location
        prev_doing = doing  # update doing
    rate.sleep()