#!/usr/bin/env python

# description: listens to /nearest_waypoint (published by whereabouts.py) and
# publishes what they are doing (from state manager) along with the location
# to the things to say topic for message_switch to say

import rospy
from std_msgs.msg import String
from cr_ros_3.msg import ThingsToSay
from all_states import *
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
    if location is None or (location == prev_location and doing == prev_doing): 
        continue
    else:
        
        rospy.loginfo('Robot is {0} near {1}'.format(doing, location))
        talker_pub.publish(ThingsToSay(
            say_at=time.time(),
            to_say='I am {0} near {1}'.format(doing, location)
        ))
        prev_location = location
        prev_doing = doing
    rate.sleep()