#!/usr/bin/env python

# description: listens to /nearest_waypoint (published by whereabouts.py) and
# publishes what they are doing (from state manager) along with the location
# to the things to say topic for message_switch to say

import rospy
from std_msgs.msg import String
from cr_ros_3.msg import ThingsToSay
from all_states import get_state
import time

def update_location(msg):
    global location
    location = msg.data

loc_sub = rospy.Subscriber('/nearest_waypoint', String, update_location)
talker_pub = rospy.Publisher('/things_to_say', ThingsToSay, queue_size=1)

location = None

rospy.init_node('location_narration')

while not rospy.is_shutdown():
    if location is None: continue
    doing = get_state().data.replace('_', ' ').lower()
    rospy.loginfo('Robot is {0} near {1}'.format(doing, location))
    talker_pub.publish(ThingsToSay(
        say_at=time.time(),
        to_say='I am {0} near {1}'.format(doing, location)
    ))
