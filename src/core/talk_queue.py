#!/usr/bin/env python

# description: listens to the /things_to_say topic, holds them in a queue, and sends them to
# the service server in talk.py
# should probably have a better, more descrptive name than message switch

import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from cr_ros_3.srv import Talk
from cr_ros_3.msg import ThingsToSay
import time

def callback(msg):
    global say_queue
    say_queue[msg.say_at] = msg.to_say

rospy.init_node('talk_queue')
sub = rospy.Subscriber('/things_to_say', ThingsToSay, callback)
talk_srv = rospy.ServiceProxy('say', Talk)

say_queue = {}

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if not say_queue:
        rate.sleep()
        continue
    t = time.time()
    for sq in sorted(k for k in say_queue if k < t):
        talk_srv(say_queue[sq])
        del say_queue[sq]
        rate.sleep()
