#!/usr/bin/env python

# description: a node which uses the robot's current estimated pose to determine
# the nearest closest pose from a list of preset waypoints

# new to cr_ros_2:
# namespaced

import rospy
import json
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

def cb(msg):
    global last_wp, wps, pub
    pos = complex(msg.pose.pose.position.x, msg.pose.pose.position.y)
    # Get closest waypoint by Euclidian distance
    closest_pos = min(wps, key=lambda k: abs(pos - k))  # calculate the closest waypoint
    if last_wp != wps[closest_pos]:  # If the closest waypoint is new, publish
        last_wp = wps[closest_pos]
        pub.publish(last_wp)
        rospy.loginfo('Closest waypoint: {0} ({1:.2f}, {2:.2f})'.format(
            last_wp,
            closest_pos.real,
            closest_pos.imag))

last_wp = None
wps = {}
# Create dict from map coordinates (complex) to waypoint names
with open(os.path.dirname(__file__) + '/../files/waypoints.json') as f:
    for wp in json.load(f):
        wps[complex(wp['location']['x'], wp['location']['y'])] = wp['name']

rospy.init_node('whereabouts')
# Publishes name of nearest waypoint as a String to /nearest_waypoint
# Only publishes when the nearest waypoint changes from the previous one
pub = rospy.Publisher('nearest_waypoint', String, queue_size=1)
sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, cb)  # uses amcl to help determine position
rospy.spin()
