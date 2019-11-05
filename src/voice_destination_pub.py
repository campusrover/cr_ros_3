#!/usr/bin/env python
import rospy
import math
import os
import json
import actionlib
import random
import time
import numpy as np
import tf
from std_msgs.msg import UInt8, String, Header, Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Pose, Point, \
    Quaternion, PoseStamped, Transform, Vector3, TransformStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from cr_ros_2.srv import Talk
from all_states import *

rospy.init_node('voice_destination_pub')


def voice_destination_cb(msg): #json from voice_intents

    # load json
    data = json.loads(msg.data.replace("u\'","\"").replace("\'","\""))

    # if the voice intent is a navigation request
    if data.get('action') == 'navigation.arrival':

        # create goal pose based on destination
        goal_pose = get_destination_pose(data.get('details').get('destination'))

        #  publish pose to destinations
        destination_pub.publish(goal_pose)
        print("pose published for " + str(data.get('details').get('destination')))

def get_destination_pose(destination):
    rospy.loginfo('going to {}'.format(destination))

    # get pose from pose dictionary
    goal_pose_unstamped = {
    "cafe" : Pose(Point(13.6, 27.6, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
    "tims office" : Pose(Point(15.5, 21.1, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
    "jordans office" : Pose(Point(16.5, 27.6, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
    "main door" : Pose(Point(19.0, 20.7, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    # default destination as Pito's office
    }.get(destination, Pose(Point(18.6, 23.2, 0.0), Quaternion(0, 0, 0.8509035, 0.525322)))

    # stamp pose
    goal_pose = PoseStamped(Header(),goal_pose_unstamped)
    goal_pose.header.stamp = rospy.get_rostime()
    goal_pose.header.frame_id = 'map'

    return goal_pose


destination_pub = rospy.Publisher('move_base/goal', PoseStamped, queue_size=1)
voice_intents_sub = rospy.Subscriber('voice_intents', String, voice_destination_cb)


rospy.spin()
