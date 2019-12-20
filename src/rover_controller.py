#! /usr/bin/env python

# new to cr_ros_3:
# remove namespacing
# this node handles communication between campus rover, the flask web app,
# and provides move base + change state callbacks to topics 

import rospy
import os
import json
import actionlib
import random
import time
import numpy as np
import tf
from std_msgs.msg import UInt8, String, Header, Bool
from pose_converter import PoseConv
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Pose, Point, \
    Quaternion, PoseStamped, Transform, Vector3, TransformStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from cr_ros_3.srv import Talk
from std_srvs.srv import Empty
from cr_ros_3.msg import ThingsToSay
from all_states import *
from state_tools import *

cam_topic = rospy.get_param("/rover_controller/cam_topic")

# possible navigation statuses 
FEEDBACK_STATUS = {
    "0":'PENDING', "1":'ACTIVE', "2":'PREEMPTED',
    "3":'SUCCEEDED', "4":'ABORTED', "5":'REJECTED', "6":'PREEMPTING',
    "7":'RECALLING',"8":'RECALLED',"9":'LOST',
    }

def im_cb(msg):
    global last_im_pub
    last_im_pub = msg
    if last_im_pub is not None and time.time() - last_im_pub  < .1:
     return

    last_im_pub = time.time()
    web_camera_pub.publish(msg)

def nav_cb(feedback):

    # stats callback for move base service
    status = FEEDBACK_STATUS[str(move_client.get_state())]
    loc = feedback.base_position.pose.position
    rospy.logdebug("Navigation in state {}, at point ({},{})".format(status,loc.x,loc.y))

last_im_pub = None

def done_cb(goal_status, done_result):
    global goal

    # change state back to waiting
    demand_state_change('waiting')
    nav_status = "Navigation {}".format(FEEDBACK_STATUS[str(goal_status)])
    rospy.loginfo(nav_status)

    # say whether the navigation was successful or not
    if goal_status == 3:
        talker(nav_status, talker_pub)
    elif goal_status == 4:
        talker("Give me a minute, I'm feeling a little tired", talker_pub)

        # clear costmap and re-sends goal
        costmap_clearer()
        move_client.send_goal(goal, feedback_cb=nav_cb, done_cb=done_cb)
        rospy.loginfo("Goal re-sent to move base navigator")
        demand_state_change('navigating')


def teleop_cb(msg):
    teleop_pub.publish(msg)


def destination_cb(msg): # input PoseStamped

    # set goal pose as input PoseStamped
    global goal
    goal_pose = msg
    goal = MoveBaseGoal(goal_pose)

    # send goal to move base and change state to navigating 
    move_client.send_goal(goal, feedback_cb=nav_cb, done_cb=done_cb)
    rospy.loginfo("Goal sent to move base navigator")
    demand_state_change('navigating')


def web_destination_cb(msg): # input json

    # get destination from json
    destination = json.loads(msg.data.replace("u\'","\"").replace("\'","\""))
    rospy.loginfo("New destination received from web server: {}".format(destination["name"]))

    # create PoseStamped to send to regular destination topic => callback
    goal_point = Point(destination["location"]["x"], destination["location"]["y"], destination["location"]["z"])
    goal_orientation = Quaternion(destination["orientation"]["x"], destination["orientation"]["y"], destination["orientation"]["z"], destination["orientation"]["w"])
    goal_pose_unstamped = Pose(goal_point, goal_orientation)
    goal_pose = PoseStamped(Header(),goal_pose_unstamped)
    goal_pose.header.stamp = rospy.get_rostime()
    goal_pose.header.frame_id = 'map'

    # pass off as PoseStamped
    destination_pub.publish(goal_pose)

# initialize node and set no goal
rospy.init_node('rover_controller')
goal = None

# web topics
web_teleop_sub = rospy.Subscriber('web/teleop', UInt8, teleop_cb)
web_destination_sub = rospy.Subscriber('web/destination', String, web_destination_cb)
destination_pub = rospy.Publisher('destination', PoseStamped, queue_size=1)
destination_sub = rospy.Subscriber('destination', PoseStamped, destination_cb)
web_camera_pub = rospy.Publisher('web/camera', CompressedImage, queue_size=1)
web_state_pub = rospy.Publisher('web/state', String, queue_size=1)
web_map_pub = rospy.Publisher('web/map', OccupancyGrid, queue_size=1, latch=True)

# subscribers
image_sub =  rospy.Subscriber(cam_topic, CompressedImage, im_cb)

# publishers
teleop_pub = rospy.Publisher('teleop_keypress', UInt8, queue_size=100)
talker_pub = rospy.Publisher('/things_to_say', ThingsToSay, queue_size=1)

#service proxys
costmap_clearer = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

# wait for move base server 
move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
move_client.wait_for_server()

rospy.spin()
