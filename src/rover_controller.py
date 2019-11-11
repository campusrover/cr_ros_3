#! /usr/bin/env python

# new to cr_ros_2:
# namespacing
# removed TB@ docking junk
# this node handles communication between campus rover and flask web app

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
from all_states import *
from state_tools import *

FEEDBACK_STATUS = {
    "0":'PENDING', "1":'ACTIVE', "2":'PREEMPTED',
    "3":'SUCCEEDED', "4":'ABORTED', "5":'REJECTED', "6":'PREEMPTING',
    "7":'RECALLING',"8":'RECALLED',"9":'LOST',
    }

def im_cb(msg):
    global last_im_pub
    if last_im_pub is not None and time.time() - last_im_pub  < .1:
     return

    last_im_pub = time.time()
    web_camera_pub.publish(msg)

def nav_cb(feedback):
    status = FEEDBACK_STATUS[str(move_client.get_state())]
    loc = feedback.base_position.pose.position
    rospy.logdebug("Navigation in state {}, at point ({},{})".format(status,loc.x,loc.y))

last_im_pub = None

def done_cb(goal_status, done_result):
    demand_state_change('waiting') #change_state(States.WAITING)
    nav_status = "Navigation {}".format(FEEDBACK_STATUS[str(goal_status)])
    rospy.loginfo(nav_status)
    if goal_status == 3:
        say(nav_status)
    elif goal_status == 4:
        say('Give me a minute, I\'m feeling a little tired')

def teleop_cb(msg):
    teleop_pub.publish(msg)

def is_dock_pose(msg):
    if msg.pose.position.x == 21.0 and msg.pose.position.y == 21.5:
        return True
    else:
        return False

def destination_cb(msg): # input PoseStamped
    global nav_controller

    ## a failed attempt to pass the dock pose and tranform desired drive pose from that. Not used.
    # if is_dock_pose(msg):
    #     goal_pose = get_docking_destination(msg)
    # else:
    #     goal_pose = msg

    goal_pose = msg

    goal = MoveBaseGoal(goal_pose)
    move_client.send_goal(goal, feedback_cb=nav_cb, done_cb=done_cb)
    rospy.loginfo("Goal sent to move base navigator")
    demand_state_change('navigating') #change_state(States.NAVIGATING)


def web_destination_cb(msg): # input json
    # clean input
    destination = json.loads(msg.data.replace("u\'","\"").replace("\'","\""))
    rospy.loginfo("New destination received from web server: {}".format(destination["name"]))

    goal_point = Point(destination["location"]["x"], destination["location"]["y"], destination["location"]["z"])
    goal_orientation = Quaternion(destination["orientation"]["x"], destination["orientation"]["y"], destination["orientation"]["z"], destination["orientation"]["w"])
    goal_pose_unstamped = Pose(goal_point, goal_orientation)
    goal_pose = PoseStamped(Header(),goal_pose_unstamped)
    goal_pose.header.stamp = rospy.get_rostime()
    goal_pose.header.frame_id = 'map'

    # pass off as PoseStamped
    destination_pub.publish(goal_pose)

rospy.init_node('rover_controller')
temp_pose_pub = rospy.Publisher('temp_pose',PoseStamped,queue_size=1)


# web topics
web_teleop_sub = rospy.Subscriber('web/teleop', UInt8, teleop_cb)
web_destination_sub = rospy.Subscriber('web/destination', String, web_destination_cb)
destination_pub = rospy.Publisher('destination', PoseStamped, queue_size=1)
destination_sub = rospy.Subscriber('destination', PoseStamped, destination_cb)
web_camera_pub = rospy.Publisher('web/camera', CompressedImage, queue_size=1)
web_state_pub = rospy.Publisher('web/state', String, queue_size=1)
web_map_pub = rospy.Publisher('web/map', OccupancyGrid, queue_size=1, latch=True)
# subscribers
image_sub =  rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, im_cb)

# publishers
teleop_pub = rospy.Publisher('teleop_keypress', UInt8, queue_size=100)
say = rospy.ServiceProxy('say', Talk)




move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
move_client.wait_for_server()

rospy.spin()
