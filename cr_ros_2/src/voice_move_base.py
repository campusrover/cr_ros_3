#! /usr/bin/env python
import rospy
import actionlib
import actionlib_tutorials.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

# init node and sub
rospy.init_node('voice_move_base')

def voice_move_base_client(request):

    # create simple move base client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # wait for move base server
    rospy.loginfo("WAITING FOR MOVE BASE SERVER")
    client.wait_for_server()
    rospy.loginfo("MOVE BASE SERVER FOUND")

    # create goal and send to move base client
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'first_move'
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)

    # wait for and log results
    client.wait_for_result()
    rospy.loginfo(client.get_result())
    rospy.loginfo("MOVE BASE GOAL FINISHED")

def nearest_waypoint(request):


# voice intent subscriber, move base publisher, and nearest waypoint cb
voice_sub = rospy.Subscriber('/voice_intents', String, voice_move_base_client)
move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
waypoint_cb = rospy.Subscriber('/nearest_waypoint', String, nearest_waypoint)




if __name__ == '__main__':

    # wait for voice intent callback
    rospy.loginfo("WAITING FOR VOICE INTENT")
    rospy.spin()
    

# #!/usr/bin/env python
# import rospy
# import math
# import os
# import json
# import actionlib
# import random
# import time
# import numpy as np
# import tf
# from std_msgs.msg import UInt8, String, Header, Bool
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import Twist, Pose, Point, \
#     Quaternion, PoseStamped, Transform, Vector3, TransformStamped
# from nav_msgs.msg import OccupancyGrid
# from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
# from cr_ros_2.srv import Talk
# from all_states import *

# rospy.init_node('voice_move_base')

# def publish_dest(destination):
#     rospy.loginfo('going to {}'.format(destination))

#     # get pose from pose dictionary
#     goal_pose_unstamped = {
#     "cafe" : Pose(Point(13.6, 27.6, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
#     "tims office" : Pose(Point(15.5, 21.1, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
#     "jordans office" : Pose(Point(16.5, 27.6, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
#     "main door" : Pose(Point(19.0, 20.7, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

#     # default destination as Pito's office
#     }.get(destination, Pose(Point(18.6, 23.2, 0.0), Quaternion(0, 0, 0.8509035, 0.525322)))

#     # stamp pose
#     goal_pose = PoseStamped(Header(),goal_pose_unstamped)
#     goal_pose.header.stamp = rospy.get_rostime()
#     goal_pose.header.frame_id = 'map'

#     return goal_pose


# destination_pub = rospy.Publisher('move_base/goal', PoseStamped, queue_size=1)

# user_input = ""

# while(user_input != "exit"):
#     user_input = raw_input("Where would you like to go? ")
#     print user_input
#     publish_dest(user_input)

# rospy.spin()


