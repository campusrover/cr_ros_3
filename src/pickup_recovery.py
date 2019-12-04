#!/usr/bin/env python

# new to cr_ros_2:
# --> new topic/ way of handling detected pickup_checker (see function flying_or_lost)
# --> namespaced topics

import rospy
import copy
import numpy as np
import math
from cr_ros_3.srv import Talk
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header, String, Bool
from cr_ros_3.msg import ThingsToSay
#from kobuki_msgs.msg import WheelDropEvent
from geometry_msgs.msg import Twist, Vector3, Point, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from all_states import *
from state_tools import *
import time

# edited callback to work with new pickup detector node
def flying_or_lost(msg):
    global is_lost, lock_current_goal, not_lost_anymore, flying

    # see if airborne
    if msg.data == True:
        if current_state_is('navigating'):  # example of new state tools: more easily readable - code inside if statement executes only in one state
            lock_current_goal = True
            move_client.cancel_all_goals()
        if demand_state_change('flying'):  # exmple of new state tools: forces a state change, then executes code within if statement if state change was successful
            talker("i'm flying!", talker_pub)
            flying = True
    # Else, if robot thinks it is on the ground
    else:
        if current_state_is('flying'):
            demand_state_change('lost')
            not_lost_anymore = False
            flying = False




def publish_pose(pose):
    pose_w_cov_stamped = PoseWithCovarianceStamped()
    pose_w_cov_stamped.pose = PoseWithCovariance(pose=pose, covariance=np.zeros((36)))
    pose_w_cov_stamped.header = Header(frame_id='map', stamp=rospy.Time())
    pose_pub.publish(pose_w_cov_stamped)

def publish_goal(goal):
    header = Header(frame_id='/map', stamp=rospy.Time())
    goal_stamped = PoseStamped(pose=goal, header=header)
    move_client.send_goal_and_wait(MoveBaseGoal(goal_stamped), execute_timeout=rospy.Duration(5))

def reset_vars():
    global not_lost_anymore, need_to_publish_whitespace
    not_lost_anymore = True
    need_to_publish_whitespace = True

    global base_offset, offset_times, polarity, offset
    base_offset = 1
    offset = base_offset
    offset_times = 2
    polarity = -1

    global quats
    quats = [east, north, west, south]

    global goal
    goal = copy.deepcopy(initial_pose.position)
    goal.y = goal.y + offset

def save_goal(msg):
    global current_goal
    if not lock_current_goal:
        current_goal = msg

def is_lost():
    return current_state_is('lost')

def publish_for_second(twist):
    start_time = rospy.Time.now().to_sec()
    while(rospy.Time.now().to_sec() < start_time + 1 and is_lost()):
        spin_pub.publish(twist)
        rate.sleep()

rospy.init_node('pickup_recovery')
""" TODO replace this subscriber """
wheel_drop_sub = rospy.Subscriber('airborne', Bool, flying_or_lost)
pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
spin_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
dest_pub = rospy.Publisher('destination', PoseStamped, queue_size=1)
goal_sub = rospy.Subscriber('destination', PoseStamped, save_goal)
talker_pub = rospy.Publisher('/things_to_say', ThingsToSay, queue_size=1)

rate = rospy.Rate(10)

root_half = math.sqrt(0.5)
east = Quaternion(w=1)
west = Quaternion(z=1)
north = Quaternion(w=root_half, z=root_half)
south = Quaternion(w=root_half, z=-root_half)

flying = False
lock_current_goal = False

initial_pose = Pose(position=Point(x=5, y=5), orientation=south)

reset_vars()

move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#talk_srv('[LOST AND FOUND: LOOKING FOR MOVE BASE SERVER]')
move_client.wait_for_server()
#talk_srv('[LOST AND FOUND: FOUND MOVE BASE SERVER]')
while not rospy.is_shutdown():

    # If the robot isn't lost
    if not is_lost():
        if lock_current_goal: # If we'd been navigating, re-publish that goal
            # header = Header(frame_id='/map', stamp=rospy.Time())
            current_goal.header.stamp = rospy.Time()
            dest_pub.publish(current_goal)
            lock_current_goal = False
            rospy.loginfo("Continuing navigation from before losing pose")

        if not not_lost_anymore: # If we just localized, reset variables for the next time we're lost
            rospy.loginfo("Pose found while scanning")
            talker("I'm not lost anymore", talker_pub)
            spin_pub.publish(Twist())
            reset_vars()

        rate.sleep()
        continue # Wait until lost

    if need_to_publish_whitespace: # If we need to tell the robot that it's in whitespace, publish the pose
        need_to_publish_whitespace = False
        publish_pose(initial_pose)

    rospy.loginfo("Scanning for fiducial")
    talker("Scanning for fiducials", talker_pub)
    for i in range(0, 8): # Wait for a second, then spin 1/8 of a circle, 8 times
        publish_for_second(Twist())
        publish_for_second(Twist(angular=Vector3(z=math.pi*0.25)))

    if not is_lost():
        continue

    # If the spinning didn't find a fiducial, move using AMCL before the next iteration of spinning
    if goal.x == goal.y:
        goal.x = goal.x + offset
    else:
        goal.y = goal.y + offset
        offset = base_offset * offset_times * polarity
        offset_times = offset_times + 1
        polarity = polarity * -1

    publish_goal(Pose(position=goal, orientation=quats[0]))
    quats = np.roll(quats, -1)

    rate.sleep()
