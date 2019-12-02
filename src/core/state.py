#!/usr/bin/env python

# Description: a service which handles changes between states. can publish
# navigation goal poses if needed (only when new state is navigating)

# changes; commented out lines that have to do with "talking" as mutant does not \
# have a speaker as of 3/19/19
# --> added namespacing to topics 

import rospy
import numpy as np

from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped

from all_states import *
from cr_ros_3.srv import *

def handle_state_change(req):
    global current_state
    old_state = current_state
    new_state = States[req.new_state]
    rospy.logdebug("Attempting state change from {} to {}".format(old_state.value, new_state.value))

    is_legal = is_legal_change(new_state)

    if is_legal:
        rospy.loginfo("State changed successfully from {} to {}".format(old_state.value, new_state.value))
        current_state = new_state
        # Publish pose if one given, only state that requires a pose is NAVIGATING
        if req.pose_to_pub != States.DUMMY_POSE.value and \
            new_state == States.NAVIGATING:
            navigate(req.pose_to_pub)
        # Make announcement if one given
        if req.to_say != States.DUMMY_STRING.value:
            say(req.to_say)
    else:
        # Illegal state change, switch to illegal state
        rospy.logerr("Illegal state change attempted from {} to {}".format(old_state.value, new_state.value))
        current_state = States.ILLEGAL_STATE_CHANGE

    state_pub.publish(current_state.value)
    return StateChangeResponse(is_legal)

def handle_state_query(req):
    global current_state
    return StateQueryResponse(current_state.value)

def is_legal_change(new_state):
    return current_state == new_state or new_state in legal_state_changes[current_state]

def navigate(pose):
    global nav_pub, goal_pose_pub
    header = Header(frame_id='/map', stamp=rospy.Time())
    pose_stamped = PoseStamped(pose=pose, header=header)
    nav_pub.publish(pose_stamped)
    goal_pose_pub.publish(req.pose_to_pub)

def say(str):
    global talk_srv
    talk_srv(str)

rospy.init_node('state_manager')
current_state = States.LOST  # Initial state of robot

state_change = rospy.Service('state_change', StateChange, handle_state_change)
state_query = rospy.Service('state_query', StateQuery, handle_state_query)

nav_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
goal_pose_pub = rospy.Publisher('goal_pose_for_fids', Pose, queue_size=1)
state_pub = rospy.Publisher('state', String, queue_size=1, latch=True)


talk_srv = rospy.ServiceProxy('say', Talk)

state_pub.publish(current_state.value)
rospy.spin()
