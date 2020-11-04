#!/usr/bin/env python

import rospy, math, json
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# constants
LINEAR_SPEED  = 0.22        # m/s
ANGULAR_SPEED = math.pi/4   # r/s

# get rotation command 
def get_rotation(time_elapsed):
    sign = 1 if intent_details.get("direction") == "right" else -1
    total_time = get_rotation_duration()
    if time_elapsed < total_time:
        t = Twist()
        t.angular.z = ANGULAR_SPEED * sign
        return t
    return Twist()


# get translation command 
def get_translation(time_elapsed):
    sign = 1 if intent_details.get("direction") == "forward" else -1
    total_time = get_translation_duration()
    if time_elapsed < total_time:
        t = Twist()
        t.linear.x = LINEAR_SPEED * sign
        return t
    return Twist()


# get duration of angular command
def get_rotation_duration():
    angle = intent_details.get("angle")
    amount = float(angle.get("amount"))
    units = angle.get("units")
    if units == "degrees":
        amount = math.radians(amount)
    seconds = amount/ANGULAR_SPEED
    return rospy.Duration(secs=seconds)


# get duration of linear command
def get_translation_duration():
    distance = intent_details.get("distance")
    amount = float(distance.get("amount"))
    units = distance.get("units")
    if units == "feet":
        amount = amount * 0.3048
    seconds = amount/LINEAR_SPEED
    return rospy.Duration(secs=seconds)


# callback to parse intent json and update global vars
def perform_action_from_intent(message):

    # record time intent was received 
    global time_received; global intent_details; global action_type
    time_received = rospy.Time.now()

    # parse action and details fields from intent json
    intent = json.loads(message.data)
    action_type = intent['action']
    intent_details = intent['details']


# get command vel for each action type
def get_vel(time_elapsed):

    # call correct method for each action type
    if action_type == "navigation.translation":
        return get_translation(time_elapsed)
    elif action_type == "navigation.rotation":
        return get_rotation(time_elapsed)
    else:
        return Twist()


# init node and declare pub/subs
rospy.init_node('voice_intent_handler')
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
intent_sub = rospy.Subscriber('voice_intents', String, perform_action_from_intent)
rate = rospy.Rate(10)

# some variables
time_received = rospy.Time.now()
intent_details = None
action_type = None

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

    # calculate time since last intent
    time_elapsed = rospy.Time.now() - time_received
    
    # publish cmd_vel based on last intent
    cmd_vel_pub.publish(get_vel(time_elapsed))

    # run at 10 hz
    rate.sleep()