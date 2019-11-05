#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# new to cr_ros_2:
# removed things pertaining to kobuki base and docking (safely)
# changed cmd_vel topic to work for tb3
from __future__ import division

import rospy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import UInt8

import sys, select, termios, tty, time

from all_states import *
import actionlib
from move_base_msgs.msg import MoveBaseAction

moveBindings = {
        '1':(1,0),
        '3':(0,1),
        '4':(0,-1),
        '2':(-1,0),
           }

speedBindings={}

buffer_key = None

def setKeyCb(msg):
    global buffer_key
    buffer_key = str(msg.data)


def getKey():
    global buffer_key
    if buffer_key is not None:
        key = buffer_key
        buffer_key = None
        return key
    return None

speed = .4
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def teleop_state():
    global was_lost, got_pose_estimate
    cancel_goals()
    prev_state = get_state()
    if prev_state is States.LOST or prev_state is States.FLYING:
        was_lost = True
        got_pose_estimate = False

    change_state(States.TELEOP)

def cancel_goals():
    prev_state = get_state()
    if prev_state is States.NAVIGATING:
        move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_client.wait_for_server()
        move_client.cancel_all_goals()
        rospy.loginfo("Cancelling nav goals")
    else:
        print "Can't cancel nav goals! State was " + prev_state.value

def stop(control_speed, control_turn):
    num_twists = 10
    sleep_time = 1/num_twists
    speed_diff = control_speed/num_twists
    turn_diff = control_turn/num_twists
    twist = Twist(linear=Point(x=control_speed), angular=Quaternion(z=control_turn))
    for i in range(0, num_twists):
        twist.linear.x -= speed_diff
        twist.angular.z -= turn_diff
        pub.publish(twist)
        time.sleep(sleep_time/2)

    pub.publish(Twist())
    if was_lost and not got_pose_estimate:
        new_state = States.LOST
    else:
        new_state = States.WAITING

    change_state(new_state)

def pose_cb(msg):
    global got_pose_estimate
    got_pose_estimate = True

rospy.init_node('turtlebot_teleop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
sub = rospy.Subscriber('web/teleop', UInt8, setKeyCb)
pose_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, pose_cb)

x = 0
th = 0
target_speed = 0
target_turn = 0
control_speed = 0
control_turn = 0
rate = rospy.Rate(10)
was_lost = False

while not rospy.is_shutdown():
    key = getKey()
    if key is None:
        if control_speed == 0 and control_turn == 0:
            rate.sleep()
            continue
    elif key in moveBindings.keys():
        if get_state() is not States.TELEOP:
            teleop_state()
        x = moveBindings[key][0]
        th = moveBindings[key][1]
        count = 0
    elif key == '0':
        cancel_goals()
        stop(control_speed, control_turn)
        x = 0
        th = 0
        control_speed = 0
        control_turn = 0
        continue

    else:
        rate.sleep()
        continue

    target_speed = speed * x
    target_turn = turn * th

    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.02 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.02 )
    else:
        control_speed = target_speed

    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.1 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.1 )
    else:
        control_turn = target_turn

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    pub.publish(twist)
    rate.sleep()

def shutdown():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

rospy.on_shutdown(shutdown)
