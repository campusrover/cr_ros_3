#!/usr/bin/env python
import rospy
from all_states import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, Point, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion

def hand_cb(msg):
    global lock_current_goal, current_goal, acknowledge_time
    rospy.loginfo(msg.data)
    # print(msg.data)
    # print(type(msg.data))
    if msg.data == "Stop":
        rospy.loginfo('I should stop')
        #print("fuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuckfuck")
        if get_state() == States.NAVIGATING and rospy.Time.now().secs > acknowledge_time:
            move_client.cancel_all_goals()
            lock_current_goal = True
            talk_srv('Hi human, you have my attention for ten seconds')
            change_state_time = rospy.Time.now().secs + 10
            while rospy.Time.now().secs < change_state_time:
                continue
            lock_current_goal = False
            dest_pub.publish(current_goal)
            talk_srv('You bore me. Time for me to carry on. Tally ho')
            acknowledge_time = rospy.Time.now().secs + 100


def save_goal(msg):
    global current_goal, lock_current_goal
    if not lock_current_goal:
        current_goal = msg



current_goal = None
lock_current_goal = False

rospy.init_node('hand_gesture')
acknowledge_time = rospy.Time.now().secs
dest_pub = rospy.Publisher('destination', PoseStamped, queue_size=1)
goal_sub = rospy.Subscriber('destination', PoseStamped, save_goal)
talk_srv = rospy.ServiceProxy('say', Talk)
hand_sub = rospy.Subscriber('hand_command', String, hand_cb)
rospy.loginfo("node initialized")


move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
move_client.wait_for_server()

rospy.spin()
