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