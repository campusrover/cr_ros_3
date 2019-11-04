#!/usr/bin/env python
import rospy
from all_states import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool, Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion
from math import pi, sin, cos

def odom_cb(msg):  # gets the current position of the robot and it's orientation from odom
    global curr_z, curr_point
    curr_point = msg.pose.pose.position # has fields x, y, z
    quat = msg.pose.pose.orientation # has fields x, y, z, w
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    euler = euler_from_quaternion(quat_list)
    curr_z = euler[2] # current z angle in radians


def cv_cb(msg):
    global curr_z, curr_point, see_person, goal_pose, report
    if get_state() == States.SEARCH:
        str_list = msg.data   # data is a string in the format: '['name', angle, distance]'
        list = str_list[1:-1].split(',')
        # list = msg.split(',')
        list[0].strip('[').strip('\'') # the 'class' (name of person) from the cv node
        rospy.loginfo('human seen: name = {} at angle {} and {} meters away'.format(list[0], list[1], list[2]))
        list[2].strip(']')
        list[1] = float(list[1]) # angle to the person (in degrees)
        list[2] = float(list[2]) # distance to person (meters)
        if report:
            talk_srv('{}, I have your order'.format(list[0]))
            report = False
        """
        # TODO: use the angle and distance to calculate a goal pose based on the current pose of the robot (will probably require odometry subscription)
        # check to make sure the current state is WAITING
        # then, publish that goal pose to the destination publisher.
        angle_change = list[1] * 180/pi # convert given change in angle to radians
        target_angle = curr_z + angle_change
        delta_x = sin(target_angle) * list[2]
        delta_y = cos(target_angle) * list[2]
        new_point = Point()
        new_point.x = curr_point.x + delta_x
        new_point.y = curr_point.y + delta_y
        new_point.z = curr_point.z

        goal_pose_unstamped = Pose(new_point, Quaternion(0.0, 0.0, 1.0, 0.0))
        # stamp pose
        goal_pose = PoseStamped(Header(), goal_pose_unstamped)
        goal_pose.header.stamp = rospy.get_rostime()
        goal_pose.header.frame_id = 'map'
        # TODO: need to publish goal poseStamped, only is state == waiting
        """
        see_person = True

def pack_cb(msg):
    global has_package
    has_package = msg.data



rospy.init_node('go_to_person')
report = True
curr_z = 0
curr_point = None
has_package = False
see_person = False
spinning = False
goal_sent = False
goal_pose = None
dest_pub = rospy.Publisher('destination', PoseStamped, queue_size=1)
cv_sub = rospy.Subscriber('face_detection', String, cv_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
spin_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
package_sub = rospy.Subscriber('has_package', Bool, pack_cb)
talk_srv = rospy.ServiceProxy('say', Talk)

rate = rospy.Rate(3)
spin_twist = Twist()
spin_twist.angular.z = 0.05
while not rospy.is_shutdown():
    if get_state() == States.WAITING and has_package: # look for person, reset variables
        change_state(States.SEARCH)
        goal_sent = False
        see_person = False
        report = True
    elif get_state() == States.SEARCH and not see_person and not spinning:
        spin_pub.publish(spin_twist)
        spinning = True # only publish once - it will latch and the robot will spin until a new cmd_vel is published

    if see_person and not goal_sent: # sneds goal pose (once) to move closer to the person
        # rospy.loginfo('going to point {}, {} from '.format(goal_pose.pose.position.x, goal_pose.pose.position.y, curr_point.x, curr_point.y))
        spin_pub.publish(Twist()) # stop moving - publishes blank twist
        spinning = False
        # dest_pub.publish(goal_pose)
        goal_sent = True

    if get_state() == States.SEARCH and not has_package: # if the package has been taken, then go back to waiting
        change_state(States.WAITING)
    #TODO: additional logic that stops spin when person is found
