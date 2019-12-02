#!/usr/bin/env python

import rospy
from cr_ros_3.srv import Talk
from std_msgs.msg import Bool, String, Header
from geometry_msgs.msg import Twist, PoseStamped, Pose
from kobuki_msgs.msg import ButtonEvent, DigitalInputEvent
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from all_states import *
from random import choice
import actionlib

# Stack of filenames
packages = []

# Subscribed topic names
release_package_topic = '/release_package'
receive_package_topic = '/receive_package'
record_start_topic = '/record_start'
record_stop_topic = '/record_stop'
physical_package_topic = '/physical_package'

# Response tools
path_to_packages = 'recordings'
say = rospy.ServiceProxy('say', Talk)
header = Header()
header.frame_id = "map"
pose = Pose()  # pose of the charging dock
pose.position.x = 21.0
pose.position.y = 21.5
pose.orientation.w = 1
pos_nav_after_package = PoseStamped()
pos_nav_after_package.header = header
pos_nav_after_package.pose = pose
current_goal = None
releasing_packages = False

def release_package(msg):
    global packages, releasing_packages
    if not msg.data:
        rospy.logerr('Something went wrong trying to release a package...')
        rospy.logdebug('Packages: ' + str(packages))
    elif not packages:
        say('I don\'t have any packages')
        rospy.logdebug('No packages to release')
    else:
        while packages:
            filename = packages.pop(0)
            if filename.endswith('.wav'):
                rospy.logdebug(filename + ' was delivered!')
                # say('I have an audio message to play, here it is...')
                say('!' + filename)  # play sound file
            elif filename.endswith('.package'):
                rospy.logdebug(filename + ' was delivered!')
                say(choice([
                    'Enjoy your package and your day!',
                    'Package delivered, another one in the books.',
                    'Thank you for using the Campus Rover package delivery system!',
                ]))
                if packages:
                    say('I also have an audio message!')
            else:
                rospy.logerr('\'{0}\' file type not recognized'.format(filename))
        releasing_packages = False

def receive_package(msg):
    global packages
    packages.append(msg.data)
    if msg.data.endswith('.wav'):
        say(choice([
            'Thank you for the recording, I will let the receiver know!',
            'Sounds good, I\'ll send it right over!'
        ]))
    rospy.logdebug('Received package' + msg.data)

def button_press_cb(msg):
    global rec_start_pub, rec_stop_pub
    state, button = msg.state, msg.button
    if state == 1:  # button pressed
        rec_start_pub.publish(True)
    else:
        rec_stop_pub.publish(True)

def basket_cb(msg):
    global physical_pub, release_pub, nav_pub, packages, say, pos_nav_after_package, current_goal, move_client, check_cancelled_goal, releasing_packages, talk_rate
    rospy.logwarn(str(msg.values))
    if not msg.values[1]:  # Placed on basket
        if get_state() == States.STOLEN_PACKAGE:
            rospy.loginfo('I am publishing to nav_pub right now!!!')
            current_goal.header.stamp = rospy.Time()
            rospy.loginfo(current_goal)
            nav_pub.publish(current_goal)
            # while get_state() != States.NAVIGATING:
            #     check_cancelled_goal.sleep()
            # say("Thanks, and don\'t touch my stuff again!")
        else:
            physical_pub.publish(True)
            say('Please ree chord a message to go along with your package.')
    else:  # Taken off basket
        if get_state() == States.NAVIGATING:
            move_client.cancel_all_goals()
            rospy.loginfo("Cancelling nav goals")
            while get_state() != States.WAITING:
                check_cancelled_goal.sleep()
            change_state(States.STOLEN_PACKAGE)
        else:
            releasing_packages = True
            release_pub.publish(True)
            while releasing_packages:
                check_cancelled_goal.sleep()
            # Return to a position
            talk_rate.sleep()
            say('Okay, Goodbye!')
            nav_pub.publish(pos_nav_after_package)


def save_goal(msg):
    global current_goal
    current_goal = msg


rospy.init_node('package_handler')
release_sub = rospy.Subscriber(release_package_topic, Bool, release_package)
receive_sub = rospy.Subscriber(receive_package_topic, String, receive_package)

button_sub = rospy.Subscriber('/mobile_base/events/button', ButtonEvent, button_press_cb)
basket_sub = rospy.Subscriber('/mobile_base/events/digital_input', DigitalInputEvent, basket_cb) # todo
release_pub = rospy.Publisher(release_package_topic, Bool, queue_size=1)
rec_start_pub = rospy.Publisher(record_start_topic, Bool, queue_size=1)
rec_stop_pub = rospy.Publisher(record_stop_topic, Bool, queue_size=1)
physical_pub = rospy.Publisher(physical_package_topic, Bool, queue_size=1)

# stop_moving_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
nav_pub = rospy.Publisher('/destination', PoseStamped, queue_size=1)
goal_sub = rospy.Subscriber('/destination', PoseStamped, save_goal)

# say = rospy.ServiceProxy('/say', Talk)

move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
move_client.wait_for_server()

talk_rate = rospy.Rate(0.5)
check_cancelled_goal = rospy.Rate(10)

# What to say when a package is stolen
script = [
    'Hey, bring that back here!',
    'Pito is going to kill me if I don\'t get that package...',
    'Get back here with my package!',
    'I can\'t lose this job.',
    'He couldn\'t have gone far, someone find him!',
    'I can get the package back, I just need funding.',
    'Oh cruel fate, what miserable agony I feel without my beloved package.',
]

while not rospy.is_shutdown():
    if get_state() == States.STOLEN_PACKAGE:
        say(choice(script))
    talk_rate.sleep()
