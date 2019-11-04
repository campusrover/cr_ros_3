#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def odom_cb(msg):
    global pose
    pose = msg.pose.pose.position


pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
sub = rospy.Subscriber("odom", Odometry, odom_cb)
rospy.init_node("speed_test")


pose = None
timer = 1
tw = Twist()
tw.linear.x = 0.1
print("{}cmd_vel")
rate = rospy.Rate(3)
while pose is None:
    continue
print(pose.x, pose.y)
start = rospy.Time.now().secs
while timer < 10 and not rospy.is_shutdown():
    pub.publish(tw)
    if(rospy.Time.now().secs  > start + timer):
        print(timer)
        timer += 1
    rate.sleep()


tw.linear.x = 0.0
print(timer, " time to stop")
print(pose.x, pose.y)
pub.publish(tw)

"""
####### Results of Speed test #######
1. at .1 m/s for 10 seconds one would expect the robot to travel about 1 meter.
2. mutant traveled about 1.33 meters (not in a straight line either... so a slightly greater distance if you calculated the sterring error)
3. tb3 wheels are set to 6.6 cm by default (diameter)
4. mutant wheels are 10 cm in diamter (150%)

run 2: mutant traveled ~ 148 cm in 10 seconds - amount of veer no measured
run 3: mutant traveled ~ 148 cm again, about 2.5 inches of veer to the left (6.35 cm)
run 4: 152 cm traveled, 2 inch veer left. odom pose from (4.2, -0.2) -> (3.78, -0.9)
run 5:
"""
