#!/usr/bin/env python

# Description: monitors cpu usage, publishes as a %, warns if usage > 90%

# changes for cr_ros_2: changed the name of the topic from laptop_cpu_usage to
# raspberry_pi_cpu_usage

import rospy
import psutil  # a python package which monitors system resource utilization
from std_msgs.msg import Float32


rospy.init_node('cpu_checker')
cpu_pub = rospy.Publisher('sbc_cpu_usage', Float32, queue_size=1)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    percent = psutil.cpu_percent()
    # warns if cpu usage is > 90%, reports usage % otherwise
    if percent > 90:
        rospy.logwarn("Turtlebot CPU usage is very high at {0:.1f}%".format(percent))
    else:
        rospy.logdebug("Turtlebot CPU usage is {0:.1f}%".format(percent))
    cpu_pub.publish(percent)
    rate.sleep()
