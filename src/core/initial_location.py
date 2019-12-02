#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


rospy.init_node('initial_location')

def amcl_cb(request):
    initial_pose_pub.publish(request)

initial_pose_pub = rospy.Publisher('initial_pose', PoseWithCovarianceStamped, queue_size=1)
amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_cb)


rospy.spin()
