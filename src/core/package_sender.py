#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
import time
import os

receive_package_topic = '/receive_package'
physical_package_topic = '/physical_package'

path_to_packages = 'recordings'

def package_cb(msg):
    if not msg.data:
        rospy.logerr('There was an error adding the physical package')
    else:
        global package_pub
        if not os.path.isdir(path_to_packages):
            os.mkdir(path_to_packages)
        fname = '{0}/{1}.package'.format(path_to_packages, str(time.time()))
        open(fname, 'a')
        rospy.loginfo('Created package ' + fname)
        package_pub.publish(fname)

rospy.init_node('package_sender')
package_pub = rospy.Publisher(receive_package_topic, String, queue_size=1)
physical_sub = rospy.Subscriber(physical_package_topic, Bool, package_cb)

rospy.loginfo('Ready for physical packages!')
rospy.spin()
