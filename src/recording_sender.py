#!/usr/bin/env python

# new to cr_ros_2:
# namespacing

import rospy
from scipy.io.wavfile import write
import numpy as np
import sounddevice as sd
from std_msgs.msg import String, Bool
import time
import os

receive_package_topic = 'receive_package'
record_start_topic = 'record_start'
record_stop_topic = 'record_stop'

path_to_packages = 'recordings'

time_started = 0
fs = 44100
rec = np.zeros((fs*15, 2))  # max recording 15 secs

def record_cb(msg):
    global time_started, fs, rec
    sd.rec(samplerate=fs, out=rec)
    time_started = time.time()

def record_stop(msg):
    global time_started, package_pub, fs, rec
    if not os.path.isdir(path_to_packages):
        os.mkdir(path_to_packages)
    fname = '{0}/{1}.wav'.format(path_to_packages, str(time.time()))
    length = ((time.time() - time_started)*fs)//1
    write(fname, fs, rec[:length])
    rospy.loginfo('Created package ' + fname)
    package_pub.publish(fname)

rospy.init_node('recording_sender')
package_pub = rospy.Publisher(receive_package_topic, String, queue_size=1)
record_start_sub = rospy.Subscriber(record_start_topic, Bool, record_cb)
record_stop_sub = rospy.Subscriber(record_stop_topic, Bool, record_stop)

rospy.spin()
