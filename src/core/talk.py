#!/usr/bin/env python

# Description: a service which can turn given text into speech

import rospy
from std_msgs.msg import String
from cr_ros_3.srv import Talk, TalkResponse
import pyttsx   # python text to speech (offline)
#import sounddevice as sd
#from scipy.io.wavfile import read
import time

def things_to_say_cb(req):
    global engine, single_inst, rate
    while single_inst:
        rate.sleep()
    single_inst = True  # lock
    to_say = req.to_say
    """ # disabled this section because sounddevice cannot be installed on raspberry pi
    if to_say[0] == '!':  # '!' at beggining of string signifies a sound file
        rospy.loginfo('Playing sound file ' + to_say[1:])
        rate_d, data = read(to_say[1:])
        sd.play(data*5.5, rate_d)
        time.sleep(data.shape[0] // rate_d)
    """
    if True:
        rospy.loginfo('Saying \"{}\"'.format(to_say))
        engine.setProperty('volume', 1.0) # max volume
        engine.setProperty('voice', 'en-us')
        engine.setProperty('rate', 120)
        engine.say(to_say)   # says the message
        engine.runAndWait()
        reload(pyttsx)
        engine = pyttsx.init()
    single_inst = False  # release the lock
    return TalkResponse()

rospy.init_node('talk')

engine = pyttsx.init()
single_inst = False  # sets lock to free initially
rate = rospy.Rate(2)

things_to_say_serv = rospy.Service('say', Talk, things_to_say_cb)

print "Ready to talk"
rospy.spin()
