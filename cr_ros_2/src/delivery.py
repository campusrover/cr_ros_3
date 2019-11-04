#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json


# Make this into a ROS node
rospy.init_node('delivery')

orders = []

publisher = rospy.Publisher('delivery_orders', String, queue_size = 10)

def manage(message):

    global orders, publisher

    intent = json.loads(message.data)

    if intent['action'] == 'delivery.order':

        orders.append({

            'item': intent['details']['item'],
            'recipient': intent['details']['recipient']

        })


    elif intent['action'] == 'delivery.announcement':

        orders = []

    else:
	       return

    publication = json.dumps(orders)

    publisher.publish(publication)


rospy.Subscriber('voice_intents', String, manage)


rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
