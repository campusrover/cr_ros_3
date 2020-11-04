#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from flask import *
import json
import random


# make this into a ROS node.
rospy.init_node('voice_webhook')

# publish intents recieved from webhook
intent_publisher = rospy.Publisher('voice_intents', String, queue_size = 10)

# initialize the flask app and default route
app = Flask(__name__)
@app.route('/')

# Parse slots JSON to get detail value
def get_slot_value(slots, detail):
    if detail in slots and 'resolutions' in slots.get(detail):
    	return slots.get(detail).get('resolutions').get('resolutionsPerAuthority') [0].get('values') [0].get('value').get('name')
    return None


# Alexa Webhook
@app.route('/alexa_webhook', methods=['GET', 'POST'])
def alexa_webhook():

    # parse json request
    query_result = request.get_json(force = True).get('request')
    action = query_result.get('intent').get('name').replace('_', '.')
    slots = query_result.get('intent').get('slots')
    details = {}

    # Default Reply
    reply = random.choice(['Ok, ', 'Sure, ', 'Got it, ', 'All right, ', 'Yes'])

    # Switch between publishing translation and rotation command
    if action == 'navigation.translation':
        details['direction'] = get_slot_value(slots, 'translation_direction')
        details['distance'] = {
            'amount': slots.get('quantity').get('value', None),
            'unit': get_slot_value(slots, 'distance_unit')
        }
    elif action == 'navigation.rotation':
        details['direction'] = get_slot_value(slots, 'rotation_direction')
        details['angle'] = {
            'amount': slots.get('quantity').get('value', None),
            'unit': get_slot_value(slots, 'angle_unit')
        }
	
    # publish intent to voice intent topic
    intent = json.dumps({'action': action, 'details': details, 'source': 'Alexa'})
    intent_publisher.publish(intent)

    # return valid web response to skill console
    return jsonify({
	'version': '1.0',
	'response': {'outputSpeech': {'type': 'PlainText', 'text': reply}},
	'interpretation': intent
    })


# run the app
if __name__ == '__main__':
    app.run()
    rospy.spin()
