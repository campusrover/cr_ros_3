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
publisher = rospy.Publisher('voice_intents', String, queue_size = 10)

package_publisher = rospy.Publisher('/has_package', Bool, queue_size = 10)


# Get current speaker from subscription

speaker = None

def set_speaker(message):
    global speaker
    speaker = message.data[1:].split(',')[0]

rospy.Subscriber('mutant/face_detection', String, set_speaker)


# Get current list of orders from subscription

orders = []

def set_orders(message):
    global orders

    try:
        orders = json.loads(message.data)
    except:
	orders = []

rospy.Subscriber('delivery_orders', String, set_orders)


# initialize the flask app and default route
app = Flask(__name__)
@app.route('/')


# Parse slots JSON to get detail value
def get_slot_value(slots, detail):
    if detail in slots and 'resolutions' in slots.get(detail):
    	return slots.get(detail).get('resolutions').get('resolutionsPerAuthority') [0].get('values') [0].get('value').get('name')

    return None;


def item_and_recipient(order):
    item = order['item']
    recipient = order['recipient']
    if recipient == None:
        recipient = "someone"
    return item + " for " + recipient


# Alexa Webhook
@app.route('/alexa_webhook', methods=['GET', 'POST'])
def alexa_webhook():

    global speaker, orders, has_package

    query_result = request.get_json(force = True).get('request')

    action = query_result.get('intent').get('name').replace('_', '.')

    slots = query_result.get('intent').get('slots')

    details = {}


    # Default Reply

    reply = random.choice(['Ok, ', 'Sure, ', 'Got it, ', 'All right, ', 'Yes, ', 'That I shall do, ', 'Here I go, '])

    if speaker == None:
        reply += random.choice(['friend', 'pal', 'buddy', 'mate', 'sir'])
    else:
        reply += speaker


    # Navigation

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

    elif action == 'navigation.halt':
        pass    # nothing needs to be done

    elif action == 'navigation.arrival':
    	details['destination'] = get_slot_value(slots, 'destination')

	if details['destination'] == None and len(orders) > 0:
	    package_publisher.publish(True)


    # Delivery

    elif action == 'delivery.intent':
        reply = 'I would be happy to get something for you, '

	if speaker == None:
            reply += random.choice(['friend', 'pal', 'buddy', 'mate', 'sir'])
    	else:
            reply += speaker

    elif action == 'delivery.order':
        details['item'] = get_slot_value(slots, 'item')
        details['recipient'] = get_slot_value(slots, 'person')

	if (details['recipient'] == None):
	    details['recipient'] = speaker

	reply = "One " + details['item']

	if details['recipient'] != None:
	    reply += " for " + details['recipient']

	reply += " coming right up!"

    elif action == 'delivery.fulfillment':
        if len(orders) == 0:
	    reply = "Nope, I don't have any orders to deliver"
	else:
	    reply = "Yeah, I must deliver "
	    if len(orders) == 1:
		reply += item_and_recipient(orders[0])
	    elif len(orders) == 2:
		reply += item_and_recipient(orders[0]) + " and " + item_and_recipient(orders[1])
	    else:
		for counter in range(len(orders) - 1):
		    reply += item_and_recipient(orders[counter]) + ", "
		reply += "and " + item_and_recipient(orders[-1])

    elif action == 'delivery.announcement':
        if len(orders) == 0:
	    reply = "Nope, I don't have any orders to deliver"
	else:
	    reply = "Hear ye, hear ye. I come bearing "
	    if len(orders) == 1:
		reply += item_and_recipient(orders[0])
	    elif len(orders) == 2:
		reply += item_and_recipient(orders[0]) + " and " + item_and_recipient(orders[1])
	    else:
		for counter in range(len(orders) - 1):
		    reply += item_and_recipient(orders[counter]) + ", "
		reply += "and " + item_and_recipient(orders[-1])
	    
	    package_publisher.publish(False)


    # Miscellaneous

    elif action == 'miscellaneous.appreciation':
	reply = "No problem, bon appetit!"

    elif action == 'miscellaneous.humor':
	jokes = [
	    "Why was the android itchy? Robo-ticks!",
	    "What do you call an android crew team? Row-bots!",
	    "What did the droid do at lunch time? It had a byte to eat!",
	    "Why don't we hear R2-D2 say anything? He says so many foul words they had to bleep everything!",
	    "Why did the robot go to the shopping mall? It had hardware and software, but it needed underware!",
	    "Why was the robot feeling bad? It had a virus!",
	    "What do you say to a dead robot? Rust in peace!",
	    "Why was the robot so tired when it finally got home? It had a hard drive!"
	]

        reply = random.choice(jokes)

    elif action == 'miscellaneous.guide':
        #reply = "Hi, this is campusrover, awaiting your command! I use natural language procesing to understand you, so you don't have to worry about being exact with your commands, I'll get the point. To have me read this guide to you, just say, echo tell campusrover to read me its guide, or some variation of that. If you would like to have me stop moving at any point, simply say, echo tell campusrover to halt, or, echo tell campusrover to stop. If you would like me to move forward or backword, simply say, echo tell campusrover to move up, or echo tell campusrover to back up. You may similarly ask me to turn by saying something like, echo tell campusrover to turn left, or, echo tell campusrover to spin right. If you would like to specify by how much I should move or turn, just say, echo go forward five feet, or echo rotate right 90 degrees. I accept distances in meters or feet and angles in degrees or radians. If you would like me to go to the cafe, just say, echo tell campusrover to go to the cafe. Please note that I recognize a variety of terms for the cafe such as canteen and food stall. You may also ask me to go back to the last person I recieved an order from by saying, echo tell campusrover to go back. If you would like to order somthing, please say, echo tell campusrover to get me some coffee, or, echo tell campusrover I would like some candy. I hope this was helpful. Have a nice day!"
        reply = "This is campusrover. It carries things around the G-zang basement."

    elif action == 'edit.reset':
	orders = []
	package_publisher.publish(False)

	reply = " I have successfully reset"
	

    intent = json.dumps({'action': action, 'details': details, 'source': 'Alexa'})
    publisher.publish(intent)

    speaker = None

    return jsonify({
	'version': '1.0',
	'response': {'outputSpeech': {'type': 'PlainText', 'text': reply}},
	'interpretation': intent
    })


# run the app
if __name__ == '__main__':
    app.run()
    package_publisher.publish(False);


rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
