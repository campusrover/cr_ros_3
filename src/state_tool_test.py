#!/usr/bin/env python

import rospy
from all_states import *
from state_tools import *
from time import sleep
from random import randint

def fifty_fifty():
    roll = randint(0, 100)
    return roll >= 50
sleep(0.2)
"""
NOTE: state.py needs a little bit of time to get started before any state queries or change requests can be made. as such, no node that immediately checks the current state should be launched alongside state.py without some sort of delay
delay should be at least 0.5 seconds. 
alternate solution: any node which constantly checks current state can be launched offboard - because state.py is an onboard node, the time between launches is enough to give state the time it need to initiallize itself. 
""" 
# this little interactive tool demos how to use the tools that have been added to state_tools.py
while not rospy.is_shutdown():
    desired_state = raw_input('pick a state to change to (current={})'.format(get_state())) # NOTE in python 2.7.x raw_input returns a string, input does not
    roll = randint(0, 100)
    if roll > 25:                                                                   # using rng in this script to simulate sensor readings on the robot
        success = demand_state_change(desired_state)                                # demand state change to the user's input state
        if success:
            print('done')
            if current_state_is('flying'):                                          # checks if the user has changed the state to flying
                print('I\'m flying?!? i\'m not sure i want to...')
                success2 = conditional_change_state('flying', 'lost', fifty_fifty)  # uses the fifty_fifty conditional to decide whether to go to lost
                if success2:
                    print('phew, grounded')
                else:
                    print('weee! I love flying, actually')
        else:
            print('oops')
    else:
        print('sorry, rng won\'t let you do that')