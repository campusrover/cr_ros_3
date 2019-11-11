#!/usr/bin/env python

import rospy
from all_states import *
from state_tools import *
from time import sleep

sleep(0.2)
"""
NOTE: state.py needs a little bit of time to get started before any state queries or change requests can be made. as such, no node that immediately checks the current state should be launched alongside state.py without some sort of delay
delay should be at least 0.5 seconds. 
alternate solution: any node which constantly checks current state can be launched offboard - because state.py is an onboard node, the time between launches is enough to give state the time it need to initiallize itself. 
""" 
print('TEST')
print(States['WAITING'])
print('END TEST')
print('am I lost? <{}>'.format(current_state_is('lost')))
demand_state_change('waiting')
print('am I waiting? <{}>'.format(current_state_is('waiting')))
print('am I navigating? <{}>'.format(current_state_is('navigating')))
