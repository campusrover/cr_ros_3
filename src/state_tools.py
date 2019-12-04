#!/usr/bin/env python

import rospy
from all_states import *
import time
from cr_ros_3.msg import ThingsToSay

def current_state_is(state):
    """
    Simple method to check if the current state is the one specified.
    Parameters: 
    state: a string that could be a state if uppercased
    Returns:
    Boolean
    """
    return get_state() == States[state.upper()]   


def conditional_change_state(current_state, new_state, cond=True):
    """
    If the robot is in a given state and a specific condition is met, then change to a different state.
    Parameters:
    current_state: string that matches a key in ez_states, representing assumed current state
    new_state: a string that could be a state if uppercased, representing desired state to transition to
    cond: a boolean value, which if true will trigger transition from current state to new state. cond can also be a function that returns true/false, this fucntion will evaluate cond (as long as cond has no args)
    Returns: 
    Boolean: true is state change was successful, false if failure
    """
    if callable(cond):
        cond = cond()
    if current_state_is(current_state) and cond:
        return change_state(States[new_state.upper()]) 
    else:
        return False


def demand_state_change(new_state):
    """
    Tries to force robot into desired state. 
    Parameters: 
    new_state: a string that could be a state if uppercased, representing desired state to transition to
    cond: a boolean value, which if true will trigger transition from current state to new state. cond can also be a function that returns true/false, this fucntion will evaluate cond (as long as cond has no args)
    new_state: a string that matches a key in ez_states which represents the desired state to change to
    Returns:
    Boolean: true if state change was successful, false if failure. 
    """
    if not current_state_is(new_state):
        return change_state(States[new_state.upper()]) 
    else:
        return False

def conditional_demand_state_change(new_state, cond):
    """
    Will only demand a state change if a condition is met, regardless of the current state
    Parameters:
    Returns:
    Boolean: true if state change was successful, false if failure. 
    """
    if callable(cond):
        cond = cond()
    if cond:
        return demand_state_change(new_state)
    else:
        return False


def talker(string_to_say, things_to_say_publisher):
    """
    Sends a string to the talk queue for the talk service to say out loud
    """
    things_to_say_publisher.publish(ThingsToSay(
        say_at=time.time(),
        to_say=string_to_say
    ))

def debug_check_state():
    return get_state()
