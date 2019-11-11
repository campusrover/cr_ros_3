#!/usr/bin/env python

import rospy
from all_states import *
from pickle import dump, load


ez_states = {'waiting': States.WAITING, 'navigating': States.NAVIGATING, 'flying': States.FLYING, 'lost': States.LOST, 'teleop': States.TELEOP, 'search': States.SEARCH, 'stolen package': States.STOLEN_PACKAGE}


def current_state_is(state):
    """
    Simple method to check if the current state is the one specified.
    Parameters: 
    state: a string that matches one of the keys in ez_states above
    Returns:
    Boolean
    """
    return get_state() == States[state.upper()]        


def conditional_change_state(current_state, new_state, cond):
    """
    If the robot is in a given state and a specific condition is met, then change to a different state.
    Parameters:
    current_state: string that matches a key in ez_states, representing assumed current state
    new_state: string that matches a key in ez_states, representing desired state to transition to
    cond: a boolean value, which if true will trigger transition from current state to new state. cond can also be a function that returns true/false, this fucntion will evaluate cond (as long as cond has no args)
    Returns: 
    Boolean: true is state change was successful, false if failure
    """
    if callable(cond):
        cond = cond()
    if current_state_is(state) and cond:
        change_state(new_state)
        return True 
    else:
        return False


def demand_state_change(new_state):
    """
    Tries to force robot into desired state. 
    Parameters: 
    new_state: a string that matches a key in ez_states which represents the desired state to change to
    Returns:
    Boolean: true id state change was successful, false if failure. 
    """
    if not current_state_is(state):
        change_state(new_state)
        return True 
    else:
        return False
