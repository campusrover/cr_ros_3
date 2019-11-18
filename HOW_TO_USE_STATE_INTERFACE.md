
# The State Manager and You: How to Add Nodes + Features to Campus Rover

Hi there. So you've made some nifty piece of code that you want to incorperate to the campus rover package. It is important when you are doing this to make sure that your desired behavior cooperates with the state manager. This guide will help you to do so. 

## How the State Manager Works
there are 3 key files to the state manager - `all_states.py`, `state.py`, and `state_tools.py`. 

* `state.py` is a node which runs two services - **state_change** and **state_query**. Their fuctions are as expected - **state_change** will change the current state of the robot, if the transition to the requested state from the current state is a legal transition (if it is an illegal transition, the robot will go to the `ILLEGAL_STATE_CHANGE` state - don't force illegal state changes). **state_query** will return the current state of the robot. 
* `all_states.py` contains the enumeration of all states as well as the dictionary of legal state changes. Therefore, if you wish to add a new state to the state manager, you must add it in all applicable places in `all_states.py`. this file also has two functions that communicate with `state.py`'s services. 
* `state_tools.py` provides some functions to make interacting with the state manager easier. `current_state_is(state)` returns True/False based on if the current state of the robot matches the query. `demand_state_change(state)` requests a state change to the passed state, as long as the robot is not already in that state. `conditional_state_change(old_state new_state cond)` will change to the requested state only if the conditional passed is true and the current state matches the old_state. These tools should be enough to interface with the state manager. To use these tools: `from state_tools import *` **NB** tThere are some other tools not listed - read the file to see all available functions.  

## How to Incorperate your Feature

Let's assume you've built some ros node (or collection of nodes) that publishes a useful topic. Now, you're going to make an *intermediary node* that uses that useful topic to interact with the state manager. 

Here's an __example__: you've made a node that uses gesture recognition to recognize a finger pointing and waving in a circle. Your node publishes a topic `/finger_spin`. You want this gesture to translate to the behavior of the campus rover turning around 180 degrees. Your intermediatry node will subscribe to `/finger_spin`. The "turning around" behavior will be handled by orderring the campus rover to navigate to a pose slightly behind where it currently is and facing in the opposite direction. Your intermediary node might simply be a callback - where `if conditional_state_change('waiting', 'navigating', msg.data): turn_pub.publish(turn_pose)` (assuming turn_pose has already been calculated, and you only want this behavior to happen from the waiting state). As you see interacting with the state manager 