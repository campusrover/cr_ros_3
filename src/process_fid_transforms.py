#!/usr/bin/env python


# changes in cr_ros_2:
# --> added namespace to all topics
# removed kobuki docking garbage

import rospy
import tf
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import Transform, TransformStamped
from pose_converter import PoseConv
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseWithCovariance
import numpy as np
import math
from all_states import *

FIDUCIAL_NAMES = {}

# add fiducials to dict in the following format - "100":"fid_100"
for num in range(100,200):
    FIDUCIAL_NAMES[str(num)] = "fid_" + str(num)

def fid_cb(msg):
    """
    callback which takes fiducial transfroms and publishes pose of
    robot base relative to map
    @param msg: A fiducial_msgs FidcucialTransformArray object
    """
    global tf_broadcaster
    global tf_listener
    global FIDUCIAL_NAMES
    global pose_pub

    # if driving, don't interupt
    if get_state() not in [States.LOST, States.TELEOP]:  #removed docked state
        return
    # if fiducials found, take the first
    if len(msg.transforms) == 0:
        return
    transform = msg.transforms[0]

    # swap y and z axes to fit map frame of reference
    pos = transform.transform.translation
    rot = transform.transform.rotation
    pos.x, pos.y, pos.z = pos.x, pos.z, pos.y
    rot.x, rot.y, rot.z = rot.x, rot.z, rot.y
    transform.transform.translation = pos
    transform.transform.rotation = rot

    # invert the transform
    homo_mat = PoseConv().to_homo_mat(transform.transform)
    inverted_tf =  PoseConv().to_tf_msg(np.linalg.inv(homo_mat))

    # send a transform from camera to fiducial
    m = TransformStamped()
    m.transform = inverted_tf
    m.header.frame_id = FIDUCIAL_NAMES.get(str(transform.fiducial_id))
    m.header.stamp = rospy.Time.now()
    m.child_frame_id = "fiducial_camera"
    tf_broadcaser.sendTransform(m)

    # calculate transform from map to base
    try:
        latest_time = tf_listener.getLatestCommonTime("/map","/fiducial_base")
        base_to_map = tf_listener.lookupTransform("/map","/fiducial_base",latest_time)
    except tf2_ros.TransformException:
        rospy.logwarn("failed to transform, is fiducial {} mapped?".format(transform.fiducial_id))
        return

    # convert transform to PoseWithCovarianceStamped
    robot_pose = PoseConv().to_pose_msg(base_to_map)
    pose_w_cov_stamped = PoseWithCovarianceStamped()
    pose_w_cov_stamped.pose.pose = robot_pose
    pose_w_cov_stamped.header.stamp = rospy.Time.now()
    pose_w_cov_stamped.header.frame_id = "map"
    rospy.logdebug("Sending fiducial pose:\n{}".format(robot_pose))

    # publish to /initialpose
    pose_pub.publish(pose_w_cov_stamped)

    # update state
    try:
        prev_state = get_state()
        if prev_state == States.LOST:
            change_state(States.WAITING)
            rospy.loginfo("Fiducial {} seen, no longer lost".format(transform.fiducial_id))
    except rospy.ServiceException:
        rospy.logerr("Could not access state service")


rospy.init_node("process_fid_tfs",) # log_level=rospy.DEBUG)
rospy.Subscriber('fiducial_transforms', FiducialTransformArray, fid_cb)    # BIG NOTE: not namespaced until we can namespace aruco detect output
pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
tf_broadcaser = tf2_ros.TransformBroadcaster()
tf_listener = tf.TransformListener()

rospy.spin()
