#!/usr/bin/env python

import face_recognition
import rospy
import cv2
import os

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from cr_ros_3.srv import Talk

IM_DIR = "/files/faces"
PUBLISH_RATE = 5 # seconds
bridge = CvBridge()

path = os.path.dirname(os.path.realpath(__file__))[:-4] + IM_DIR
all_people = [("Alexander", "/alex.jpg"), ("Pito", "/pito.jpg"), ("Cam", "/cam.png"),\
            ("Brad", "/brad.png"), ("Alec", "/Alec.png"), ("Ben", "/ben.png"), \
            ("Jacky", "/jacky.png"), ("Aaron", "/aaron.png"), ("Ari", "/ari.png"), \
            ("Huaigu", "/huaigu.png"), ]
showcase_people = [("Jacky", "/jacky.png")]

people = showcase_people  # this is why everyone is jacky

known_face_encodings = []
known_face_names = []
last_seen = {}

def recognize_cb(msg):
    im = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_frame = im[:, :, ::-1]

    # Find all the faces and face encodings in the current frame of video
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    face_names = []
    for face_encoding in face_encodings:
        # See if the face is a match for the known face(s)
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

        # If a match was found in known_face_encodings, just use the first one.
        if True in matches:
            first_match_index = matches.index(True)
            name = known_face_names[first_match_index]
            last_publish = last_seen.get(name)
            if last_publish is None or rospy.get_time() - last_publish > PUBLISH_RATE: # published long enough ago
                try:
                    talk_srv("Hello {}".format(name))
                except rospy.ServiceException:
                    rospy.logwarn("Talking service unavailable")
                last_seen[name] = rospy.get_time()

""" this topic is what aruco_detect publishes - see mutant_offboard.launch"""
im_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed_throttle", CompressedImage, recognize_cb)
talk_srv = rospy.ServiceProxy("say", Talk)
rospy.init_node("greeter")

# Load all faces
for person in people:
    if os.path.exists(path + person[1]):
        rospy.logdebug("Loading " + person[0])

        person_im = face_recognition.load_image_file(path+person[1])
        person_encoding = face_recognition.face_encodings(person_im)[0]

        known_face_encodings.append(person_encoding)
        known_face_names.append(person[0])

rospy.spin()
