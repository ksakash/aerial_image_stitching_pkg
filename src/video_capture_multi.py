#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from laser_pkg.msg import ImagePose
from collections import deque

pose0 = PoseStamped ()
pose1 = PoseStamped ()
pose2 = PoseStamped ()
pose3 = PoseStamped ()

def cb0 (data):
    global pose0
    pose0.pose = data.pose.pose

def cb1 (data):
    global pose1
    pose1.pose = data.pose.pose

def cb2 (data):
    global pose2
    pose2.pose = data.pose.pose

def cb3 (data):
    global pose3
    pose3.pose = data.pose.pose

rospy.init_node ('video_capture')
pub = rospy.Publisher ('/image_pose', ImagePose, queue_size=50, latch=True)
sub0 = rospy.Subscriber ('/uav0/mavros/global_position/local/adjusted', \
                         Odometry, cb0, queue_size=10)
sub1 = rospy.Subscriber ('/uav1/mavros/global_position/local/adjusted', \
                         Odometry, cb1, queue_size=10)
sub2 = rospy.Subscriber ('/uav2/mavros/global_position/local/adjusted', \
                         Odometry, cb2, queue_size=10)
sub3 = rospy.Subscriber ('/uav3/mavros/global_position/local/adjusted', \
                         Odometry, cb3, queue_size=10)

url0 = "rtsp://192.168.43.1:8554/fpv_stream"
url1 = "rtsp://192.168.42.129:8554/fpv_stream"
url2 = ""
url3 = ""

cap0 = cv2.VideoCapture (url0)
cap1 = cv2.VideoCapture (url1)
cap2 = cv2.VideoCapture (url2)
cap3 = cv2.VideoCapture (url3)

bridge = CvBridge ()

if (cap0.isOpened () == False):
    print ("error in opening video stream0")

if (cap1.isOpened () == False):
    print ("error in opening video stream1")

if (cap2.isOpened () == False):
    print ("error in opening video stream2")

if (cap3.isOpened () == False):
    print ("error in opening video stream3")

image_de = deque ()
queue_len = 50

while (cap0.isOpened() or cap1.isOpened() or cap2.isOpened() or \
       cap3.isOpened()):
    ret, frame = cap0.read ()
    if ret:
        image_de.append (frame)
        if len (image_de) > queue_len:
            image_de.popleft ()
    ret, frame = cap1.read ()
    if ret:
        image_de.append (frame)
        if len (image_de) > queue_len:
            image_de.popleft ()
    ret, frame = cap2.read ()
    if ret:
        image_de.append (frame)
        if len (image_de) > queue_len:
            image_de.popleft ()
    ret, frame = cap3.read ()
    if ret:
        image_de.append (frame)
        if len (image_de) > queue_len:
            image_de.popleft ()

    if len (image_de) == 0:
        break

    frame = image_de.popleft ()
    image = bridge.cv2_to_imgmsg (frame, encoding="bgr8")
    msg = ImagePose ()
    msg.pose = pose
    msg.image = image
    pub.publish (msg)
    time.sleep (3)

    if (cv2.waitKey (25) & 0xFF == ord ('q')):
        break

cap.release ()
cv2.destroyAllWindows ()

