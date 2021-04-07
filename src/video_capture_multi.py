#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from aerial_image_stitching_pkg.msg import ImagePose
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

pose0 = PoseStamped ()
pose1 = PoseStamped ()
pose2 = PoseStamped ()
pose3 = PoseStamped ()

init0 = False
init1 = False

def cb0 (data):
    global pose0, init0
    pose0.pose = data.pose.pose
    init0 = True

def cb1 (data):
    global pose1, init1
    pose1.pose = data.pose.pose
    init1 = True

def cb2 (data):
    global pose2
    pose2.pose = data.pose.pose

def cb3 (data):
    global pose3
    pose3.pose = data.pose.pose

rospy.init_node ('video_capture_multi')

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

'''
cap2 = cv2.VideoCapture (url2)
cap3 = cv2.VideoCapture (url3)
'''

bridge = CvBridge ()

if (cap0.isOpened () == False):
    print ("error in opening video stream0")
else:
    print ("cap0 opened")

if (cap1.isOpened () == False):
    print ("error in opening video stream1")
else:
    print ("cap1 opened")

'''
if (cap2.isOpened () == False):
    print ("error in opening video stream2")

if (cap3.isOpened () == False):
    print ("error in opening video stream3")
'''

image_pose_de = deque ()
queue_len = 100
count = 0
tic = time.time ()

rate = rospy.Rate (80)

'''
while not (init0 and init1) and not rospy.is_shutdown ():
    print ("waiting for odometry")
    rate.sleep ()
'''

def crop_image (frame):
    frame = frame[:,240:1680,:]
    frame = cv2.rotate (frame, cv2.ROTATE_90_CLOCKWISE)

while (cap0.isOpened() and cap1.isOpened() and not rospy.is_shutdown ()):
    ret0, frame0 = cap0.read ()
    '''debug
    print ("frame0 read", ret0)
    cv2.imshow ('frame0', frame0)
    if (cv2.waitKey (25) & 0xFF == ord ('q')):
        break
    '''
    if ret0:
        if (count % 80 == 0):
            print ("uav0: pushing")
            crop_image (frame0)
            msg = ImagePose ()
            msg.pose = pose0
            msg.image = bridge.cv2_to_imgmsg (frame0, encoding="bgr8")
            image_pose_de.append (msg)
            if len (image_pose_de) > queue_len:
                image_pose_de.popleft ()

    ret1, frame1 = cap1.read ()
    '''debug
    print ("frame1 read", ret1)
    cv2.imshow ('frame', frame1)
    if (cv2.waitKey (25) & 0xFF == ord ('q')):
        break
    '''
    if ret1:
        if (count % 80 == 0):
            print ("uav1: pushing")
            crop_image (frame1)
            msg = ImagePose ()
            msg.pose = pose1
            msg.image = bridge.cv2_to_imgmsg (frame1, encoding="bgr8")
            image_pose_de.append (msg)
            if len (image_pose_de) > queue_len:
                image_pose_de.popleft ()

    '''
    ret, frame = cap2.read ()
    crop_image (frame)
    if ret:
        if (count % 70 == 0):
            msg = ImagePose ()
            msg.pose = pose2
            msg.image = bridge.cv2_to_imgmsg (frame, encoding="bgr8")
            image_pose_de.append (frame)
            if len (image_pose_de) > queue_len:
                image_pose_de.popleft ()
    else:
        break

    ret, frame = cap3.read ()
    crop_image (frame)
    if ret:
        if (count % 60 == 0):
            msg = ImagePose ()
            msg.pose = pose3
            msg.image = bridge.cv2_to_imgmsg (frame, encoding="bgr8")
            image_pose_de.append (frame)
            if len (image_pose_de) > queue_len:
                image_pose_de.popleft ()
    else:
        break
    '''

    if len (image_pose_de) > 0 and (count % 60 == 0):
        print ("debug: publishing")
        msg = image_pose_de.popleft ()
        pub.publish (msg)

    count += 1
    rate.sleep ()

cap0.release ()
cap1.release ()

'''
cap2.release ()
cap3.release ()
'''

cv2.destroyAllWindows ()
