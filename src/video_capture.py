#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from laser_pkg.msg import ImagePose
from cv_bridge import CvBridge, CvBridgeError

pose = PoseStamped ()

def cb (data):
    global pose
    pose.pose = data.pose.pose

rospy.init_node ('video_capture')
pub = rospy.Publisher ('/image_pose', ImagePose, queue_size=50, latch=True)
sub = rospy.Subscriber ('/mavros/global_position/local/adjusted', Odometry, cb, queue_size=10)

url = "rtsp://192.168.43.1:8554/fpv_stream"
cap = cv2.VideoCapture (url)
bridge = CvBridge ()

if (cap.isOpened () == False):
    print ("error in opening video stream")

while (cap.isOpened()):
    ret, frame = cap.read ()
    if (ret == True):
        cv2.imshow ('Frame', frame)
        image = bridge.cv2_to_imgmsg (frame, encoding="bgr8")
        msg = ImagePose ()
        msg.pose = pose
        msg.image = image
        pub.publish (msg)
        time.sleep (4)
        if (cv2.waitKey (25) & 0xFF == ord ('q')):
            break
    else:
        break

cap.release ()
cv2.destroyAllWindows ()

