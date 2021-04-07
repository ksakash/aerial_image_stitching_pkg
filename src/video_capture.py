#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
import copy
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from aerial_image_stitching_pkg.msg import ImagePose
from cv_bridge import CvBridge, CvBridgeError

pose = PoseStamped ()
init = False

def cb (data):
    global pose, init
    pose.pose = data.pose.pose
    init = True

def show_image_pose (frame, pose):
    cv2.imshow ('frame', frame)

    if (cv2.waitKey (25) & 0xFF == ord ('q')):
        return False

    posx = pose.pose.position.x
    posy = pose.pose.position.y
    posz = pose.pose.position.z

    w = pose.pose.orientation.w
    x = pose.pose.orientation.x
    y = pose.pose.orientation.y
    z = pose.pose.orientation.z

    q = Quaternion (w, x, y, z)
    e = q.to_euler (degrees=True)
    roll = float (e[0])
    pitch = float (e[1])
    yaw = float (e[2])

    print ("orientation:", roll, pitch, yaw)
    print ("position:", posx, posy, posz)
    print ("------------------------------")

    return True

rospy.init_node ('video_capture')
pub = rospy.Publisher ('/image_pose', ImagePose, queue_size=50, latch=True)
sub = rospy.Subscriber ('mavros/global_position/local/adjusted', Odometry, cb, queue_size=10)

url = "rtsp://192.168.43.1:8554/fpv_stream"
#url = rospy.get_param ("url", url)
id = rospy.get_param ("id", 0)
uav_name = "UAV" + str (id)

cap = cv2.VideoCapture (url)
bridge = CvBridge ()

if (cap.isOpened () == False):
    print (uav_name, "error in opening video stream")

count = 0
offset = 240
width = 1920

sync = rospy.get_param ("sync", False)
rate = rospy.Rate (20)

mod = int (rospy.get_param ("/mod", 70))

while not init and not rospy.is_shutdown ():
    print (uav_name, "waiting for odometry")
    rate.sleep ()

while (cap.isOpened() and not rospy.is_shutdown ()):
    ret, frame = cap.read ()
    if sync and not show_image_pose (frame, pose):
        break
    if not sync and ret:
        if (count % mod == 0):
            cropped = copy.copy (frame[:,offset:width-offset,:])
            cropped = cv2.rotate (cropped, cv2.ROTATE_90_CLOCKWISE)
            print (uav_name, cropped.shape)
            image = bridge.cv2_to_imgmsg (cropped, encoding="bgr8")
            msg = ImagePose ()
            msg.pose = pose
            msg.image = image
            pub.publish (msg)
    count += 1

cap.release ()
cv2.destroyAllWindows ()

