#!/usr/bin/env python3

import os
import roslib
import rospy
import cv2
from squaternion import Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge ()
count = 0

offset = [-40, -50, 0]

data_filename = 'imageData.txt'
fhandle = open (data_filename, 'w+')

yaw = 0
pitch = 0
roll = 0

lon = 0
lat = 0
alt = 0

def pose_callback (data):
    global yaw, pitch, roll
    global lon, lat, alt

    w = data.pose.orientation.w
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z

    lon = float (data.pose.position.x) + float (offset[0])
    lat = float (data.pose.position.y) + float (offset[1])
    alt = float (data.pose.position.z) + float (offset[2])

    q = Quaternion (w, x, y, z)
    e = q.to_euler (degrees=True)
    roll = float (e[0])
    pitch = float (e[1])
    yaw = float (e[2])

def callback (data):
    global count
    count += 1
    try:
      cv_image = bridge.imgmsg_to_cv2 (data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.imshow ("Image window", cv_image)
    #cv2.waitKey (3)

    if (count % 40 == 0):
        filename = 'images/' + str (count) + '.jpg'
        cv2.imwrite (filename, cv_image)
        global fhandle
        st = (os.path.basename(filename)) + "," + '%.3f'%lon + "," + \
            '%.3f'%lat + "," + '%.3f'%alt + "," + \
            '%.3f'%yaw + "," + '%.3f'%pitch + "," + \
            '%.3f'%roll + "\n"
        print (st)
        fhandle.write (st)

if __name__ == '__main__':
    rospy.init_node ('image_capture', anonymous=True)
    image_sub = rospy.Subscriber ("/iris/usb_cam/image_raw", Image, callback)
    pose_sub = rospy.Subscriber ("/mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.spin ()
