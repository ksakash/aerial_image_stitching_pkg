import rospy
from laser_pkg.msg import ImagePose
import numpy as np

import cv2

import time
from squaternion import Quaternion
from cv_bridge import CvBridge, CvBridgeError

f = open ('videoStreamImages.txt', 'w+')
count = 0
bridge = CvBridge ()

def cb (data):
    global f, count, bridge
    try:
        curr_img = bridge.imgmsg_to_cv2 (data.image, "bgr8")
    except CvBridgeError as e:
        print (e)

    w = data.pose.pose.orientation.w
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z

    lon = data.pose.pose.position.x
    lat = data.pose.pose.position.y
    alt = data.pose.pose.position.z

    q = Quaternion (w, x, y, z)
    e = q.to_euler (degrees=True)
    roll = float (e[0])
    pitch = float (e[1])
    yaw = float (e[2])

    st = (os.path.basename(filename)) + "," + '%.3f'%lon + "," + \
        '%.3f'%lat + "," + '%.3f'%alt + "," + \
        '%.3f'%yaw + "," + '%.3f'%pitch + "," + \
        '%.3f'%roll + "\n"
    print (st)
    f.write (st)
    cv2.imwrite ('video_stream/' + str (count) + '.jpg', curr_img)
    count += 1

rospy.init_node ('cap_video_stream')
sub = rospy.Subscriber ('/image_pose', ImagePose, cb)

rospy.spin ()
