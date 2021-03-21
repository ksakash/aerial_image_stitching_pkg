#!/usr/bin/env python3

import roslib
import rospy
import cv2
import time
import numpy as np

from laser_pkg.msg import ImagePose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node ('image_pose_pub', anonymous=True)

    pub = rospy.Publisher ("/image_pose", ImagePose, queue_size=5, latch=True)

    dirname = '/home/ksakash/misc/aerial_image_stitching/airsim_images'
    filename = '/home/ksakash/misc/aerial_image_stitching/airsimImage.txt'
    bridge = CvBridge ()

    data_matrix = np.genfromtxt (filename, delimiter=",", usecols=range(1, 7), dtype=float)
    image_name_matrix = np.genfromtxt (filename, delimiter=",", usecols=[0], dtype=str)
    count = 0

    while not rospy.is_shutdown () and count < len (data_matrix):
        pose = PoseStamped ()
        pose.pose.position.x = data_matrix[count][0]
        pose.pose.position.y = data_matrix[count][1]
        pose.pose.position.z = data_matrix[count][2]

        image_name = image_name_matrix[count]
        image_path = dirname + '/' + image_name
        img = cv2.imread (image_path)
        image = bridge.cv2_to_imgmsg (img, encoding="bgr8")

        msg = ImagePose ()
        msg.pose = pose
        msg.image = image

        pub.publish (msg)

        count += 1
        time.sleep (4.0)
