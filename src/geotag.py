#!/usr/bin/env python3

from collections import deque
import rospy
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

queue_size = 6
odometry_de = deque ()

rospy.init_node ('geotagging_images')
pub = rospy.Publisher ('mavros/global_position/local/adjusted', Odometry, queue_size=10, latch=True)

def cb (data):
    global odometry_de, pub
    odometry_de.append (data)
    if (len (odometry_de) > queue_size):
        msg = odometry_de.popleft ()
        pub.publish (msg)

sub = rospy.Subscriber ('mavros/global_position/local', Odometry, cb, queue_size=10)
rospy.spin ()
