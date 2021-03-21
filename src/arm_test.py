#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math
import time

import sys

current_state = State ()
def state_cb (data):
    global current_state
    current_state = data

rospy.init_node ('arm_test')
rate = rospy.Rate (20)

state_sub = rospy.Subscriber("mavros/state", State, state_cb)

print ("ARM TEST INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the connection with FCU")
    rate.sleep()

rospy.wait_for_service ('mavros/cmd/arming')
print ("mavros/cmd/arming service is active...")

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)

arm_res = arming_client (value=True)
if arm_res.success:
    print ("ARM sent", arm_res.success)
else:
    print ("Failed Arming!")
    sys.exit (-1)

print ("Arming for 5 seconds ...")
time.sleep (5)

arm_res = arming_client (value=False)
if arm_res.success:
    print ("DISARM sent", arm_res.success)
else:
    print ("Failed DisArming!")
    sys.exit (-1)

print ("test completed successfully !!")
