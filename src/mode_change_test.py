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

rospy.init_node ('mode_change_test')
rate = rospy.Rate (20)

state_sub = rospy.Subscriber("mavros/state", State, state_cb)

print ("MODE CHANGE TEST INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the connection with FCU")
    rate.sleep()

rospy.wait_for_service ('mavros/set_mode')
print ("mavros/set_mode service is active...")

set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)
mode_res = set_mode_client (custom_mode="GUIDED")

if mode_res.mode_sent:
    print ("guided mode sent", mode_res.mode_sent)
else:
    print ("failed guided mode!")
    sys.exit (-1)

while (current_state.mode != 'GUIDED' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)

# ------------------------------------------------------

mode_res = set_mode_client (custom_mode="LAND")

if mode_res.mode_sent:
    print ("land mode sent", mode_res.mode_sent)
else:
    print ("failed land mode!")
    sys.exit (-1)

while (current_state.mode != 'LAND' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)

# -------------------------------------------------------

mode_res = set_mode_client (custom_mode="STABILIZE")

if mode_res.mode_sent:
    print ("stabilize mode sent", mode_res.mode_sent)
else:
    print ("failed stabilize mode!")
    sys.exit (-1)

while (current_state.mode != 'STABILIZE' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)

# -------------------------------------------------------

mode_res = set_mode_client (custom_mode="RTL")

if mode_res.mode_sent:
    print ("RTL mode sent", mode_res.mode_sent)
else:
    print ("failed RTL mode!")
    sys.exit (-1)

while (current_state.mode != 'RTL' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)
