#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math
import time

current_state = State ()
curr_pose = PoseStamped ()
pose = PoseStamped ()

def state_cb (data):
    global current_state
    current_state = data

def pose_cb (data):
    global curr_pose
    curr_pose = data

rospy.init_node ('offboard_control_ardupilot')
rate = rospy.Rate (20)

state_sub = rospy.Subscriber("mavros/state", State, state_cb)
pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10, latch=True)

print ("INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the state to be connected")
    rate.sleep()

rospy.wait_for_service ('mavros/set_mode')
print ("mavros/set_mode service is active...")

set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)
mode_res = set_mode_client (custom_mode="GUIDED")

if mode_res.mode_sent:
    print ("guided mode sent", mode_res.mode_sent)
else:
    print ("failed guided mode!")

while (current_state.mode != 'GUIDED' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)

rospy.wait_for_service ('mavros/cmd/arming')
print ("mavros/cmd/arming service is active...")

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)

arm_res = arming_client (value=True)
if arm_res.success:
    print ("ARM sent", arm_res.success)
else:
    print ("Failed Arming!")

takeoff_client = rospy.ServiceProxy ("mavros/cmd/takeoff", CommandTOL)
takeoff_res = takeoff_client (altitude=1.5)

if takeoff_res.success:
    print ("takeoff sent", takeoff_res.success)
else:
    print ("failed takeoff!")

time.sleep (5)

pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 10

print ("publishing target pos on the local pose topic ..")

local_pos_pub.publish (pose)
t = 10
time.sleep (t)

print ("completed the test!!")

land_client = rospy.ServiceProxy ("mavros/cmd/land", CommandTOL)
land_res = land_client ()

if land_res.success:
    print ("land sent", land_res.success)
else:
    print ("landing failed!")
    rospy.shutdown ()

while not rospy.is_shutdown ():
    rate.sleep ()
