#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped

current_state = State ()
curr_pose = PoseStamped ()

def state_cb (data):
    global current_state
    current_state = data

def pose_cb (data):
    global curr_pose
    curr_pose = data

rospy.init_node ('offboard_control')

state_sub = rospy.Subscriber("mavros/state", State, state_cb)
pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10, latch=True)

rospy.wait_for_service ('mavros/cmd/arming')
print ("mavros/cmd/arming service is active...")
rospy.wait_for_service ('mavros/set_mode')
print ("mavros/set_mode service is active...")

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)

rate = rospy.Rate (20)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the state to be connected")
    rate.sleep()

pose = PoseStamped ()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

print ("publishing target pos on the local pose topic ..")
for i in range (100):
    local_pos_pub.publish (pose)
    rate.sleep ()

last_request = rospy.Time.now ()

print ("trying to arm the vehicle..")
while not rospy.is_shutdown ():
    if ((current_state.mode != 'OFFBOARD') and \
        ((rospy.Time.now () - last_request) > rospy.Duration (5.0))):
        res = set_mode_client (custom_mode="OFFBOARD")
        if (res.mode_sent):
            print ("Offboard enabled")
        last_request = rospy.Time.now ()
    elif ((not current_state.armed) and \
        ((rospy.Time.now () - last_request) > rospy.Duration (5.0))):
        res = arming_client (value=True)
        if (res.success):
            print ("Vehicle armed")
        last_request = rospy.Time.now ()
    local_pos_pub.publish (pose)
    rate.sleep ()
