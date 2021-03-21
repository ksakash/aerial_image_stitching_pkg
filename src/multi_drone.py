#!/usr/bin/env python3

import airsim
import cv2
import numpy as np
import os
import pprint
import setup_path
import tempfile
import threading
import time
from squaternion import Quaternion

robots = [0]
clients = []

intial_pos = [(0, 0), (1, 0), (0, 1), (1, 1)]
real_init_pos = [(0, 0), (5, 0)]

image_filename = 'airsimImage.txt'
file_handle = open (image_filename, 'w+')

for id in robots:
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    clients.append (client)

for id in robots:
    vehicle_name = "Drone" + str (id)
    clients[id].enableApiControl (True, vehicle_name)
    clients[id].armDisarm (True, vehicle_name)

airsim.wait_key('Press any key to takeoff')

def readInput (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    arr = []

    for line in lines:
        (x, y) = (int (line.split(' ')[0]), int (line.split(' ')[1]))
        arr.append ((x, y))

    return arr

h = 25
image_width = 1920
image_height = 1080
unit_y = 0.4 * h
unit_x = (image_height/image_width) * unit_y

def save_imu_pose (client, filename, id):
    filename = filename + '.jpg'
    vehicle_name = "Drone" + str (id)
    imu_data = client.getImuData (vehicle_name=vehicle_name)
    state = client.getMultirotorState (vehicle_name=vehicle_name)

    lon = state.kinematics_estimated.position.y_val + real_init_pos[id][0] # x val
    lat = state.kinematics_estimated.position.x_val + real_init_pos[id][1] # y val
    alt = state.kinematics_estimated.position.z_val

    w = imu_data.orientation.w_val
    x = imu_data.orientation.x_val
    y = imu_data.orientation.y_val
    z = imu_data.orientation.z_val

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
    global file_handle
    file_handle.write (st)

def save_image (client, tmp_dir, count, id):
    vehicle_name = "Drone" + str (id)
    responses = client.simGetImages ([airsim.ImageRequest ("1", airsim.ImageType.Scene, False, False)], \
                                      vehicle_name=vehicle_name)
    filename = os.path.join(tmp_dir, str(count))

    for idx, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress: # png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: # uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
            img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + '.jpg'), img_rgb) # write to png
    save_imu_pose (client, filename, id)

def perform_course (tmp_dir):
    waypoints = []
    for id in robots:
        filename = "waypoints_" + str (id)
        waypoint = readInput (filename)
        waypoints.append (waypoint)

    num_points = len (waypoints[0])
    max_speed = 2
    print ("no. of points:", num_points)

    image_count = 0
    for i in range (num_points):
        print ("point:", i)
        threads = []
        for id in robots:
            vehicle_name = "Drone" + str (id)
            x = (waypoints[id][i][1] - intial_pos[id][1]) * unit_x
            y = (waypoints[id][i][0] - intial_pos[id][0]) * unit_y
            z = -h
            print ("robot id:", id, "x:", y, "y:", x, "z:", z)
            threads.append (clients[id].moveToPositionAsync(x, y, z, \
                            max_speed, vehicle_name=vehicle_name))

        count = 0
        for th in threads:
            th.join ()
            save_image (clients[count], tmp_dir, image_count, count)
            count += 1
            image_count += 1

tmp_dir = "airsim_images"
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

perform_course (tmp_dir)

airsim.wait_key('Press any key to reset to original state')

for id in robots:
    vehicle_name = "Drone" + str (id)
    clients[id].armDisarm(False, vehicle_name)
    clients[id].reset()
    clients[id].enableApiControl(False, vehicle_name)
