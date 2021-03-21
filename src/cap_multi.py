#!/usr/bin/env python3

import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

import time
from squaternion import Quaternion

vehicles = [0, 1]

initial_pos = [(0, 0), (5, 0)]

image_filename = 'airsimImage.txt'
file_handle = open (image_filename, 'w+')

def save_imu_pose (client, filename, id):
    filename = filename + '.jpg'
    vehicle_name = "Drone" + str (id)
    imu_data = client.getImuData (vehicle_name=vehicle_name)
    state = client.getMultirotorState (vehicle_name=vehicle_name)

    lon = state.kinematics_estimated.position.y_val + initial_pos[id][0] # x val
    lat = state.kinematics_estimated.position.x_val + initial_pos[id][1] # y val
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

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

tmp_dir = "airsim_images"
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

count = 0
while (True):
    for id in vehicles:
        save_image (client, tmp_dir, count, id)
        count += 1
    time.sleep (0.5)
