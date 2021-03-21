#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import time

def readInput (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    x_ = []
    y_ = []

    for line in lines:
        (x, y) = (int (line.split(' ')[0]), int (line.split(' ')[1]))
        x_.append (x)
        y_.append (y)

    return (x_, y_)

R = 2
x_arr = []
y_arr = []

for r in range (R):
    # filename = 'robot' + str (r) + '.plan'
    filename = '../plans/waypoints_' + str (r)
    (x, y) = readInput (filename)
    x_arr.append (x)
    y_arr.append (y)

obsx = [2, 3, 1, 3, 1, 2]
obsy = [0, 0, 2, 2, 4, 4]

initialx = []
initialy = []

for r in range (R):
    initialx.append (x_arr[r][0])
    initialy.append (y_arr[r][0])

plt.scatter (initialx, initialy, c=['r','b'])
plt.ylim ((-1, 9))
plt.xlim ((-1, 9))
plt.pause (1)

for i in range (0, len (x_arr[0]), 1):
    plt.plot (x_arr[0][i:i+2], y_arr[0][i:i+2], 'ro-')
    plt.plot (x_arr[1][i:i+2], y_arr[1][i:i+2], 'bo-')
    # plt.plot (x_arr[2][i:i+2], y_arr[2][i:i+2], 'go-')
    # plt.plot (x_arr[3][i:i+2], y_arr[3][i:i+2], 'yo-')
    plt.pause (0.1)

plt.show ()
