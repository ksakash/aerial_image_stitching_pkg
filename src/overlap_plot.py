#!/usr/bin/env python3

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import random
import math

def readInput (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    arr = []

    for line in lines:
        (x, y) = (int (line.split(' ')[0]), int (line.split(' ')[1]))
        arr.append ((x, y))

    return arr

filename = 'waypoints_0'
points = readInput (filename)

filename = 'waypoints_1'
points1 = readInput (filename)

filename = 'waypoints_2'
points2 = readInput (filename)

filename = 'waypoints_3'
points3 = readInput (filename)

h = 2
width = 2 * h
height = 0.75 * width

padding = 0

x_origin = -(h + padding)
y_origin = -((height/2) + padding)

x_end = 8 - x_origin
y_end = 8 - y_origin

color = (0.5,0.5,0.5)
color1 = (0.5,0.5,0.5)
color2 = (0.5,0.5,0.5)
color3 = (0.5,0.5,0.5)

fig, ax = plt.subplots (figsize=(x_end-x_origin,y_end-y_origin))
ax.set (xlim=(x_origin,x_end), ylim=(y_origin,y_end))
# ax.axis ('off')
alpha = 0.1

c = 0
for p in points:
    xy = (p[0] - width/2, p[1] - height/2)
    rect = Rectangle (xy, width, height, linewidth=1, edgecolor='none', facecolor=color, alpha=alpha)
    # ax.add_patch (rect)

    xy1 = (points1[c][0] - width/2, points1[c][1] - height/2)
    rect1 = Rectangle (xy1, width, height, linewidth=1, edgecolor='none', facecolor=color1, alpha=alpha)
    # ax.add_patch (rect1)

    xy2 = (points2[c][0] - width/2, points2[c][1] - height/2)
    rect2 = Rectangle (xy2, width, height, linewidth=1, edgecolor='none', facecolor=color2, alpha=alpha)
    # ax.add_patch (rect2)

    xy3 = (points3[c][0] - width/2, points3[c][1] - height/2)
    rect3 = Rectangle (xy3, width, height, linewidth=1, edgecolor='none', facecolor=color3, alpha=alpha)
    # ax.add_patch (rect3)

    plt.scatter ([p[0]],[p[1]], c=['r'])
    plt.scatter ([points1[c][0]],[points1[c][1]], c=['g'])
    plt.scatter ([points2[c][0]],[points2[c][1]], c=['b'])
    plt.scatter ([points3[c][0]],[points3[c][1]], c=['c'])

    plt.pause (1)
    c += 1

plt.show ()
