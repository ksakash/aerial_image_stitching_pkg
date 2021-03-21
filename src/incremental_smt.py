#!/usr/bin/env python3

from z3 import *
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

import copy

def fill_obstacle_tocover (x_, y_, obstacles, visible, l, map):
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    x_l = int (max (0, x_-l))
    x_h = int (min (dimension_x, x_+l+1))
    y_l = int (max (0, y_-l))
    y_h = int (min (dimension_y, y_+l+1))
    for x in range (x_l, x_h):
        for y in range (y_l, y_h):
            if (map[y][x] == 0.0) and (x,y) not in obstacles:
                obstacles.append ((x,y))
            elif (map[y][x] == 0.5) and (x,y) not in visible:
                visible.append ((x,y))

def safe_space (x_, y_, l, map):
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    x_l = int (max (0, x_-l))
    x_h = int (min (dimension_x, x_+l+1))
    y_l = int (max (0, y_-l))
    y_h = int (min (dimension_y, y_+l+1))
    arr = []
    for x in range (x_l, x_h):
        for y in range (y_l, y_h):
            if map[y][x] == 0.5 or map[y][x] == 1.0:
                arr.append ((x,y))
    return arr

def get_rec (x, y, t, alpha):
    color = ''
    if t == 'covered':
        color = 'yellow'
    elif t == 'visible':
        color = 'green'
    elif t == 'obstacle':
        color = 'black'
    elif t == 'positions':
        color = 'blue'
    else:
        color = 'gray'

    rec = Rectangle ((x,y), 1, 1, linewidth=1, edgecolor='black', facecolor=color, alpha=alpha)
    return rec

def draw_rec (ax, arr, t, alpha):
    for (x,y) in arr:
        rect = get_rec (x, y, t, alpha)
        ax.add_patch (rect)

def visualize (ax, visible_array, obstacle_array, covered_array, alpha):
    draw_rec (ax, visible_array, 'visible', alpha)
    draw_rec (ax, obstacle_array, 'obstacle', alpha)
    draw_rec (ax, covered_array, 'covered', alpha)

map = np.array ([[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]])

local_map = np.full (map.shape, 0.5)

print (map.shape)

num_covered = 0
total = map.shape[0] * map.shape[1]
R = 1
T = 4

local_range = 1

start_x = np.empty ((R,))
start_y = np.empty ((R,))

start_x[0] = 0
# start_x[1] = 1
# start_x[2] = 0
# start_x[3] = 2

start_y[0] = 0
# start_y[1] = 0
# start_y[2] = 1
# start_y[3] = 2

dimension_x = map.shape[1]
dimension_y = map.shape[0]

obstacles = []
visible = []
covered = []

obstacles_added = []
visible_added = []
covered_added = []

for r in range (R):
    fill_obstacle_tocover (start_x[r], start_y[r], obstacles, visible, local_range, map)

visible_added = visible
obstacles_added = obstacles

k = 0
files = []

for r in range (R):
    filename = 'waypoints_' + str (r)
    f = open (filename, 'w+')
    files.append (f)

motion_w = 2 # cost of taking each step
old_w = 50 # reward of visiting old grid first
visible_w = 2 # cost of covering visible grids which are near
not_covered_w = 5 # cost of covering already covered grid
n_neighbors = 5
old_limit = 400
visible_dict = {}

fig, ax = plt.subplots (figsize=(dimension_x, dimension_y))
ax.set (xlim=(0,dimension_x), ylim=(0,dimension_y))
ax.axis ('off')
alpha = 1

uncovered = []
for x in range (dimension_x):
    for y in range (dimension_y):
        uncovered.append ((x,y))

draw_rec (ax, uncovered, 'uncovered', alpha)
prev_positions = []

prev_coord = []
for r in range (R):
    prev_coord.append (None)

while num_covered < 0.97 * total:

    # print ("obstacles added:",obstacles_added)
    # print ("covered added:",covered_added)
    # print ("visible added:",visible_added)

    visualize (ax, visible_added, obstacles_added, covered_added, alpha)
    positions = [(start_x[r], start_y[r]) for r in range (R)]
    draw_rec (ax, prev_positions, 'covered', alpha)
    draw_rec (ax, positions, 'positions', alpha)
    prev_positions = copy.copy (positions)

    for r in range (R):
        print ("robot " + str (r) + ":", start_x[r], start_y[r])

    num_visible = len (visible)

    s = Optimize ()
    total_c = Int ('total_c')
    total_re = Int ('total_re')

    X = [[Int("x_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    Y = [[Int("y_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    P = [[Int("p_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    S = [[Int("s_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    NC = [[Int("NC_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

    Re = [Int("re_%s" % (j)) for j in range(num_visible)]

    C = [[Int("c_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

    total_cost_array = Re
    for r in range (R):
        total_cost_array += S[r] + C[r] + NC[r]

    s.add (total_c == Sum (total_cost_array))

    for r in range (R):
        s.add (And (X[r][0] == start_x[r], Y[r][0] == start_y[r]))
        s.add (Or (X[r][T-1] != start_x[r], Y[r][T-1] != start_y[r]))

    # print ("obstacles:", obstacles)
    # print ("visible:", visible)

    '''
    for i in range (num_visible):
        s.add (Or (And (Re[i] <= old_w * num_visible, Re[i] >= 0), Re[i] == 100))
    '''

    for i in range (num_visible):
        s.add (Or (And (Re[i] <= 0, Re[i] >= -old_limit), Re[i] == 100))

    # obstacle avoidance
    for r in range (R):
        for t in range (T):
            for obst in obstacles:
                s.add (Or (X[r][t] != obst[0], Y[r][t] != obst[1]))

            # stay within bounds
            s.add (And (X[r][t] < dimension_x, X[r][t] >= 0))
            s.add (And (Y[r][t] < dimension_y, Y[r][t] >= 0))

            s.add (And (P[r][t] < 5, P[r][t] >= 0))
            s.add (Or (C[r][t] == 3 * motion_w, C[r][t] == 5 * motion_w))
            s.add (And (S[r][t] >= 0, S[r][t] <= visible_w * (dimension_x + dimension_y)))
            s.add (Or (NC[r][t] == not_covered_w, NC[r][t] == 0))

    # motion primitives
    for r in range (R):
        for t in range (T-1):
            s.add(Implies(P[r][t] == 0, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t], C[r][t] == 3 * motion_w))) # same
            s.add(Implies(P[r][t] == 1, And(X[r][t+1] == X[r][t]+1, Y[r][t+1] == Y[r][t], C[r][t] == 5 * motion_w))) # right
            s.add(Implies(P[r][t] == 2, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]+1, C[r][t] == 5 * motion_w))) # up
            s.add(Implies(P[r][t] == 3, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]-1, C[r][t] == 5 * motion_w))) # down
            s.add(Implies(P[r][t] == 4, And(X[r][t+1] == X[r][t]-1, Y[r][t+1] == Y[r][t], C[r][t] == 5 * motion_w))) # left

    # collision avoidance
    for t in range(0, T):
        for r1 in range (R):
            for r2 in range (R):
                if (r1 != r2 and r1 < r2):
                    s.add (Or(X[r1][t] != X[r2][t], Y[r1][t] != Y[r2][t]))

    safe = []
    for r in range (R):
        safe += safe_space (start_x[r], start_y[r], local_range, map) # need to change safe space
    safe += covered

    # local bound constraints
    for r in range (R):
        for t in range (T):
            s.add (Or ([And (X[r][t] == x, Y[r][t] == y) for (x, y) in safe]))

    for (x,y) in visible:
        if (x,y) not in visible_dict:
            visible_dict[(x,y)] = 0
        else:
            if visible_dict[(x,y)] > -old_limit:
                visible_dict[(x,y)] -= old_w
            else:
                visible_dict[(x,y)] = -old_limit

    # cover as many visible space as possible
    count = 0
    for (x,y) in visible:
        s.add (Implies (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)]), Re[count] == visible_dict[(x,y)]))
        s.add (Implies (Not (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)])), Re[count] == 100))
        count += 1

    '''
    # cover as many visible space as possible
    count = 0
    for (x,y) in visible:
        s.add (Implies (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)]), Re[count] == old_w * count))
        s.add (Implies (Not (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)])), Re[count] == 100))
        count += 1
    '''

    # if lost then the robot should gravitate towards the visible grids
    for r in range (R):
        for t in range (T):
            for (x,y) in safe:
                cost = 0
                arr = copy.copy (visible)
                arr = np.array (arr)
                dist = abs (arr[:,0] - x) + abs (arr[:,1] - y)
                ind = dist.argsort ()[:n_neighbors]
                dist = dist[ind]
                for item in dist:
                    cost += item
                '''
                for i in range (num_visible):
                    cost += (abs (visible[i][0] - x) + abs (visible[i][1] - y))
                cost = visible_w * int (cost / num_visible)
                '''
                cost = visible_w * int (cost / dist.shape[0])
                s.add (Implies (And (X[r][t] == x, Y[r][t] == y), S[r][t] == cost))

    for r in range (R):
        for t in range (T):
            s.add (Implies (Or ([And (X[r][t] == x, Y[r][t] == y) for (x,y) in covered]), NC[r][t] == not_covered_w))
            s.add (Implies (Not (Or ([And (X[r][t] == x, Y[r][t] == y) for (x,y) in covered])), NC[r][t] == 0))

    h = s.minimize (total_c)

    if str (s.check ()) == 'unsat':
        print ("UNSAT!")
        break

    model = s.model ()

    for r in range (R):
        for t in range (T):
            print (model[X[r][t]], model[Y[r][t]])
        print ('-------------------')

    for r in range (R):
        start_x[r] = int (str (model[X[r][T-1]]))
        start_y[r] = int (str (model[Y[r][T-1]]))

    covered_added = []
    visible_added = []
    obstacles_added = []
    temp_obst = copy.copy (obstacles)
    temp_visible = copy.copy (visible)

    # remove the grids already covered
    for r in range (R):
        for t in range (T):
            i = int (str (model[Y[r][t]]))
            j = int (str (model[X[r][t]]))
            local_map[i][j] = 1.0
            map[i][j] = 1.0
            if (j, i) not in covered:
                covered_added.append ((j,i))
            covered.append ((j,i))
            if ((j,i) in visible):
                visible.remove ((j,i))
                visible_dict.pop ((j,i))

    # fill the visible array from the trajectory that robot followed
    for r in range (R):
        for t in range (T):
            i = int (str (model[Y[r][t]]))
            j = int (str (model[X[r][t]]))
            fill_obstacle_tocover (j, i, obstacles, visible, local_range, map)
            if (j,i) == prev_coord[r]:
                continue
            s = str (j) + " " + str (i) + "\n"
            files[r].write (s)
            files[r].flush ()
            prev_coord[r] = (j,i)

    for v in visible:
        if v not in temp_visible:
            visible_added.append (v)

    for obst in obstacles:
        i = obst[1]
        j = obst[0]
        local_map[i][j] = 0.0

        if obst not in temp_obst:
            obstacles_added.append (obst)

    # print (local_map)
    # print (visible_dict)

    # print ("no. of 1s and 0s", np.count_nonzero (local_map == 1.0), len (obstacles))
    num_covered = np.count_nonzero (local_map == 1.0) + len (obstacles)

    print ("no. of cells covered:", num_covered)
    print ("horizon:", k)

    k += 1

    plt.pause (0.01)

for f in files:
    f.close ()
