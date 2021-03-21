#!/usr/bin/env python3

import open3d
import numpy as np

x = np.linspace (-3, 3, 401)

mesh_x, mesh_y = np.meshgrid (x, x)

z = np.sinc ((np.power (mesh_x, 2) + np.power (mesh_y, 2)))
z_norm = (z - z.min()) / (z.max () - z.min ())

xyz = np.zeros ((np.size (mesh_x), 3))
xyz[:,0] = np.reshape (mesh_x, -1)
xyz[:,1] = np.reshape (mesh_y, -1)
xyz[:,2] = np.reshape (z_norm, -1)

pcd = open3d.geometry.PointCloud()
pcd.points = open3d.utility.Vector3dVector (xyz)

open3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

