#!/usr/bin/env python3

import numpy as np
from scipy.spatial import Delaunay

N = 100
p = np.random.rand (N, 3)
tri = Delaunay (p)
s = tri.simplices
e1 = p[s[:,1],:] - p[s[:,0],:]
e2 = p[s[:,2],:] - p[s[:,0],:]
e3 = p[s[:,3],:] - p[s[:,0],:]

t1 = np.cross (e1, e2, axis=1)
t2 = np.einsum ('ij,ij->i', t1, e3)
t2 = abs (t2) / 6
vol = np.sum (t2)
print ('volume:', vol)

