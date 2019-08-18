#!/usr/bin/python
from matplotlib import patches
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import utils

plt.rcParams["figure.figsize"] = (6,6)
fig, ax = plt.subplots()
p1 = (4.0,0.0)
p2 = (.0,.0)
p3 = (1.0,5.0)

line1 = np.array([p1,p2])
line2 = np.array([p2,p3])
points = utils.tangent_circle_segment(line1, line2, 2)
ax.plot(points[:,0],points[:,1],'-',color='b')
line1[1] = points[0]
line2[0] = points[-1]
ax.plot(line1[:,0], line1[:,1], '-', color='r')
ax.plot(line2[:,0], line2[:,1], '-', color='r')
ax.set_xlim(-1,6)
ax.set_ylim(-1,6)
plt.show()
