import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from math import *

#some helper values
p=4
theta=pi/6
x1 = p*cos(theta/2)
y1 = p*sin(theta/2)
vertices =[(-x1-p/2,0), (-p/2, y1), (p/2, y1), (x1+p/2, 0), (p/2, -y1), (-p/2, -y1)] 
midPoint = [3,4]

#set up the plot
fig = plt.figure()
ax = fig.add_subplot(111)

#function to rotate and translate the standard shape to a new position
def plot_polygon(vertices, midPoint, theta):
    polygon = patches.Polygon(vertices, color="red", alpha=0.50) 
#    r = mpl.transforms.Affine2D().rotate(theta) + ax.transData
#    t = mpl.transforms.Affine2D().translate(midPoint[0],midPoint[1]) + ax.transData
    r = mpl.transforms.Affine2D().rotate(theta)
    t = mpl.transforms.Affine2D().translate(midPoint[0],midPoint[1])
    tra = r + t + ax.transData
    polygon.set_transform(tra)
#    polygon.set_transform(r)
#    polygon.set_transform(t)
    ax.add_patch(polygon)

plot_polygon(vertices, midPoint, theta)

plt.xlim(-30, 30)
plt.ylim(-30, 30)

plt.grid(True)

plt.show()
