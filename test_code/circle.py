#!/usr/bin/python
from matplotlib import patches
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
plt.rcParams["figure.figsize"] = (10,10)
radius = 2.6
circle1 = plt.Circle((0, 0), 5, color='r', fill=False)
fig, ax = plt.subplots()    
arc = patches.Arc((0,0), 2*radius,2*radius,angle=.0, theta1=0, theta2=0, fill=False)
line, = ax.plot([], [], lw=1)
line_edge, = ax.plot([], [], lw=2, color='r')
ax.add_patch(arc)
#ax.add_artist(circle1)
ax.set_xlim((-4, 4))
ax.set_ylim((-4, 4))
ax.axhline(y=0, color='k')
ax.axvline(x=0, color='k')
ax.grid(True, which='both')
a, b = 0.6,radius + .2
a_edge, b_edge = -0.1, radius + .2
turn_angle = 360
aryx, aryy = [], []
aryx_edge, aryy_edge = [], []
def animate(i):
    global arc
    ax.patches.remove(arc)
    theta = -1 * i + 90
    rot = i*-1*np.pi/180.0
    arc = patches.Arc((0,0), 2*radius,2*radius,angle=0, theta1=theta, theta2= 90, fill=False)
    ax.add_patch(arc)
    x = a*np.cos(rot) - b*np.sin(rot)
    y = a*np.sin(rot) + b*np.cos(rot)
    x_edge = a_edge*np.cos(rot) - b_edge*np.sin(rot)
    y_edge = a_edge*np.sin(rot) + b_edge*np.cos(rot)
    global aryx, aryy
    global aryx_edge, aryy_edge
    if i > turn_angle - 1:
        aryx, aryy = list(), list()
        aryx_edge, aryy_edge = list(), list()
        line.set_data([],[])
        line_edge.set_data([],[])
        return
    aryx.append(x)
    aryy.append(y)
    aryx_edge.append(x_edge)
    aryy_edge.append(y_edge)
    line.set_data(aryx, aryy)
    line_edge.set_data(aryx_edge, aryy_edge)
"""
    x = np.linspace(0,i*5.0/90 , 1000)
    y = np.sin(2 * np.pi * (x - 0.01 * i))
"""
def init():
    line.set_data([], [])
    line_edge.set_data([], [])
    return None
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=turn_angle, interval=100, blit=False)
plt.show()
