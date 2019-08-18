#!/usr/bin/python
import matplotlib.pyplot as plt
from numpy import *
import numpy as np
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       print "do not intersect"

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def perp( a ) :
    b = empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = dot( dap, db)
    num = dot( dap, dp )
    return (num / denom.astype(float))*db + b1

p1 = array( [0.0, 0.0] )
p2 = array( [1.0, 0.0] )

p3 = array( [4.0, -5.0] )
p4 = array( [4.0, 2.0] )

#print seg_intersect( p1,p2, p3,p4)

#p1 = array( [2.0, 2.0] )
#p2 = array( [4.0, 3.0] )

#p3 = array( [6.0, 0.0] )
#p4 = array( [6.0, 3.0] )

print seg_intersect( p1,p2, p3,p4)
ax = plt.subplot(aspect='equal')
line1 = np.array([p1, p2])
print line_intersection((p1,p2 ), (p3, p4))
line2 = np.array([p3 , p4])
ax.plot(line1[:,0], line1[:,1], '-', color='b')
ax.plot(line2[:,0], line2[:,1], '-', color='r')
ax.set_xlim(-5,10)
ax.set_ylim(-5,10)
plt.show()
