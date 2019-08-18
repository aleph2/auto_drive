#!/usr/bin/python
import math
import matplotlib.pyplot as plt
from math import pi
import numpy as np
import utils
from scipy.spatial import ConvexHull
from shapely.geometry import LineString
from shapely.geometry import Point
def PointsInCircum(cent, r,n=100):
    px, py = cent
    return [(px + math.cos(2*pi/n*x)*r  , py+math.sin(2*pi/n*x)*r)  for x in xrange(0,n+1)]

def test(ax, cnt, dist):
    cent = 2,2
    points = PointsInCircum(cent, 2.5, cnt)
    for point in points:
        line1, line2, line3 = right_parall((2,2), point, dist)
        ax.plot(line1[:,0], line1[:,1], '-', color='b')
        ax.plot(line2[:,0], line2[:,1], '-', color='k')
        ax.plot(line3[:,0], line3[:,1], '-', color='r')
ax = plt.subplot(aspect='equal')
plt.rcParams["figure.figsize"] = (10,10)
#test(ax, 2, 2)
init_point = [0.,8.]
points = np.array([[ 3.89249297,  4.830331  ],
       [ 1.49239712,  4.87183442],
       [ 5.74384336,  2.9175217 ],
       [ 5.79473849,  5.28523338],
       [ 1.76526573,  6.38559647],
       [ 8.20160261,  4.56009111],
       [ 8.37290608,  6.80455167],
       [ 2.31792145,  8.10613093],
       [ 3.15127403,  3.72835402],
       [ 5.1708471 ,  2.32274065],
       [ 5.25753655,  4.20066954],
       [ 3.46612351,  5.54606794],
       [ 3.77500978,  3.1660874 ],
       [ 1.50743819,  1.26271003],
       [ 4.72331086,  7.02764254],
       [ 5.10634964,  2.64737931],
       [ 8.97578669,  8.02008634],
       [ 1.74042989,  0.75657199],
       [ 2.72029368,  3.84578814],
       [ 0.98003488,  3.28489766],
       [ 8.11850315,  8.79264186],
       [ 6.16364891,  8.14669341],
       [ 0.23849892,  8.20779404],
       [ 0.01619374,  2.10170429],
       [ 7.73888849,  5.37719602],
       [ 4.70695023,  7.19321801],
       [ 6.84227219,  6.73097437],
       [ 8.64082907,  6.34397411],
       [ 4.79756232,  0.9087276 ],
       [ 3.69644477,  0.39261183]])


points = [[0.0,0.0],[0.5,5.5],[8.3,8.6],[7.6,0.3]]
#points = 10* np.random.rand(30, 2)
def griding_path(init_point, bounday_points):
    i = 0
    path = []
    points = bounday_points
    while i < 100 :
        i = i + 1
        try:
            points = utils.convex_hull(points)
        except Exception:
            break

        lines = utils.points_2_lines(points)
        lines = utils.shift2maxline(lines)
        points = utils.pointsoflines(lines)
        if i == 1:
            wall = np.append(points, points[:1],axis=0)
            ax.plot(wall[:,0],wall[:,1],'-',color='r')
        elif i == 2:
            size = len(lines)
            first_line = lines[0]
            start_point = utils.line_middle(first_line[0], first_line[1],-.1)
            end_point = utils.line_middle(first_line[0], first_line[1],-0.1)
            path.append(init_point)
            arc_points = utils.tangent_circle_segment(np.array([init_point,start_point]), np.array([start_point, first_line[1]]), .3)
            path.extend(arc_points)
            for j in range(size):
                line1 = lines[j]
                line2 = lines[(j+1)%size]
                arc_points = utils.tangent_circle_segment(lines[j], lines[(j + 1)%size], .3)
                #if j == 0:
                 #   start_point = arc_points[0]
    #            ax.plot(arc_points[:,0],arc_points[:,1],'-',color='r')
    #            path.append(line1[0].tolist())
                path.extend(arc_points)
            path.append(end_point)
        else :
            first_line = lines[0]
            start_point = utils.line_middle(first_line[0], first_line[1],-0.1)
            end_point = utils.line_middle(first_line[0], first_line[1], -0.1)
            path.append(start_point)
            path.extend(points[1:])
            path.append(points[0])
            path.append(end_point)
    #    [ax.plot(line[:,0], line[:,1], '-', color='b') for line in lines]
        lines = utils.zoom_in_convex(points, .6) 
        if lines.size == 0:
            break
    #    tmp_points = np.append(points, points[0:1], axis=0)
    #    ax.plot(tmp_points[:,0],tmp_points[:,1],'-',color='b')

        points = utils.inner_convex_points(lines)
    #innerpoints = inner_convex_points(lines)
    pointsary = np.array(path)
    return pointsary
#print pointsary
pointsary = griding_path(init_point, points)
ax.plot(pointsary[:,0],pointsary[:,1],'-',color='b')
#lines = zoom_in_convex(ax, innerpoints, .5)
ax.set_xlim(-1,10)

ax.set_ylim(-1,10)
plt.show()
