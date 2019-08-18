#!/usr/bin/python
import math
import matplotlib.pyplot as plt
from math import pi
import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import LineString
from shapely.geometry import Point
def rotate(origin, point, sinv):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in sin value.
    """
    ox, oy = origin
    px, py = point
    cosv = math.sqrt(1 - sinv*sinv) 
    qx = ox + cosv * (px - ox) - sinv * (py - oy)
    qy = oy + sinv * (px - ox) + cosv * (py - oy)
    return qx, qy
def pointsoflines(lines):
    return lines[::,0]
def line_middle(origin, point, drift):
    norm_v = point - origin
    dist =  .5*np.linalg.norm(norm_v) + drift 
    middle = scale(origin, point, dist)
    return middle
def scale(origin, point, len):
        unit_v = (point - origin)/np.linalg.norm(point - origin)
        return unit_v*len + origin

def translate(origin, point, dist):
    """
    translate geometry vector with dist keeping its origin direction
    """
    ox, oy = origin
    px, py = point
    len = math.sqrt((px-ox)*(px-ox) + (py-oy)*(py-oy))
    qx = (px - ox) * dist / len
    qy = (py - oy) * dist / len
    return qx, qy
def shift2maxline(lines):
    maxid, _ = max(enumerate(lines), key = lambda l: np.linalg.norm(l[1][1] - l[1][0]))
    return np.roll(lines, -1*maxid, axis=0)
def lines_sharp_angle_cos(line1, line2):
    """
    line2 must be succss of line1, meaning line2's start point is line1's end point
    """
    norm_1 = line1[0] - line1[1]
    norm_2 = line2[1] - line2[0]
    cos_angle = np.dot(norm_1, norm_2) / (np.linalg.norm(norm_1) * np.linalg.norm(norm_2))
    return cos_angle

def line_axis_angle(line):
    norm = line[1] - line[0]
    return np.angle(norm[...,0] + 1j*norm[...,1])

def tangent_circle_segment(line1, line2, radius):
    cos_angle = lines_sharp_angle_cos(line1, line2)
    angle = np.arccos(cos_angle)
    R = radius / np.sin(angle/2)
    tmp = rotate(line2[0], line2[1], -1*np.sqrt((1 - cos_angle)*0.5 ))
    end_angle = np.pi*0.5 + line_axis_angle(line1)
    start_angle = angle + end_angle - np.pi
    delta_angle = np.arange(end_angle, start_angle + np.pi/180, -1 * np.pi/180)
    cent_point = scale(line2[0], tmp, R)
    x = radius*np.cos(delta_angle ) + cent_point[0]
    y = radius*np.sin(delta_angle ) + cent_point[1]
    return np.stack((x,y), axis=-1)

def right_parall(origin, point, dist):
    """
    create parallel vector at given vector right side
    """
    x0, y0 = origin
    x1, y1 = point
    start = origin
    end = point
    len = math.sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0))
#    sinv = dist/len
    tanv = dist/len
    cosv = math.sqrt(1/(1+tanv*tanv))
    sinv = cosv*tanv
    t_dist = len/cosv - len
    n = rotate(start, end, -1*sinv)
    dx, dy = translate(start, n, t_dist)
    ns = start[0] + dx, start[1] + dy
    ne = n[0] + dx, n[1] + dy
    n1 = rotate(ne, ns, sinv)
    
    line1 = np.array([start , end])
    line2 = np.array([ns, ne])
    line3 = np.array([n1, ne])
    return line1, line2, line3

def points_2_lines(points):
    points = np.append(points, points[0:1],axis=0)
    lines = np.array(zip(points, points[1::]))
    return lines
def convex_hull(points):
    hull = ConvexHull(points)
    indices = hull.vertices[::-1]
    points = np.take(hull.points, indices, axis=0)
    return points
"""
points should be in the results of convex hull from numpy
return value: every two elements denotes a line start and end point
Those lines are disconnected to each other
"""
def zoom_in_convex(points, dist):
    lines = np.empty(shape=[0,2])
    try:
        hull = ConvexHull(points)
    except Exception:
        return lines
    indices = hull.vertices[::-1]
    points = np.take(hull.points, indices, axis=0)
    for i in range(len(points) - 1):
        start = points[i]
        point = points[i+1]
        line1, line2, line3 = right_parall(start, point, dist)
#        ax.plot(line1[:,0], line1[:,1], '-', color=c)
        lines = np.append(lines, line3, axis=0)
    start = points[-1]
    point = points[0]
    line1, line2, line3 = right_parall(start, point, dist)
#    ax.plot(line1[:,0], line1[:,1], '-', color=c)
    lines = np.append(lines,line3, axis=0)
    return lines

def inner_convex_points(lines):
    linepoints = lines.tolist()
    lineary = zip(linepoints, linepoints[1:])[::2]
    geolines = map(lambda x: LineString(x), lineary)
    maxidx, _ = max(enumerate(geolines), key = lambda l:l[1].length)
    rotatedlines = geolines[maxidx:] + geolines[:maxidx]
    linesize = len(geolines)
    pointsDict = dict()
    idx = 0
    idxchain = np.full(linesize, fill_value=-1,  dtype=np.int)
    for idx, geoline in enumerate(rotatedlines):
        succ = idx + 1
        minpoint = None
        mindist = float("inf")
        dist = 0
        origin = Point(geoline.coords[0])
        dest = Point(geoline.coords[1])
        x1,y1 = origin.coords[:][0]
        x2,y2 = dest.coords[:][0]
        nextidx = -1
        while succ < idx + linesize - 1:
            #the line end point is on left side of current line, will discard it
            succLine = rotatedlines[succ%linesize]
            succEndPoint = succLine.coords[1]
            x,y = succEndPoint
            if (x - x1)*(y2-y1) - (y-y1)*(x2-x1) < 0:
                succ = succ + 1
                continue
            cp = geoline.intersection(rotatedlines[succ % linesize])
            dist = origin.distance(cp)
            if cp.is_empty:
                succ = succ + 1 
                continue
            if dist < mindist :
                minpoint, mindist = cp, dist
                nextidx = succ%linesize
            succ = succ + 1
        pointsDict[idx] = minpoint
        try:
            idxchain[idx] = nextidx
        except Exception as error:
            print "The line size is", linesize
            print error
            raise Exception(error)

    print idxchain 
    newpoints = []
    idx = 0
    while idxchain[idx] != -1:
        dicPoint = pointsDict[idx]
        if dicPoint == None:
            break
        try:
            newpoints.append(list(pointsDict[idx].coords[:][0]))
        except Exception as error:
            print "The size of points Dict is", len(pointsDict)
            raise Exception(error)
        tmp = idxchain[idx]
        idxchain[idx] = -1
        idx = tmp
    return newpoints
if __name__ == "__main__":

    ax = plt.subplot(aspect='equal')
    #test(ax, 2, 2)
    points = 5* np.random.rand(30, 2)
    i = 0
    alllines = []
    while i < 100 :
        color = 'b'
        if i == 0:
            color = 'r'
        i = i + 1
        lines = zoom_in_convex(ax, points,.5, color) 
        alllines.append(lines)
        if lines.size == 0:
            break
        points = inner_convex_points(lines)
    #innerpoints = inner_convex_points(lines)
    #lines = zoom_in_convex(ax, innerpoints, .5)
    print "----------------------"
    for lines in alllines:
        print lines
        print "========================"
    ax.set_xlim(-1,5)

    ax.set_ylim(-1,5)
    plt.show()
