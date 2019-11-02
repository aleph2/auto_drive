#!/usr/bin/python
import math
from math import pi
import numpy as np
from xml.dom import minidom
import svg
from svg.path import parse_path
class PathParser:

    def __init__(self):
        self.path_idx = []
        self.path = []

    def getPath(self, init_point, svg_file, origin,width, height, resolution, segment):
        xmldoc = minidom.parse(svg_file)
        path_strings = [path.getAttribute('d') for path in xmldoc.getElementsByTagName('path')]
        ps = parse_path(path_strings[0])
        self.resolution = resolution 
        self.origin = origin
        self.height = height
        self.path = [init_point]
        self.path_idx = [0]
        self.path.append(self.point2Array(ps[0].point(0))) 
        self.path_idx.append(1) 
        valide_paths = [p for p in ps if p.length() > 0 ]   
        map(lambda p : self.path.extend(self.pointsOfPath(p, segment)[1:]) , valide_paths)

    def pointsOfPath(self, p, segment):
        if isinstance(p, svg.path.Line):
            self.path_idx.append(self.path_idx[-1] + 1)
            return [self.point2Array(p.point(0)), self.point2Array(p.point(1))]
        
        curve =  [self.point2Array(p.point(x)) for x in np.arange(0, 1, segment/(p.length()*self.resolution))]
        self.path_idx.append(self.path_idx[-1] + len(curve) - 1)
        return curve

    def point2Array(self, point):
        return np.array([point.real, self.height - point.imag])*self.resolution + self.origin
