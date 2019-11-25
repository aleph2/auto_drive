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
        self.constrs = []
    def loadPath(self, init_point, svg_file, origin,width, height, resolution, segment):
        xmldoc = minidom.parse(svg_file)
        path_nodes = xmldoc.getElementsByTagName('path') 
       # paths_string = [path.getAttribute('d') for path in path_nodes ]
        paths_string = [path.getAttribute('d') for path in path_nodes if path.parentNode.tagName == 'svg']
        constr_string = [path.getAttribute('d') for path in path_nodes if path.parentNode.tagName == 'g']
        path_strings = [path.getAttribute('d') for path in xmldoc.getElementsByTagName('path')]
        self.resolution = resolution 
        self.origin = origin
        self.height = height
        self.segment = segment
        self.paths = [self.svg2path(row_path) for row_path in paths_string]
        self.paths[0].insert(0, init_point)
        self.parseConstr(constr_string)
    def parseConstr(self, constr_strings):
        constrs = [parse_path(row_path) for row_path in constr_strings]
        map(lambda constr : self.constrs.extend(self.pointsOfFullPath(constr, self.segment)), constrs)
    def svg2path(self, row_path):
        ps = parse_path(row_path)
        path = []
        self.path_idx = [0]
        path.append(self.point2Array(ps[0].point(0))) 
        valide_paths = [p for p in ps if p.length() > 0 ]   
        map(lambda p : path.extend(self.pointsOfNavPath(p, self.segment)[1:]) , valide_paths)
        return path
    def pointsOfFullPath(self, p, segment):
        curve_len = p.length()*self.resolution;
        curve =  [self.point2Array(p.point(x)) for x in np.arange(0, 1, segment/curve_len)]
        return curve    
    def pointsOfNavPath(self, p, segment):
        if isinstance(p, svg.path.Line):
            self.path_idx.append(self.path_idx[-1] + 1)
            return [self.point2Array(p.point(0)), self.point2Array(p.point(1))]
        return self.pointsOfFullPath(p, segment)

    def point2Array(self, point):
        return np.array([point.real, self.height - point.imag])*self.resolution + self.origin
