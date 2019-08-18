from shapely.geometry import LineString
line1 = LineString([(0,0), (1,1)])
line2 = LineString([(0,1), (1,0)])
a = line1.intersection(line2)
print a.is_empty
