import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

fig = plt.figure()
ax = fig.add_subplot(111)
plt.plot([-3,3],[-3,3])

x = [-1,0,1,1,0,-1]
y = [1,1,1,-1,-1,-1]
scale = 2
poly = Polygon( np.c_[x,y]*scale, facecolor='red', edgecolor='red', alpha=0.5)
ax.add_patch(poly)

plt.show()
