#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
#ax = plt.subplot(aspect='equal')
x = np.arange(0, 10, 0.1)
ha = np.arange(.7, 1, 0.1)
l = .2
y = [([(np.sqrt(h*h + (s+l)*(s+l)) - s -l ) for s in x]) for h in ha]
x = np.repeat([x], len(ha), axis=0)
i = 0
while i < len(ha):
    plt.plot(x[i],y[i], label="l={}".format(i*.1 + .6))
    i = i + 1
ax.set_xlim(0,10)
ax.set_ylim(0,2)
plt.legend()
plt.grid(True)
plt.show()

