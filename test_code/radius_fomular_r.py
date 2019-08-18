#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
#ax = plt.subplot(aspect='equal')
x = np.arange(0, 10, 0.1)
h = .70
la = np.arange(.2, 1, .2)
y = [([(np.sqrt(h*h + (s+l)*(s+l)) - s -l ) for s in x]) for l in la]
x = np.repeat([x], len(la), axis=0)
i = 0
while i < len(la):
    plt.plot(x[i],y[i], label="l={}".format(i*.2 + .2))
    i = i + 1
ax.set_xlim(0,10)
ax.set_ylim(0,.5)
plt.legend()
plt.grid(True)
plt.show()

