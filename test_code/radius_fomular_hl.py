#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
#ax = plt.subplot(aspect='equal')
x = np.arange(0, 10, 0.1)
ha = np.arange(.3,.8 , 0.05)
la = np.arange(.125,.3,.1)
s = 5
plt.title("R={}".format(s))
y = [([(np.sqrt(h*h + (s+l)*(s+l)) - s -l ) for h in ha]) for l in la]
x = np.repeat([ha], len(la), axis=0)
i = 0
while i < len(la):
    plt.plot(x[i],y[i], label="l={}".format(i*.1 + .125))
    i = i + 1
ax.set_xlim(.0,.9)
ax.set_ylim(0.0,.2)
plt.legend()
plt.grid(True)
plt.show()

