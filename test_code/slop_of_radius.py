#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
ax = plt.subplot(aspect='equal')
x = np.arange(0, 20, 0.1)
h, l = .60,.2 
#y = [(1/np.sqrt(1+h*h/((s+l)*(s+l))) ) for s in x]

y = [(((s+l)*(s+l) + 1)/np.sqrt((s+l)*(s+l) + h*h)) for s in x]
plt.plot(y)
ax.set_xlim(-1,20)
ax.set_ylim(-1,20)
plt.show()

