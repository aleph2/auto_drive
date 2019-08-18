#!/usr/bin/python
import matplotlib.pyplot as plt
ax = plt.subplot(aspect='equal')
a = [(1.2041495755897975, 0.6573127121093038), (1.5043460283843462, 1.679354557080379)]
b = []
b.append(a)
print b
x = a[0]
y = a[1]
plt.plot(x,y)
#plt.plot(x,y)
plt.ylabel('some numbers')
ax.set_xlim(-1,5)
ax.set_ylim(-1,5)
plt.show()
