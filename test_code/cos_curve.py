#!/usr/bin/python
import numpy as np

import matplotlib.pyplot as plot

import matplotlib.pyplot as plot


# Get x values of the cosine wave

time = np.arange(0, 20, .01);

print len(time) 

# Amplitude of the cosine wave is cosine of a variable like time

amplitude   = np.cos(time)

 

# Plot a cosine wave using time and amplitude obtained for the cosine wave
fig, ax = plot.subplots()
#plot.plot(time, amplitude)

c1 = plot.Circle((0, 0),2, color='r', fill=False)
ax.add_artist(c1)

# Give a title for the cosine wave plot

plot.title('Cosine wave')

 

# Give x axis label for the cosine wave plot

plot.xlabel('Time')

 

# Give y axis label for the cosine wave plot

plot.ylabel('Amplitude = cosine(time)')

 

# Draw the grid for the graph

plot.grid(True, which='both')

 

plot.axhline(y=0, color='b')

 

# Display the cosine wave plot
ax.set_xlim((-10, 10))
ax.set_ylim((-10, 10))
plot.show()
