#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.lines as Line2D
fig, ax = plt.subplots()

fig.set_size_inches(6,6)          # Make graph square
#scatter([-0.1],[-0.1],s=0.01)     # Move graph window a little left and down

line1 = [(0,0), (1,0)]
line2 = [(0,0), (0,1)]

# Note that the Line2D takes a list of x values and a list of y values,
# not 2 points as one might expect.  So we have to convert our points
# an x-list and a y-list.
(line1_xs, line1_ys) = zip(*line1)
(line2_xs, line2_ys) = zip(*line2)

ax.add_line(Line2D(line1_xs, line1_ys, linewidth=2, color='blue'))
ax.add_line(Line2D(line2_xs, line2_ys, linewidth=2, color='red'))
plot()
show()
