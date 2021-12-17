from RRT_map import RRT_map as rrt
from obstacle_maps import *

#Set width and height also in obstacle_maps.py
h, w = 600, 1000
b = rrt(h, w)

b.set_start(50, 50)
b.set_goal(900, 550)

obstacles = sim2

for key in obstacles:
	b.add_obstacle(key, obstacles[key])

GRID = False

if GRID:
	stepsize = 50
	offset = stepsize/2
	for n in numpy.arange(0, h, stepsize):
		for m in numpy.arange(0, w, stepsize):
			if b.in_free_space((m, n)):
				b.add_vertix(m+offset, n+offset)

b.RRT()