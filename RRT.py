import pygame

from RRT_helper import RRT_graph
from RRT_helper import RRT_map

import random
import time

def main():
	dimensions = (800,800)
	start = (50,50)
	goal = (800,400)
	obstacle_dimension = 40
	num = 4

	map_ = RRT_map(start, goal, dimensions, obstacle_dimension, num)
	graph = RRT_graph(start, goal, dimensions, obstacle_dimension, num)

	obstacles = graph.make_obstacles()
	map_.draw_map(obstacles)

	for i in range(500):
		if i % 10 == 0:
			x, y, parent = graph.bias(goal)
			pygame.draw.circle(map_.map, map_.red, (x[-1], y[-1]), map_.node_radius, 0)
			pygame.draw.line(map_.map, map_.red, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), map_.edge_thickness)

		else:
			x, y, parent = graph.expand()
			pygame.draw.circle(map_.map, map_.red, (x[-1], y[-1]), map_.node_radius, 0)
			pygame.draw.line(map_.map, map_.red, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), map_.edge_thickness)

		if i % 5 == 0:
			pygame.display.update()

	pygame.event.clear()
	pygame.event.wait(0)

if __name__ == "__main__":
	main()