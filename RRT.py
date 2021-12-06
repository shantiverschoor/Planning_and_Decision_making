import pygame

from RRT_helper import RRT_graph
from RRT_helper import RRT_map

import random
import time

def main():
	dimensions = (800,800)
	start = (50,50)
	goal = (800,400)
	obstacle_dimension = 20
	num = 100

	map_ = RRT_map(start, goal, dimensions, obstacle_dimension, num)
	graph = RRT_graph(start, goal, dimensions, obstacle_dimension, num)

	obstacles = graph.make_obstacles()
	map_.draw_map(obstacles)

	# while True:

	# 	x, y = graph.sample_map()
	# 	n = graph.number_of_nodes()
	# 	graph.add_node(n, x, y)
	# 	graph.add_edge(n-1, n)
	# 	x1, y1 = graph.x[n], graph.y[n]
	# 	x2, y2 = graph.x[n-1], graph.y[n-1]
	# 	if graph.in_free_space():
	# 		pygame.draw.circle(map_.map, map_.red, (graph.x[n], graph.y[n]), map_.node_radius, map_.node_thickness)
	# 		if not graph.crosses_obstacle(x1, x2, y1, y2):
	# 			pygame.draw.line(map_.map, map_.blue, (x1, y1), (x2, y2), map_.edge_thickness-2)

	# 	pygame.display.update()

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