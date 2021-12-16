import pygame
import random
import math
import time
import numpy as np

class Map():

	def __init__(self, height, width):
		self.map_height, self.map_width = height, width
		self.start = (None, None)
		self.goal = (None, None)
		self.marge = 20
		self.goal_node = None
		self.count_vert = 0

		self.bumper_radius = 5

		self.obstacles = [	pygame.Rect((0,0), (self.map_width, 5)),
							pygame.Rect((0,self.map_height-5), (self.map_width, 5)),
							pygame.Rect((0,0), (5, self.map_height)),
							pygame.Rect((self.map_width-5,0), (5,self.map_height))]

		self.obstacles_space = [pygame.Rect((0,0), (self.map_width, 5 + self.bumper_radius)),
								pygame.Rect((0,self.map_height - 5 - self.bumper_radius), (self.map_width, 5 + self.bumper_radius)),
								pygame.Rect((0,0), (5 + self.bumper_radius, self.map_height)),
								pygame.Rect((self.map_width - 5 - self.bumper_radius,0), (5 + self.bumper_radius,self.map_height))]

		self.vertices = {}
		self.connections = {}
		self.colors = {
			"black" : (0,0,0),
			"gray" : (70,70,70),
			"blue" : (0,0,255),
			"red" : (255,0,0),
			"green" : (0,255,0),
			"white" : (255,255,255),
			"orange": (255,102,0)}

		self.img_filename = None

		self.map_window_name = "RRT path planning"
		pygame.display.set_caption(self.map_window_name)
		self.map_= pygame.display.set_mode((width, height))
		self.map_.fill(self.colors['white'])

	def set_goal(self, x, y):
		#self.vertices['goal'] = (x, y)
		self.goal = (x,y)

	def set_start(self, x, y):
		self.vertices['start'] = (x, y)
		self.start = (x, y)

	def in_free_space(self, left_corner, ):
		for obstacle in self.obstacles_space:
			if obstacle.collidepoint(left_corner):
				return False
		return True

	def sample(self):
		x = random.uniform(0, self.map_width)
		y = random.uniform(0, self.map_height)

		while not self.in_free_space((x, y)):
			x = random.uniform(0, self.map_width)
			y = random.uniform(0, self.map_height)

		return x, y

	def add_vertix(self, x, y):
		self.count_vert += 1
		self.vertices[self.count_vert] = (x, y)
		return self.count_vert

	def add_edge(self, n1, n2):
		vertix1, vertix2 = self.vertices[n1], self.vertices[n2]
		self.connections[n2] = n1

	def add_obstacle(self, left_corner, dimensions):
		rect = pygame.Rect(left_corner, dimensions)
		self.obstacles.append(rect)

		os_left_corner = (left_corner[0] - self.bumper_radius, left_corner[1] - self.bumper_radius)
		os_dimensions = (dimensions[0] + 2*self.bumper_radius, dimensions[1] + 2*self.bumper_radius)
		os_rect = pygame.Rect(os_left_corner, os_dimensions)
		self.obstacles_space.append(os_rect)

	def draw_vertices(self):
		for key in self.vertices:
			if key == 'start':
				x, y = self.vertices[key]
				pygame.draw.circle(self.map_, self.colors['red'], (x,y), 4)
			elif key == 'goal':
				x, y = self.vertices[key]
				pygame.draw.circle(self.map_, self.colors['green'], (x,y), 6)
			else:
				x, y = self.vertices[key]
				pygame.draw.circle(self.map_, self.colors['blue'], (x, y), 5)

	def draw_edges(self):
		for key in self.connections:
			n1, n2 = self.connections[key], key
			v1 = self.vertices[n1]
			v2 = self.vertices[n2]
			pygame.draw.line(self.map_, self.colors['blue'], v1, v2, 2)

	def draw_obstacles(self):
		for obstacle in self.obstacles:
			pygame.draw.rect(self.map_, self.colors['black'], obstacle)

	def draw_goal(self):
		pygame.draw.circle(self.map_, self.colors['green'], self.goal, 12)

	def display(self):
		self.map_.fill(self.colors['white'])
		self.draw_obstacles()
		self.draw_vertices()
		self.draw_edges()
		self.draw_goal()
		pygame.display.update()

	def crosses_obstacles(self, n1, n2):
		vertix1, vertix2 = self.vertices[n1], self.vertices[n2]
		acc = 1000
		x1, y1 = vertix1
		x2, y2 = vertix2
		for obstacle in self.obstacles_space:
			for i in range(0,acc+1):
				x = i *((x2-x1)/acc) + x1
				y = i *((y2-y1)/acc) + y1

				if obstacle.collidepoint(x,y):
					return True
		return False

	def distance(self, a, b):
		return math.sqrt(a**2+b**2)

	def nearest(self, n):
		d = self.distance(self.map_width, self.map_height)
		i = None
		x, y = self.vertices[n]

		for key in self.vertices:
			if key == n:
				continue
			xtemp, ytemp = self.vertices[key]
			xdiff, ydiff = x-xtemp, y-ytemp
			dtemp = self.distance(xdiff, ydiff)
			if dtemp < d:
				d = dtemp
				i = key
		return i

	def is_goal(self, n):
		if self.goal[0] - self.marge < self.vertices[n][0] < self.goal[0] + self.marge\
		and self.goal[1] - self.marge < self.vertices[n][1] < self.goal[1] + self.marge:
			self.goal_node = n
			return True
		return False

	def dist(self, s1, s2):
		x1 = s1[0]
		y1 = s1[1]

		x2 = s2[0]
		y2 = s2[1]
		return np.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

	def RRT(self):
		goal_reached = False
		while not goal_reached:
			accp = False
			while not accp:
				xrand, yrand = self.sample()
				nrand = self.add_vertix(xrand, yrand)
				nnear = self.nearest(nrand)
				xnear, ynear = self.vertices[nnear]

				self.display()
				if not self.crosses_obstacles(nnear, nrand):

					self.add_edge(nnear, nrand)
					accp = True
					goal_reached = self.is_goal(nrand)
				else:
					self.remove_vertix(nrand)
				self.display()
		self.draw_path()

	def get_child(self, n):
		if n == 'start':
			return [n]
		else:
			child = self.connections[n]
			return  [n] + self.get_child(child)

	def draw_path(self):
		path = self.get_child(self.goal_node)
		for i in range(len(path)-1):
			pos1, pos2 = self.vertices[path[i]], self.vertices[path[i+1]]
			pygame.draw.line(self.map_, self.colors['red'], pos1, pos2, 4)
		pygame.display.update()
		print("Nodes used: ",self.count_vert)

	def remove_vertix(self, n):
		del self.vertices[n]
		self.count_vert -= 1
		self.draw_vertices()


h, w = 600, 1000
b = Map(h, w)

b.set_start(50, 50)
b.set_goal(950, 550)

wall = 10

sim1 = {(0,100): (200,wall),
		(300,300): (50,30),
		(400,0): (wall,460),
		(0,450): (150,wall),
		(150,350): (10,200),
		(500,100): (50,30),
		(500,400): (50,30),
		(660,300): (50,30),
		(800, 200): (wall, 400)}


sim2 = {(0,150): (400,wall),
	   (550+wall,0): (wall,h-150),
	   (400,150): (wall,h-150),
	   (550+wall,h-150): (w-150, wall)}

obstacles = sim1

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

pygame.event.clear()
pygame.event.wait(0)