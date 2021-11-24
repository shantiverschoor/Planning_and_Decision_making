import random 
import math
import pygame

class RRT_map:
	def __init__(self, start_point, goal_point, map_dimensions, obs_dim, obs_num):
		self.start_point = start_point
		self.goal_point = goal_point
		self.map_dimensions	= map_dimensions
		self.map_height, self.map_width = self.map_dimensions

		self.map_window_name = "RRT path planning"
		pygame.display.set_caption(self.map_window_name)
		self.map = pygame.display.set_mode((self.map_width, self.map_height))
		self.map.fill((255,255,255))
		
		self.node_radius = 4
		self.node_thickness = 0
		self.edge_thickness = 0

		self.obstacles = []
		self.obsdim =  obs_dim
		self.obs_num = obs_num

		self.gray = (70,70,70)
		self.blue = (0,0,255)
		self.red = (255,0,0)
		self.green = (0,255,0)
		self.white = (255,255,255)

	def draw_map(self, obstacles):
		pygame.draw.circle(self.map, self.green, self.start_point, self.node_radius+10,0)
		pygame.draw.circle(self.map, self.red, self.goal_point, self.node_radius+20,0)
		self.draw_obstacles(obstacles)

	def draw_path(self):
		pass

	def draw_obstacles(self, obstacles):
		for obstacle in obstacles:
			pygame.draw.rect(self.map, self.gray, obstacle)
			
class RRT_graph:
	def __init__(self, start_point, goal_point, map_dimensions, obs_dim, obs_num):
		(x,y) = start_point
		self.start_point = start_point
		self.goal_point = goal_point
		self.goal_reached = False
		self.map_height, self.map_width = map_dimensions

		self.x = [x]
		self.y = [y]
		self.parent = [0]

		self.obstacles = []
		self.obs_dim =  obs_dim
		self.obs_num = obs_num

		self.goal_state = None
		self.path = []

	def make_rndm_rect(self):
		upper_corner_x = random.uniform(0, self.map_width-self.obs_dim)
		upper_corner_y = random.uniform(0, self.map_height-self.obs_dim)

		return(upper_corner_x, upper_corner_y)

	def make_obstacles(self):
		obstacles_to_make = []

		for i in range(0, self.obs_num):
			start_goal_obs_confl = False
			while not start_goal_obs_confl:
				rect = pygame.Rect(self.make_rndm_rect(), (self.obs_dim, self.obs_dim))
				if rect.collidepoint(self.start_point) or rect.collidepoint(self.goal_point):
					start_goal_obs_confl = False
				else:
					start_goal_obs_confl = True
				obstacles_to_make.append(rect)
				obstacles_to_make.append(pygame.Rect((100,100), (20,20)))
			
		self.obstacles = obstacles_to_make.copy()
		return obstacles_to_make

	def add_node(self, n, x, y):
		self.x.insert(n, x)
		self.y.append(y)
	def remove_node(self, n):
		self.x.pop(n)
		self.y.pop(n)

	def add_edge(self, parent, child):
		self.parent.insert(child, parent)

	def remove_edge(self):
		self.parent.pop(n)

	def number_of_nodes(self):
		return len(self.x)

	def distance(self, n1, n2):
		print("x: ", self.x)
		print("y: ", self.y)
		print(n1,n2)
		x1, y1 = self.x[n1], self.y[n1]
		x2, y2 = self.x[n2], self.y[n2]
		a, b = (x1-x2)**2, (y1-y2)**2
		return math.sqrt(a+b)

	def sample_map(self):
		x = random.uniform(0, self.map_width)
		y = random.uniform(0, self.map_height)
		return int(x), int(y)
		
	def nearest_neigh(self, n):
		dmin = self.distance(0, n)
		nnear = 0
		for i in range(0, n):
			if self.distance(i, n) < dmin:
				nnear = i

		print("nnear: ", nnear)
		return nnear

	def in_free_space(self):
		n = self.number_of_nodes()-1
		x, y = self.x[n], self.y[n]
		for obstacle in self.obstacles:
			if obstacle.collidepoint(x, y):
				self.remove_node(n)
				return False
		return True 

	def crosses_obstacle(self, x1, x2, y1, y2):
		for obstacle in self.obstacles:
			acc = 100
			for i in range(0,acc+1):
				x = i *((x2-x1)/acc) + x1
				y = i *((y2-y1)/acc) + y1

				if obstacle.collidepoint(x,y):
					return True
		return False

	def connect(self, n1, n2):
		(x1, y1) = (self.x[n1], self.y[n1])
		(x2, y2) = (self.x[n2], self.y[n2])
		if self.crosses_obstacle(x1, x2, y1, y2):
			self.remove_node(n2)
		else:
			self.add_edge(n1, n2)
			return True

	def step(self, nnear, nrand, dmax = 35):
		d = self.distance(nnear, nrand)
		if d > dmax:
			u = dmax/d
			x_near, y_near = self.x[nnear], self.y[nnear]
			x_rand, y_rand = self.x[nrand], self.y[nrand]

			px, py = x_rand-x_near, y_rand-y_near
			theta = math.atan2(px, py)
			x, y = int(x_near + dmax * math.cos(theta)), int(y_near + dmax * math.sin(theta))
			self.remove_node(nrand)

			if abs(x - self.goal_point[0] < dmax) and abs(y - self.goal_point[1] < dmax):
				self.add_node(nrand, self.goal_point[0], self.goal_point[1])
				self.goal_state = x_rand
				self.goal_reached = True
			else:
				self.add_node(nrand, x, y)

	def path_to_goals(self):
		pass

	def get_path_coords(self):
		pass

	def bias(self, ngoal):
		n = self.number_of_nodes()
		self.add_node(n, ngoal[0], ngoal[1])
		nnear = self.nearest_neigh(n)
		self.step(nnear, n)
		self.connect(nnear, n)
		return self.x, self.y, self.parent

	def expand(self):
		n = self.number_of_nodes()
		x, y = self.sample_map()
		self.add_node(n, x, y)
		if self.in_free_space():
			x_near = self.nearest_neigh(n)
			self.step(x_near, n)
			self.connect(x_near, n)
		return self.x, self.y, self.parent

	def cost(self):
		pass