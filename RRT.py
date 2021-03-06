import random
import math
import pygame
from copy import deepcopy
import numpy as np
from scipy import interpolate
import time

class RRTMap:
    def __init__(self, start, goal, MapDimensions):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.height, self.width = self.MapDimensions

        self.grey = (70, 70, 70)
        self.black = (0, 0, 0)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)
        self.white = (255, 255, 255)
        self.purple = (75, 0, 130)
        self.baby_blue = (137, 207, 240)

        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.fill(self.white)

        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1


    def drawMap(self, objects):
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.nodeRad+20, 1)
        self.drawObs(objects)

    def drawPath(self, path, raw=True, color=(0, 0, 255)):
        # Draw nodes in red that are in the path
        if raw:
            for node in path:
                pygame.draw.circle(self.map, self.red, node, self.nodeRad+3, 0)

        # Draw edges in red that are in the path
        for i in range(len(path) - 1):
            pos1, pos2 = path[i], path[i + 1]
            if raw:
                pygame.draw.line(self.map, self.red, pos1, pos2, self.edgeThickness)

            else:
                pygame.draw.line(self.map, self.green, pos1, pos2, self.edgeThickness+1)


    def drawObs(self, objects):
        # Draw the obstacles
        for obj in objects['obstacles']:
            pygame.draw.rect(self.map, self.black, obj)
        # Inflate obstacles with a safety marging to avoid collision
        for obj in objects['bumpers']:
            pygame.draw.rect(self.map, self.grey, obj, width=1)

    def configuration(self, path):
        angle1 = 270
        confgs = []

        # Extract the (x,y) pixel coordinates from the nodes
        for p, c in enumerate(path):
            if p == len(path)-1: # stop when the goal is achieved
                break

            x2, x1 =  path[p+1][0], c[0]
            y2, y1 =  path[p+1][1], c[1]

            # Using Pythagoras to find the angle that is tangent to path (line between two nodes)
            w, h = x2-x1, y2-y1
            r =(w**2 + h**2)**0.5
            angle2 =  270 - math.degrees(math.atan2(math.sin(math.asin(h/r)), math.cos(math.acos(w/r))))
            acc = 100   # Number of interpolation points betweed two vertices in graph;
                        # may determine driving speed of vehicle in animation due to computationel limitations 
            for i in range(0,acc+1):
                x = i *((x2-x1)/acc) + x1
                y = i *((y2-y1)/acc) + y1
                angle = i * (angle2-angle1)/acc + angle1

                confg = (x, y, angle) # Configuration of the car
                confgs.append(confg)

            angle1 = angle2

        return confgs

    def animate(self, obstacles, path, img):
        img = pygame.image.load(img)
        imgh = img.get_height()
        imgw = img.get_width()

        f = 0.05
        img = pygame.transform.scale(img, (imgw*f, imgh*f))

        imgh = img.get_height()
        imgw = img.get_width()

        angle1 = 270

        # Extract the (x,y) pixel coordinates from the nodes
        for p, c in enumerate(path):
            if p == len(path)-1: # stop when the goal is achieved
                break

            x2, x1 =  path[p+1][0], c[0]
            y2, y1 =  path[p+1][1], c[1]

            # Using pythagoras to find the angle that is tangent to path (line between two nodes)
            w, h = x2-x1, y2-y1

            angle2 =  270 - math.degrees(math.atan2(h,w))

            acc = 200 # speed of the car
            for i in range(0,acc+1):
                # Interpolation of the angles to find the orientation of the car
                x = i *((x2-x1)/acc) + x1
                y = i *((y2-y1)/acc) + y1
                angle = i * (angle2-angle1)/acc + angle1

                rotated_img = pygame.transform.rotate(img, np.abs(angle))

                loc = (x-imgw/2, y-imgh/2)
                new_rect = rotated_img.get_rect(center=img.get_rect(topleft = loc).center)

                self.map.fill(self.baby_blue)
                self.drawPath(path, raw=False)
                self.drawPath(path[:p+1], raw=False, color=self.yellow)
                self.drawMap(obstacles)
                self.map.blit(rotated_img, new_rect.topleft)
                pygame.display.update()
                pygame.event.clear()

            angle1 = angle2

class RRTGraph:
    def __init__(self, start, goal, MapDimensions):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.height, self.width = MapDimensions

        # Initiatie lists to store x and y coordinates and the parent of the child node
        self.x = []
        self.y = []
        self.parent = []

        # Initialize the tree structure
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        self.obstacles = []

        self.goalstate = None
        self.path = []

    # Make obstacles and safety boundaries
    def makeObs(self, rect_arguments):
        b = 60 # Set obstacles enlargement (inflation for safety)

        objects = {}
        obstacles = []
        bumpers = []

        for ob_arg in rect_arguments:
            obstacles.append(pygame.Rect(ob_arg))
            bumpers.append(pygame.Rect(ob_arg[0]-b/2, ob_arg[1]-b/2, ob_arg[2]+b, ob_arg[3]+b))

        objects['obstacles'] = obstacles
        objects['bumpers'] = bumpers

        self.obstacles = bumpers.copy() # Use this for checking whether samples crosses an obstacle
        return objects

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        return (px+py)**(0.5)

    # Sample random point within workspace
    def sample_env(self):
        x = int(random.uniform(0, self.width))
        y = int(random.uniform(0, self.height))
        return x, y

    # Check closest neighbors
    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    # Sample only in the Free space
    def isFree(self):
        n = self.number_of_nodes()-1 # check last node in the list
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rect = obs.pop(0)
            if rect.collidepoint(x, y):
                self.remove_node(n)
                return False # Sample is in the obstacle space
        return True # Sample is in the Free space

    # Check if the connection betweed nodes will intersect with an obstacle
    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rect = obs.pop(0)
            for i in range(0, 101): # interpolation between two points
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rect.collidepoint(x, y):
                    return True # edge does cross at least one obstacle
        return False # edge does not cross an obstacle

    # Create edge between two vertices
    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
        else:
            self.add_edge(n1, n2) # Create connection between two nodes if it doesn't crosses an obstacle
            return True

    # Create a node between two nodes given a predefined radius
    def step(self, nnear, nrand):
        d = self.distance(nnear, nrand)
        dmax = 30 # maximum radius of each edge (fine tune parameter)

        if d > dmax:
            u = dmax/d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])

            (px, py) = (xrand-xnear, yrand-ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax*math.cos(theta)),
                      int(ynear + dmax*math.sin(theta)))
            self.remove_node(nrand)

            # When close to the goal: instead of creating a random node, pick the goal node
            if (abs(x-self.goal[0]) < dmax) and (abs(y-self.goal[1]) < dmax):
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    # Get the parent of every node moving backwards from the goal node to the start node
    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0 ):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    # Get path coordinates from start to goal
    # Use when simulating the drive of the mobile robot
    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))

        return pathCoords

    def pathLength(self, list_):
        if len(list_) == 1:
            return 0
        else:
            diffx = list_[-1][0] - list_[-2][0]
            diffy = list_[-1][1] - list_[-2][1]
            dist = math.sqrt(diffx**2 + diffy**2)
            return dist + self.pathLength(list_[:-1])

    # Using heuristic function to expand in the direction of the goal
    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    # Random expansion
    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_env()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def smooth(self, path, weight_data=0.25, weight_smooth=0.4, tolerance=0.000001):
        """
        https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
        Creates a smooth path for a n-dimensional series of coordinates.
        Arguments:
            path: List containing coordinates of a path
            weight_data: Float, how much weight to update the data (alpha)
            weight_smooth: Float, how much weight to smooth the coordinates (beta).
            tolerance: Float, how much change per iteration is necessary to keep iterating.
        Output:
            new: List containing smoothed coordinates.
        """
        new = deepcopy(path)
        new = []
        for i in path:
            new.append(list(i))
        dims = len(path[0])
        change = tolerance

        while change >= tolerance:
            change = 0.0
            for i in range(1, len(new) - 1):
                for j in range(dims):
                    x_i = path[i][j]
                    y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                    y_i_saved = y_i
                    y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                    new[i][j] = y_i

                    change += abs(y_i - y_i_saved)

        newpath = []
        for i in new:
            newpath.append(list(i))
        return new

