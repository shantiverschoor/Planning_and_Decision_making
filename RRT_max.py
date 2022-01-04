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

        # Colors
        self.grey = (70, 70, 70)
        self.black = (0, 0, 0)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.purple = (75, 0, 130)

        # Window settings
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.fill(self.white)

        # Node and edges
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        # Obstacles
        self.obstacles = []

    def drawMap(self, objects):
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad + 20, 1)
        self.drawObs(objects)

    def drawPath(self, path, raw=True):
        # draw nodes in red that are in the path
        if raw:
            for node in path:
                pygame.draw.circle(self.map, self.red, node, self.nodeRad+3, 0)

        # draw edges in red that are in the path
        for i in range(len(path) - 1):
            pos1, pos2 = path[i], path[i + 1]
            if raw:
                pygame.draw.line(self.map, self.red, pos1, pos2, self.edgeThickness)

            else:
                pygame.draw.line(self.map, self.green, pos1, pos2, self.edgeThickness+1)


    def drawObs(self, objects):
        for obj in objects['obstacles']:
            pygame.draw.rect(self.map, self.black, obj)
        for obj in objects['bumpers']:
            pygame.draw.rect(self.map, self.grey, obj, width=1)

    def animate(self, path, img):
        img = pygame.image.load(img)
        imgh = img.get_height()
        imgw = img.get_width()

        f = 0.5
        img = pygame.transform.scale(img, (imgw*f, imgh*f))

        imgh = img.get_height()
        imgw = img.get_width()

        angle1 = 270

        for i, c in enumerate(path):
            if i == len(path)-1:
                break

            x2, x1 =  path[i+1][0], c[0]
            y2, y1 =  path[i+1][1], c[1]

            xdiff, ydiff = x2-x1, y2-y1

            angle2 = 270 - math.degrees(math.atan(ydiff/xdiff))

            acc = 500
            for i in range(0,acc+1):
                x = i *((x2-x1)/acc) + x1
                y = i *((y2-y1)/acc) + y1
                angle = i *((angle2-angle1)/acc) + angle1

                rotated_img = pygame.transform.rotate(img, angle)

                loc = (x-imgw/2, y-imgh/2)
                new_rect = rotated_img.get_rect(center=img.get_rect(topleft = loc).center)

                self.map.fill(self.white)
                self.drawPath(path, raw=False)
                self.map.blit(rotated_img, new_rect.topleft)
                pygame.display.update()
                pygame.event.clear()

                #time.sleep(0.005)

            angle1 = angle2

class RRTGraph:
    def __init__(self, start, goal, MapDimensions):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.height, self.width = MapDimensions

        # list to store x and y coordinates and the parent of the child node
        self.x = []
        self.y = []
        self.parent = []

        # Initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # Obstacles
        self.obstacles = []

        # Path
        self.goalstate = None
        self.path = []

    # Make obstacles and safety boundaries
    def makeObs(self, rect_arguments):
        b = 50 # set obstacle enlargement

        objects = {}
        obstacles = []
        bumpers = []

        for ob_arg in rect_arguments:
            obstacles.append(pygame.Rect(ob_arg))
            bumpers.append(pygame.Rect(ob_arg[0]-b/2, ob_arg[1]-b/2, ob_arg[2]+b, ob_arg[3]+b))

        objects['obstacles'] = obstacles
        objects['bumpers'] = bumpers

        self.obstacles = bumpers.copy() # use this for checking whether samples crosses an obstacle
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

    # Euclidean distance
    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        return (px+py)**(0.5)

    # Create random samples
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

    # obstacle detection
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

            # When we are close to the goal instead of creating a random node, we use the goal node
            # This stops the weird behavior we had that we go beyond the goal (tell in report)
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
    # This will be used when simulating the drive of the mobile robot
    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        print(pathCoords)
        return pathCoords

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

    def smooth(self, path, weight_data=0.25, weight_smooth=0.6, tolerance=0.000001):
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

        print("smooth:", newpath)
        return new