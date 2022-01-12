import pygame
from RRT import RRTGraph
from RRT import RRTMap
import time
from obstacles import *
from car_kinematic import *

def main():

    # Set map dimensions
    height = 600
    width = 1000
    dimensions = (height,width)

    # Set start and goal
    start = (50, 50)
    goal = (900, 550)

    iteration = 0
    t1 = 0

    sampling_bias = 10 # Tune parameter on how much sampling you want towards the goal
    sampling_update = 1 # Update map

    # Setup simulation
    pygame.init()
    map = RRTMap(start, goal, dimensions)
    graph = RRTGraph(start, goal, dimensions)

    # Setup obstacles in the map
    obstacles = graph.makeObs(obs3)
    map.drawMap(obstacles)

    t1 = time.time()
    t_start = time.time()
    while (not graph.path_to_goal()): # Stop simulation when the path to the goal is found
        elapsed = time.time() - t1
        t1 = time.time()

        if elapsed > 10: # Prevent iterating while the goal is found
            raise

        # A bias is used for expanding the tree such that 100%/Bias will be based on expanding towards the goal and
        # an 100% - 100%/Bias will be based on random expansions

        if iteration % sampling_bias == 0: # heuristic sampling
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)
        else:
            X, Y, Parent = graph.expand() # random sampling
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        if iteration % sampling_update == 0:
            pygame.display.update()
        iteration += 1
    t_end = time.time()
    print((t_end - t_start))

    pathCoords = graph.getPathCoords()
    map.drawPath(pathCoords) # Draw the raw path in red
    pathLen = graph.pathLength(pathCoords)
    smooth_path = graph.smooth(graph.getPathCoords())
    map.drawPath(smooth_path, raw=False) # Draw the smooth path in green
    pygame.display.update()
    time.sleep(1)

    confgs = map.configuration(smooth_path)
    inputs = BW_kinematics(confgs)
    states = FW_kinematics(inputs)

    # Animation of the car driving the computed path from start to goal
    map.animate(obstacles, smooth_path[::-1], 'car.png')

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == "__main__":
    result = False
    while not result: # Keep trying until the path to goal is found
        try:
            main()
            result = True
        except Exception as e:
            print(e)
            result = False