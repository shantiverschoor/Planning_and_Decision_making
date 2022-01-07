import pygame
from RRT_max import RRTGraph
from RRT_max import RRTMap
import time
from obstacles_max import *

def main():

    # Map info
    dimensions = (600, 1000)
    start = (50, 50)
    # start = (250, 550) # start position 2 of obs3
    goal = (900, 550)

    iteration = 0
    t1 = 0

    sampling_bias = 10 # Tune parameter on how much sampling you want towards the goal
    sampling_update = 1 # Update map

    # Setup simulation
    pygame.init()
    map = RRTMap(start, goal, dimensions)
    graph = RRTGraph(start, goal, dimensions)

    # setup obstacles in the map
    obstacles = graph.makeObs(obs2)
    map.drawMap(obstacles)

    t1 = time.time()
    while (not graph.path_to_goal()): # Stop simulation when the path to the goal is found
        elapsed = time.time() - t1
        t1 = time.time()

        if elapsed > 10: # prevent iterating while the goal is found
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

    t2 = time.time()

    # Draw raw path and smooth path
    map.drawPath(graph.getPathCoords()) # red
    smooth_path = graph.smooth(graph.getPathCoords())
    map.drawPath(smooth_path, raw=False) # green

    # Animation of the car driving the computed path from start to goal
    print('[info] Start animation')
    # map.animate(smooth_path[::-1], 'car.png')

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

