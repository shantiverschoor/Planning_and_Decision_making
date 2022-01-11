import pygame
from RRT_max import RRTGraph
from RRT_max import RRTMap
import time
from obstacles_max import *
from car_kinematic import *

def main():

    # Map info
    height = 600
    width = 1000
    dimensions = (height,width)

    start = (50, 50)
    # start = (250, 550) # start position 2 of obs3
    goal = (900, 550)

    iteration = 0
    t1 = 0

    sampling_bias = 1000000 # Tune parameter on how much sampling you want towards the goal
    sampling_update = 1 # Update map

    # Setup simulation
    pygame.init()
    map = RRTMap(start, goal, dimensions)
    graph = RRTGraph(start, goal, dimensions)

    # setup obstacles in the map
    obstacles = graph.makeObs(obs3)
    map.drawMap(obstacles)

    # time.sleep(1)
    t1 = time.time()
    t_start = time.time()
    while (not graph.path_to_goal()): # Stop simulation when the path to the goal is found
        elapsed = time.time() - t1
        t1 = time.time()

        if elapsed > 10: # prevent iterating while the goal is found
            raise

        # A bias is used for expanding the tree such that 100%/Bias will be based on expanding towards the goal and
        # an 100% - 100%/Bias will be based on random expansions

        if iteration  == -1: # heuristic sampling
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

    trials = [2.788, 3.160, 4.0219, 3.676, 3.620, 2.966, 3.007, 2.405, 2.396, 3.757, 2.525, 2.890, 1.905, 2.578, 2.776]
    trials_obs3 = np.array([4.321, 4.565, 2.078, 3.355, 2.088, 2.883, 4.084, 3.757, 2.970, 3.905, 2.464, 3.817, 2.411, 3.534, 3.248])

    map.drawPath(graph.getPathCoords()) # red
    smooth_path = graph.smooth(graph.getPathCoords())
    map.drawPath(smooth_path, raw=False) # green
    pygame.display.update()
    time.sleep(1)

    confgs = map.configuration(smooth_path)
    inputs = BW_kinematics(confgs)
    # print(inputs)
    states = FW_kinematics(inputs)
    # print(states)

    # Animation of the car driving the computed path from start to goal
    print('[info] Start animation')
    # map.animate(obstacles, smooth_path[::-1], 'car.png')

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