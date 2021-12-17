import pygame
from scipy import interpolate
import numpy as np

def B_spline(waypoints):
    x = []
    y = []

    for point in waypoints:
        x.append(point[0])
        y.append(point[1])

    tck, *rest = interpolate.splprep([x,y])
    u = np.linspace(0, 1, num=1000)
    smooth = interpolate.splev(u, tck)

    return smooth

smooth = []
ControlPoints = []

pygame.init()
pygame.display.set_caption("RRT path planning")
map = pygame.display.set_mode((800, 512))
map.fill((255, 255, 255))
running = True
while(running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            pygame.draw.circle(map,(0,0,0), pos, 7, 0)
            ControlPoints.append(pos)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                smooth = B_spline(ControlPoints)
                X_smooth, Y_smooth = smooth
                map.fill((255, 255, 255))
                for x, y in zip(X_smooth, Y_smooth):
                    pygame.draw.circle(map, (255, 0, 0), (x,y), 2, 0)

                for point in ControlPoints:
                    pygame.draw.circle(map, (0, 0, 0), point, 7, 0)

    pygame.display.update()