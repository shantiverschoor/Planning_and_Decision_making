height, width = 20, 1600

# no obstacles
obs1= [] 

# square obstacles
obs2 = [(0, 100, 200, 20),
(300, 300, 50, 30),
(400, 0, 20, 460),
(0, 450, 150, 20),
(150, 350, 50, 200),
(500, 100, 50, 30),
(500, 400, 50, 30),
(660, 300, 50, 30),
(800, 200, 20, 400)]
# Determine shortest path
#         (50, 50, 200, 1),
#         (250, 50, 1, 440),
#         (250, 490, 500, 1),
#         (750, 170, 1, 320),
#         (750, 170, 150, 1),
#         (900, 170, 1, 380)

# Maze map
obs3 = [(125, 0, 20, 225),
(125, 350, 20, 100),
(125, 450, 500, 20),
(300, 450, 20, 150),
(125, 350 - 20, 200, 20),
(325, 125, 20, 350 - 125),
(475, 0, 20, 320),
(625, 250 + 20, 20, 200),
(625, 250, 150, 20),
(775, 150, 20, 100 + 20),
(625, 150, 150, 20),
(775, 400, 225, 20)]
# Determine shortest path
#       (50, 50, 1, 205),
#       (50, 255, 200, 1),
#       (250, 95, 1, 160),
#       (250, 95, 150, 1),
#       (400, 95, 1, 255),
#       (400, 350, 150, 1),
#       (550, 120, 1, 230),
#       (550, 120, 275, 1),
#       (825, 120, 1, 240),
#       (745, 360, 80, 1),
#       (745, 360, 1, 190),
#       (745, 550, 155, 1)

# trapped in corner
obs4 = [(125, 0, 20, 225),
        (20, 225, 200, 20)]