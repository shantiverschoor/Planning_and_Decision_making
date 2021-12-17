h, w = 600, 1000
wall = 10

# Map 1 driving through corners
sim1 = {(0,150): (400,wall),
	   (550+wall,0): (wall,h-150),
	   (400,150): (wall,h-150),
	   (550+wall,h-150): (w-150, wall)}

# Map 2 with some static obstacles
sim2 = {(0,100): (200,wall),
		(300,300): (50,30),
		(400,0): (wall,460),
		(0,450): (150,wall),
		(150,350): (10,200),
		(500,100): (50,30),
		(500,400): (50,30),
		(660,300): (50,30),
		(800, 200): (wall, 400)}



# Map 3 driving through a maze
sim3 = {(125, 0): (wall, 225),
		(125, 350): (wall, 100),
		(125, 450): (500, wall),
		(300, 450): (10, 150),
		(125, 350-wall): (200, wall),
		(325,125): (wall, 350-125),
		(475, 0): (wall, 320),
		(625, 250+wall): (wall, 200),
		(625, 250): (150, wall),
		(775, 150): (wall, 100+wall),
		(625, 150): (150, wall),
		(775, 400): (225, wall)}