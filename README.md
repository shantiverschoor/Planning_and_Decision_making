# Planning_and_Decision_making
This is the repository of group 8 for the subject RO47005 Planning and Decision making 

# The code 
This is the repository of group 8 for the subject RO47005 Planning and Decision making.
For running the RRT algorithm please download this repository and run main.py:
## main.py
For running the RRT algorithm run main.py. Here it is also possible to change the start and goal position, the bias and the length and width of the map. 
## RRT.py
The class RRTmap containts the definitions to draw the map, obstacles and path then de animate function will start the animation with the car model
defined in configuration. 
The RRT will find a path with the class RRTGraphThis class contains all the functions to implement the RRT algorithm.  To optimize the RRT algorithm the following custom functions bias, smooth and step have been added to the class.
## obstacles.py
This is a python script that defines the shape of the obstacles that have been sorted by two different maps. 
## car_kinematic.py
This file contains the kinematic bicycle model of the car. 
## car_model.py
This is the start of implementing a controller to navigate the RRT path as a the kinematic bicycle model.