from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import math
import random

# State constraints and limitations on inputs
delta_min = -30*np.pi/180 # Minimum steering angle [rad]
delta_max = 30*np.pi/180 # Maximum steering angle [rad]

v_min = 0 # Minimum longitudinal velocity [m/s]
v_max = 3 # Maximum longitudinal velocity [m/s]

'''
    The implementation of the kinematic bicycle model in the RRT path planning algorithm:

    State represented by Xn = [x, y, theta], where: 
        x, y are the coordinates located at the center of the rear axle [m]
        theta is orientation of the car [rad]
        
    Inputs: Xn - current state; a list
            u - input specified as [v_delta, a]
    Output: Xn_dot - derivative of Xn specified as [x_dot, y_dot, theta_dot]
'''

def car_model(Xn, u):
    # Unpack state variables
    x, y, theta = Xn
    x = float(x)
    y = float(y)
    theta = float(theta)

    # Unpack input variables
    v, delta = u
    delta = float(delta)
    v = float(v)

    L = 0.5 # wheelbase [m]

    # Implement the state constraints on the steering angle (delta) and longitudinal speed (v)
    if delta > delta_max:
        delta = delta_max
    elif delta < delta_min:
        delta = delta_min

    if v > v_max:
        v = v_max
    elif v < v_min:
        v = v_min

    # Equations of motion
    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = (v / L) * np.tan(delta)

    # Derivative of state Xn
    Xn_dot = np.array([x_dot, y_dot, theta_dot])

    return Xn_dot

'''
    new_state: This function uses Fourth-Order Runge-Kutta (RK4) method for calculating an approximation of the next state Xn+1
    Inputs: Xn - current state; a list
            u - input of the model 
    Output: Xnew - the next state Xn+1 with respect to the current state Xn
'''

def new_state(Xn, u):
    # RK4 method
    k1 = car_model(Xn, u)
    k2 = car_model(Xn + k1 / 2, u)
    k3 = car_model(Xn + k2 / 2, u)
    k4 = car_model(Xn + k3, u)

    delta_t = 0.2 # Step size

    Xnew = Xn + (1/6) * (k1 + 2 * k2 + 2 * k3 + k4) * delta_t
    Xnew = [float(Xnew[0]), float(Xnew[1]), float(Xnew[2]), float(Xnew[3]), float(Xnew[4])]
    return Xnew