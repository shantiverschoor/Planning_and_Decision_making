from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import math
import random

# State constraints and limitations on inputs
delta_min = -30*np.pi/180 # Minimum steering angle [rad]
delta_max = 30*np.pi/180 # Maximum steering angle [rad]
delta_dot_min = -0.4 # Minimum steering angular velocity [rad/s]
delta_dot_max = 0.4 # Maximum steering angular velocity [rad/s]

v_min = 0 # Minimum longitudinal velocity [m/s]
v_max = 15/3.6 # Maximum longitudinal velocity [m/s]
a_max = 2 # Maximum longitudinal acceleration [m/s^2]

'''
    The implementation of the kinematic bicycle model in the RRT path planning algorithm:
    
    State represented by Xn = [x, y, theta, v, delta], where: 
        x, y are the coordinates located at the center of the rear axle [m]
        theta is orientation of the car [rad]
        v is the longitudinal speed [m/s]
        delta is the steer angle [rad]
        
    Inputs: Xn - current state; a list
            u - input specified as [v_delta, a]
    Output: Xn_dot - derivative of Xn specified as [x_dot, y_dot, theta_dot, a, delta_dot]
'''

def car_model(Xn, u): # Use this for simulation
    # Unpack state variables
    x, y, theta, v, delta = Xn
    x = float(x)
    y = float(y)
    theta = float(theta)
    delta = float(delta)
    v = float(v)

    # Unpack input variables
    v_delta, a = u

    L = 0.5 # wheelbase [m]

    # Implement the state constraints on the steering angle and longitudinal speed
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
    v_dot = a
    delta_dot = v_delta

    B = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [(1/L)*np.tan(delta), 0], [0, 1]])
    B =
    signal.StateSpace()


    # Derivative of state Xn
    Xn_dot = np.array([x_dot, y_dot, theta_dot, v_dot, delta_dot])

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
