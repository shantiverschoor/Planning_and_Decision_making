import numpy as np

# State constraints and limitations on inputs
min_steer = -30*np.pi/180 # Minimum steering angle [rad]
max_steer = 30*np.pi/180 # Maximum steering angle [rad]

v_min = 0 # Minimum longitudinal velocity [m/s]
v_max = 3 # Maximum longitudinal velocity [m/s]

L = 0.5 # Wheel base of vehicle [m]

dt = 0.1

class LinearBicycleModel(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0):
        """Instantiate the object."""
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v

    def update(self, throttle, delta):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param a: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, min_steer, max_steer)

        # Equations of motion
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.v / L * np.tan(delta) * dt
        self.theta = normalize_angle(self.theta)
        self.v += throttle * dt

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return
