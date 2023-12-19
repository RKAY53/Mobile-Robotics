import numpy as np
from simple_pid.pid import PID # >>pip3 install simple-pid
import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation # >>pip3 install scipy
import sys
from math import cos, sin, pi, sqrt, atan2
# Increase the recursion limit (use with caution)

# PID further info
# https://github.com/m-lundberg/simple-pid

def pi_clip(angle):
    '''Function to map angle error values between [-pi, pi)'''
    while angle >= math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def transformations(Rab, Rbc, Tab, Tbc):
    Rac = Rab.dot(Rbc)

    Tac = Rab.dot(Tbc) + Tab

    quat_ac = Rotation.from_matrix(Rac).as_quat()

    euler_ac = Rotation.from_matrix(Rac).as_euler('xyz')

    return Rac, quat_ac, euler_ac, Tac

# Example inputs
Rab = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
Rbc = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
Tab = np.array([1, 2, 3])
Tbc = np.array([4, 5, 6])

Rac, quat, euler, Tac = transformations(Rab, Rbc, Tab, Tbc)

print("Rac:")
print(Rac)
print("Quaternion:")
print(quat)
print("Euler angles:")
print(euler)
print("Tac:")
print(Tac)


class problem_set2:

    pid_w = PID(-2.0, -0.0, -0.0, setpoint=0.0, output_limits=(-5, 5))
    Kp_v = -2
    Ki_v = -0.0001
    Kd_v = -0.5
    pid_v = PID(Kp_v, Ki_v, Kd_v, setpoint=0.0, output_limits=(0, 2))

    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.xd = 0.0
        self.yd = 0.0
        self.time = np.arange(0, 40, 0.1)
        self.dt = 0.1

        self.v = 0
        self.w = 0
        self.theta = 0
        self.results = [[], [], [], [], [], [], []]

        for t in self.time:
            self.desired_trajectory(t)
            self.update_robot_state()
            angle_error, distance_error = self.compute_error()
            self.w, self.v = self.compute_control(angle_error, distance_error)
            self.save_results()

    def desired_trajectory(self, t):
        # Create a circular trajectory with a radius of 10 and a frequency of 0.03 Hz
        self.xd = 10 * cos(2 * pi * 0.03 * t)
        self.yd = 10 * sin(2 * pi * 0.03 * t)

    def update_robot_state(self):
        self.x += self.v * cos(self.theta) * self.dt
        self.y += self.v * sin(self.theta) * self.dt
        self.theta += self.w * self.dt

    def compute_error(self):
        delta_x = self.xd - self.x
        delta_y = self.yd - self.y
        distance_error = sqrt(delta_x ** 2 + delta_y ** 2)
        angle_error = -self.theta + atan2(delta_y, delta_x)
        return angle_error, distance_error

    def compute_control(self, angle_error, distance_error):

        control_w = self.pid_w(angle_error, dt=self.dt)
        control_v = self.pid_v(distance_error, dt=self.dt)

        return control_w, control_v


    def save_results(self):
        self.results[0].append(self.x)
        self.results[1].append(self.y)
        self.results[2].append(self.xd)
        self.results[3].append(self.yd)
        self.results[4].append(self.theta)
        self.results[5].append(self.w)
        self.results[6].append(self.v)

if __name__ == '__main':
    problem = problem_set2()
    plt.figure(1)
    plt.plot(problem.results[0], problem.results[1], 'b')
    plt.plot(problem.results[2], problem.results[3], 'r')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend(["Actual trajectory", "Desired trajectory"])

    plt.figure(2)
    plt.plot(problem.results[5], 'b')
    plt.plot(problem.results[6], 'r')
    plt.legend(["Angular velocity w", "Forward velocity v"])
    plt.show()