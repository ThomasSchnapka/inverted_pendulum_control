#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPC controller for a "real" inverted pendulum built using old 3D-printer parts.
To be used connected to an Arduino running the "ip_mpc_arduino.ino" script

@author: Thomas Schnapka
"""

import cvxpy as cp
import time
import numpy as np
import serial


# model paramters
g = 9.8  # gravity
l = 1  # length of pendulum
m = 0.1  # mass of pendulum
M = 1  # mass of cart
max_velocity = (400.0/800)*np.pi*12.73 # maximum stepper motor velocity

# mpc parameters
update_time = 0.05
N = 10                  # prediction time
tt = 0.1                # preditiction step duration (time ticks)
u = 0

# cost terms
Q = np.diag([1.0, 1.0, 10.0, 1.0])
R = np.diag([1e-4])

# initial state
# [theta, theta_dot, x, x_dot]
state =     np.array([np.deg2rad(0), 0, 0, 0])
state_set = np.array([np.deg2rad(0), 0, 0, 0])

np.set_printoptions(precision=3, suppress=True)

### state space matrices ######################################################

A = np.matrix([[0,               1, 0, 0],
               [((M+m)*g)/(M*l), 0, 0, 0],
               [0,               0, 0, 1],
               [-m*g/M,          0, 0, 0]])


B = np.matrix([0, -1/(M*l), 0, 1/M]).T


### optimization #############################################################

# optimization variables
x_opt = cp.Variable((4, N+1))
u_opt = cp.Variable((1, N))
state_act = cp.Parameter(4)
# state space matrices involving the time horizon
A_tt = np.eye(4) + tt * A
B_tt = tt * B

cost = 0
constr = []

for n in range(N):
    cost += cp.sum_squares((x_opt[:,n+1]-state_set)@Q) 
    cost += cp.sum_squares(u_opt[:,n]@R)
    constr += [x_opt[:,n+1] == A_tt@x_opt[:,n] + B_tt@u_opt[:,n]]
constr += [x_opt[:,N] == state_set]
constr += [x_opt[:,0] == state_act]
constr += [ x_opt[3, :] <= max_velocity]
constr += [-x_opt[3, :] <= max_velocity]
problem = cp.Problem(cp.Minimize(cost), constr)

def get_MPC_output(state):
    '''model predictive control'''
    global state_act, u
    state_act.value = state
    
    problem.solve(verbose=False)
    
    if problem.status != cp.OPTIMAL:
        u = 0
        print('[Warning] no optimal soultion found')
    else:
        u = u_opt.value[0,0]
    return u

### communication #############################################################

arduino = serial.Serial(port='/dev/cu.wchusbserialfd120', #'/dev/cu.wchusbserialfa130'
                        baudrate=115200, timeout=.1)
time.sleep(1) # mandatory! Arduino needs some time to establish serial connection

def write_read_plant(u):
    '''exchange data with plant/Arduino'''
    arduino.write(bytes(np.format_float_positional(u, 2), 'utf-8'))
    #arduino.write("1".encode())
    #time.sleep(0.05)
    data = arduino.readline()
    data = data.decode("utf-8")
    state = np.fromstring(data, count=4, sep=" ")
    return state


### loooooooooop #############################################################

# wait until pendulum is turned upwards
while True:
    time.sleep(update_time)
    state = write_read_plant(u)
    if abs(state[0]) < 0.02:
        break

print("start")

while True:
    time.sleep(update_time)
    u = get_MPC_output(state)
    #print(state, u)
    state = write_read_plant(u)
    
arduino.close()






