"""
Closed loop simulation of an inverted pendulum using MPC
Simulation contains the constrains resulting by the used slow stepper motor

Based on https://matplotlib.org/3.2.2/gallery/animation/double_pendulum_sgskip.html

@author: Thomas Schnapka
"""

import cvxpy as cp
import time
from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

# open plot in new window instead of inline
# use %matplotlib qt if plot is not showing up
get_ipython().run_line_magic('matplotlib', 'qt')

# model paramters
g = 9.8  # gravity
l = 1  # length of pendulum
m = 0.1  # mass of pendulum
M = 1  # mass of cart
max_velocity = (400.0/800)*np.pi*12.73 # maximum stepper motor velocity
max_acceleration = np.pi*12.73*40000.0/800 # maximum stepper motor acceleration

# mpc parameters
update_time = 0.05
N = 10                  # prediction time
tt = 0.2                # preditiction step duration (time ticks)

# cost terms
Q = np.diag([1.0, 1.0, 2.0, 0.01])
R = np.diag([1e-3])

# animation parameters
cart_width = 0.2
cart_height = 0.2
dt = 0.05
t = np.arange(0, 5, dt)

# initial state
# [theta, theta_dot, x, x_dot]
state =     np.array([np.deg2rad(0), 0, -1, 0])
state_set = np.array([np.deg2rad(0), 0,  1, 0])

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
    cost += cp.sum_largest((x_opt[:,n+1]-state_set)@Q, 1) 
    cost += cp.sum_squares(u_opt[:,n]@R)
    constr += [x_opt[:,n+1] == A_tt@x_opt[:,n] + B_tt@u_opt[:,n]]
constr += [x_opt[:,N] == state_set]
constr += [x_opt[:,0] == state_act]
constr += [ x_opt[3, :] <= max_velocity]
constr += [-x_opt[3, :] <= max_velocity]
problem = cp.Problem(cp.Minimize(cost), constr)

def get_MPC_output(state):
    '''model predictive control'''
    global state_act
    state_act.value = state
    
    problem.solve(verbose=False)
    
    if problem.status != cp.OPTIMAL:
        u = 0
        print('[Warning] no optimal soultion found')
    else:
        u = u_opt.value[0,0]
    return u


### simulation ###############################################################

last_time = -20
u = 0

def derivs(state, t):
    '''differential equations for state calculation'''
    global last_time, u
    if t - last_time > update_time:
        u = get_MPC_output(state)
        last_time = t
    
        
    # motor constraints for control input
    a = u/M
    a = np.minimum(np.abs(a), max_acceleration)*np.sign(a)
    u = a*M
    
    # state calculation
    state_dot = np.zeros_like(state)
    state_dot[0] = state[1]
    state_dot[1] = ((u*cos(state[0]) - (M+m)*g*sin(state[0]) + m*l*cos(state[0])*sin(state[0])*state[1]**2)
                    /(m*l*cos(state[0])**2 - (M+m)*l))
    state_dot[2] = state[3]
    state_dot[3] = ((u + m*l*sin(state[0])*state[1]**2 - m*g*cos(state[0])*sin(state[0]))
                     /(m + M - m*cos(state[0])**2))
    
    # motor constraints for state
    state_dot[2] = min(abs(state_dot[2]), max_velocity )*np.sign(state_dot[2])
    state_dot[3] = min(abs(state_dot[3]), max_velocity)*np.sign(state_dot[3])

    return state_dot

# integrating ODE
y = integrate.odeint(derivs, state, t)


### animation ################################################################

x_cart = y[:, 2]
y_cart = np.zeros_like(x_cart)

x_pendulum = l*sin(y[:, 0]) + x_cart
y_pendulum = l*cos(y[:, 0])

fig = plt.figure()
ax = fig.add_subplot((111), autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_aspect('equal')
ax.grid()

rect = ax.add_patch(Rectangle((0, 0), 0, 0, facecolor='gray'))

line, = ax.plot([], [], 'o-', lw=2)
course, = ax.plot([], [], '-', color='red', lw=0.5)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
type_text = ax.text(0.45, 1.05, 'MPC', transform=ax.transAxes)


def init():
    line.set_data([], [])
    course.set_data([], [])
    time_text.set_text('')
    rect.set_xy((-cart_width/2, -cart_height/2))
    rect.set_width(cart_width)
    rect.set_height(cart_height)
    return line, time_text, rect, course, type_text


def animate(i):
    thisx = [x_cart[i], x_pendulum[i]]
    thisy = [y_cart[i], y_pendulum[i]]
    line.set_data(thisx, thisy)
    rect.set_x(x_cart[i] - cart_width/2)
    time_text.set_text(time_template % (i*dt))
    course_x, course_y = course.get_data()
    course_x = np.append(course_x, x_pendulum[i])
    course_y = np.append(course_y, y_pendulum[i])
    course.set_data(course_x, course_y)
    return line, time_text, rect, course, type_text


ani = animation.FuncAnimation(fig, animate, range(1, len(y)),
                              interval=dt*1000, blit=True, init_func=init)
plt.show()






