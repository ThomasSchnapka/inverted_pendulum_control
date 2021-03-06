"""
Closed loop simulation of an inverted pendulum using LQR
Simulation contains the constrains resulting by the used slow stepper motor

Based on https://matplotlib.org/3.2.2/gallery/animation/double_pendulum_sgskip.html
http://www.taumuon.co.uk/2016/02/lqr-control-of-inverted-pendulum-in-c-using-eigen.html

@author: Thomas Schnapka
"""

from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
from IPython import get_ipython
from matplotlib.patches import Rectangle
import scipy.linalg


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

# cost terms
Q = np.diag([1.0, 1.0, 2.0, 1.0])
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
# (linearized at theta = 0)


A = np.matrix([[0,               1, 0, 0],
               [((M+m)*g)/(M*l), 0, 0, 0],
               [0,               0, 0, 1],
               [-m*g/M,          0, 0, 0]])


B = np.matrix([0, -1/(M*l), 0, 1/M]).T

# check if system is controllable
C = np.hstack((B, A@B, A**2@B, A**3@B))
if np.linalg.matrix_rank(C) < 4:
    raise Exception("system not controllable")


### calculation of process values #############################################

def lqr(A,B,Q,R):
    """
    from http://www.mwm.im/lqr-controllers-with-python/
    Solve the continuous time lqr controller.
     
    dx/dt = A x + B u
     
    cost = integral x.T*Q*x + u.T*R*u
    """
    #ref Bertsekas, p.151
     
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
     
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
     
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    return K, X, eigVals

K, X, V = lqr(A, B, Q, R)


### simulation ###############################################################

def derivs(state, t):
    '''differential equations for state calculation'''
    u = -K@(state - state_set)
    # motor constraints
    a = u/M
    a = np.minimum(np.abs(a), max_acceleration)*np.sign(a)
    u = a*M
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
type_text = ax.text(0.45, 1.05, 'LQR', transform=ax.transAxes)


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
print("K:", K)
