"""
Open loop simulation of an inverted pendulum

Based on https://matplotlib.org/3.2.2/gallery/animation/double_pendulum_sgskip.html
http://www.taumuon.co.uk/2016/02/lqr-control-of-inverted-pendulum-in-c-using-eigen.html
"""

from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
from IPython import get_ipython
from matplotlib.patches import Rectangle


# open plot in new window instead of inline
# use %matplotlib qt if plot is not showing up
get_ipython().run_line_magic('matplotlib', 'qt')

g = 9.8  # gravity
l = 1.0  # length of pendulum
m = 1.0  # mass of pendulum
M = 1.0  # mass of cart
cart_width = 0.5
cart_height = 0.5

# create a time array from 0..100 sampled at 0.05 second steps
dt = 0.05
t = np.arange(0, 20, dt)

# initial values
theta = 0.23
theta_dot = 0.0
x = 0.0
x_dot = 0.0

# initial state
state = np.array([theta, theta_dot, x, x_dot])

def derivs(state, t):
    '''differential equations for state calculation'''
    dydx = np.zeros_like(state)
    dydx[0] = state[1]
    dydx[1] = (((M+m)*g*sin(state[0])/cos(state[0])
               - m*l*state[1]**2*sin(state[0]))
               /((l*(m+M)/cos(state[0])) - m*l*cos(state[0])))
    dydx[2] = state[3]
    dydx[3] = ((l*dydx[1] - g*sin(state[0]))/cos(state[0]))

    return dydx

# integrate your ODE using scipy.integrate.
y = integrate.odeint(derivs, state, t)

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


def init():
    line.set_data([], [])
    course.set_data([], [])
    time_text.set_text('')
    rect.set_xy((-cart_width/2, -cart_height/2))
    rect.set_width(cart_width)
    rect.set_height(cart_height)
    return line, time_text, rect, course


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
    return line, time_text, rect, course


ani = animation.FuncAnimation(fig, animate, range(1, len(y)),
                              interval=dt*1000, blit=True, init_func=init)
plt.show()
