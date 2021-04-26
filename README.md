# Optimal Inverted Pendulum Control
### Optimal control of an inverted pendulum built using old 3D printer parts

I created the test setup and the code to try out the things I learned in [these](http://underactuated.mit.edu/) notes about Underactuated Robotics
and to gather knowledge to be used in my [Quadruped Robot](https://github.com/ThomasSchnapka/quaro).

Two different optimal control approaches are implemented: Least Squares Regulator (MPC) and Model Predictive Control (MPC). For each kind of controller, there exists
a simulation (results can be seen below). Furthermore, there exists a hardware setup built using old 3D-printer parts. This test rig is connected to an Arduino Mega which
controls a stepper motor moving the pendulum communicates with the encoder. In the case of the LQR approach, the Arduino himself is the controller. 
Because the Arduino has not enough computing power to be used as an MPC, it is connected via serial connection to a Python script running on a laptop.

![lqr image](https://github.com/ThomasSchnapka/inverted_pendulum_control/blob/main/lqr.gif?raw=true)
![mpc image](https://github.com/ThomasSchnapka/inverted_pendulum_control/blob/main/mpc.gif?raw=true)

### main sources
1. https://matplotlib.org/3.2.2/gallery/animation/double_pendulum_sgskip.html
1. http://www.taumuon.co.uk/2016/02/lqr-control-of-inverted-pendulum-in-c-using-eigen.html
1. http://underactuated.mit.edu/
