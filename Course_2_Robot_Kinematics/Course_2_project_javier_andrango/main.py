from modern_robotics import *
import numpy as np
import IKinBodyIterates as ik

"""Exercise description:
    Test your new function for the UR5 robot of example 4.5 of chapter
    4.1.2(Figure 4.6). The home configuration of the end-effector M is
    given in the book, as well as the numerical values of the constants
    L1,L2,H1,H2,W1,W2.
    The desired end-effectorconfiguration is:
    T = np.array([[0, 1, 0, -0.5],
              [0, 0, -1, 0.1],
              [-1, 0, 0, 0.1],
              [0, 0, 0, 1]])
    Choose an initial guess (thetha0), so that the numerical inverse
    kinematics converges after 3-5 steps.
"""


# Data description
Blist = np.array([[0, 1, 0, 0.191, 0, 0.817],
                  [0, 0, 1, 0.095, -0.817, 0],
                  [0, 0, 1, 0.095, -0.392, 0],
                  [0, 0, 1, 0.095, 0, 0],
                  [0, -1, 0, -0.082, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T

M = np.array([[-1, 0, 0, 0.817],
              [0, 0, 1, 0.191],
              [0, 1, 0, -0.006],
              [0, 0, 0, 1]])

T = np.array([[0, 1, 0, -0.5],
              [0, 0, -1, 0.1],
              [-1, 0, 0, 0.1],
              [0, 0, 0, 1]])

thetalist0 = np.array([2.5, 0.7, 4.1, -2.3, 0.46, -3.6])
eomg = 0.001
ev = 0.0001

[thetalist, success] = ik.IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
print("thetalist:", thetalist)
print("success:", success)
