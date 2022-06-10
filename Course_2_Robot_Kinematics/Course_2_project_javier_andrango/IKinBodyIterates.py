from modern_robotics import *
import numpy as np


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics (joint angle list) in the body frame for
    an open chain robot.

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances, as a report for each iteration for the
                       initial guest to the final solution
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.

    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 100 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        Report for each iteration of the Newton-Raphson process
        (np.array([[....                           ],
                   [1.57073819, 2.999667, 3.14153913]))
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 100
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

    # Edited code from the original function-------------------------------
    # Newton-Raphson process,for iteration 0 (the initial guess)
    print("iteration ", i)
    print("joint vector(theta_i):", thetalist, sep='\n', end='\n\n')
    print("SE(3) end−effector config(Tsb(theta_i)):", \
          FKinBody(M, Blist, thetalist), sep='\n', end='\n\n')
    print("error twist V_b:", Vb, sep='\n', end='\n\n')
    print("angular error magnitude ∣∣omega_b∣∣:", \
          np.linalg.norm([Vb[0], Vb[1], Vb[2]]), end='\n\n')
    print("linear error magnitude ∣∣v_b∣∣:", \
          np.linalg.norm([Vb[3], Vb[4], Vb[5]]), end='\n\n')
    print("---------------------------------------------------")

    # Joint vector matrix, for iteration 0
    iterate_joints = np.array([thetalist])
    # -------------------------------------------------------------------

    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

        # Edited code from the original function-------------------------------
        # Newton-Raphson process,for iteration 1 to the final solution

        print("iteration ", i)
        print("joint vector(theta_i):", thetalist, sep='\n', end='\n\n')
        print("SE(3) end−effector config(Tsb(theta_i)):", \
              FKinBody(M, Blist, thetalist), sep='\n', end='\n\n')
        print("error twist V_b:", Vb, sep='\n', end='\n\n')
        print("angular error magnitude ∣∣omega_b∣∣:", \
              np.linalg.norm([Vb[0], Vb[1], Vb[2]]), end='\n\n')
        print("linear error magnitude ∣∣v_b∣∣:", \
              np.linalg.norm([Vb[3], Vb[4], Vb[5]]), end='\n\n')
        print("---------------------------------------------------")

        # Joint vector matrix, for iteration 1 to the final solution
        iterate_joints = np.append(iterate_joints, [thetalist], 0)
        # -------------------------------------------------------------------

    # Create a csv file from joint vector matrix
    # print("joint vector matrix:")
    print(iterate_joints, end='\n\n')
    np.savetxt('csv/iterates.csv', iterate_joints, delimiter=',', fmt='%f')
    return (thetalist, not err)
