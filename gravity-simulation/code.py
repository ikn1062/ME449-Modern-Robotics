import modern_robotics as mr
import numpy as np


def integrate(theta_list, time):
    # This function takes in the joint angles and total time for sim 1 and sim 2
    tlist = theta_list
    T = time

    # The following lines include the provided kinematic and inertial parameters of the UR5
    M01 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]])
    M12 = np.array([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]])
    M23 = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]])
    M34 = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
    M45 = np.array([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]])
    M56 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]])
    M67 = np.array([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]])
    G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])

    Glist = np.array([G1, G2, G3, G4, G5, G6])
    Mlist = np.array([M01, M12, M23, M34, M45, M56, M67])
    Slist = np.array([[0, 0, 0, 0, 0, 0],
                      [0, 1, 1, 1, 0, 1],
                      [1, 0, 0, 0, -1, 0],
                      [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
                      [0, 0, 0, 0, 0.81725, 0],
                      [0, 0, 0.425, 0.81725, 0, 0.81725]])

    # The following lines define the input parameters for ForwardDynamics function
    dtlist = np.array([0, 0, 0, 0, 0, 0])
    taulist = np.array([0, 0, 0, 0, 0, 0])
    g = np.array([0, 0, -9.81])
    ftip = np.array([0, 0, 0, 0, 0, 0])

    # This initialises a matrix of joint angles
    joint_angles = np.array(tlist).copy()

    # The next few lines define the time step, dt and time count variable, t
    dt = 0.01
    t = 0
    # The next few lines include the while loop for calculating joint angles, vel, and accel at each time step
    while t < T:
        ddtlist = mr.ForwardDynamics(tlist, dtlist, taulist, g, ftip, Mlist, Glist, Slist)
        tlist, dtlist = mr.EulerStep(tlist, dtlist, ddtlist, dt)
        t = t + dt
        joint_angles = np.vstack((joint_angles, tlist))
    np.savetxt('joint_angles_sim2.csv', joint_angles, delimiter=',')
    return


# These lines are the input parameters for sim 1 and sim 2
T_1 = 3
tlist_1 = np.array([0, 0, 0, 0, 0, 0])
T_2 = 5
tlist_2 = np.array([0, -1, 0, 0, 0, 0])
integrate(tlist_2, T_2)
