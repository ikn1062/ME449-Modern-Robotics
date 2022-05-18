import numpy as np
import modern_robotics as mr

from numpy import cos
from numpy import sin
from numpy import arccos
from numpy import pi


"""
Initial Parameters:::
l, w, r, Tb0, M0e, B -> taken from Kinematics of the youBot on the wiki page 
F -> taken from eqn 13.33 from book
"""
l, w, r = (0.47/2, 0.3/2, 0.0475)

Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
M0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
B = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0],
              [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]])

F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])


"""
MAIN FUNCTION:::
"""


def mobile_manipulation(Tsc_i, Tsc_g, Tse_i, robot_config, Ki, Kp):
    """
    Capstone Mobile Manipulator function
    :param Tsc_i: the initial resting configuration of the cube object (np array)
    :param Tsc_g: the desired final resting configuration of the cube object (np array)
    :param Tse_i: the reference initial configuration of the youBot (np array)
    :param robot_config: A vector containing 3 chassis configuration, 5 arm configuration, 4 wheel angles (np vector)
    :param Ki: PI Controller (np array)
    :param Kp: Constant Control Gain (np array)
    :return: Csv file which drives the youBot to successfully pick up the block and put it down at the desired location
             A data file containing the 6-vector end-effector error
    """
    print("Running mobile_manipulation function...")

    Tce_g = np.array([[-cos(pi / 4), 0, sin(pi / 4), 0.01], [0, 1, 0, 0],
                      [-sin(pi / 4), 0, -cos(pi / 4), -0.01], [0, 0, 0, 1]])
    Tce_s = np.array([[0, 0, 1, -0.25], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
    print("Creating trajectory...")
    trajectory = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_g, Tce_g, Tce_s)

    # Robot Configuration: [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state]
    robot_config_csv = robot_config
    error_csv = np.array([0, 0, 0, 0, 0, 0])

    integral = 0
    dt = 0.01

    print("Creating Configuration csv array...")
    print("Creating Error csv array...")

    for i in range(len(trajectory)-1):
        arm_config = robot_config[3:8]

        # These steps for creating J_e are taken from pages 199, 551-552 of the book
        J_arm = mr.JacobianBody(B.transpose(), arm_config)
        T_zero = mr.FKinBody(M0e, B.transpose(), arm_config)
        F_6 = np.array([[0, 0, 0, 0], [0, 0, 0, 0], F[0], F[1], F[2], [0, 0, 0, 0]])
        J_body = np.dot(mr.Adjoint(np.dot(np.linalg.inv(T_zero), Tb0)), F_6)
        J_e = np.append(J_body, J_arm, axis=1)

        # Generating current config, current reference config, and reference config at next time step
        Tsb = Tsb_q(robot_config[0], robot_config[1], robot_config[2])
        X = np.dot(np.dot(Tsb, Tb0), T_zero)
        Xd = N_to_T(trajectory[i])
        Xd_next = N_to_T(trajectory[i+1])

        # Obtaining velocities for NextState and Xerr for csv file
        vel, Xerr, integral = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, J_e, integral)

        # Generating next robot configuration using NextState
        vel = np.array([vel[4], vel[5], vel[6], vel[7], vel[8], vel[0], vel[1], vel[2], vel[3]])
        gripper = trajectory[i][-1]
        robot_config = NextState(robot_config, vel, dt=dt, grip=gripper)

        # Adding configuration and error vectors to the csv array
        robot_config_csv = np.vstack((robot_config_csv, robot_config))
        error_csv = np.vstack((error_csv, Xerr))

    # Outputting error and robot configuration as csv files
    print("Outputting Configuration csv file...")
    np.savetxt('robot_config.csv', robot_config_csv, delimiter=',')
    print("Outputting error csv file...")
    np.savetxt('error.csv', error_csv, delimiter=',')


def N_to_T(N_vec):
    return np.array([[N_vec[0], N_vec[1], N_vec[2], N_vec[9]],
                     [N_vec[3], N_vec[4], N_vec[5], N_vec[10]],
                     [N_vec[6], N_vec[7], N_vec[8], N_vec[11]],
                     [0, 0, 0, 1]])


def NextState(current_config, speeds, dt=0.01, max_speed=500, grip=0):
    """
    MILESTONE 1:::
    Nextstate computes new joint/wheel angles and chassis configuration after time step dt
    :param current_config: A vector containing 3 chassis configuration, 5 arm configuration, 4 wheel angles
    :param speeds: A vector containing 5 arm joint speeds and 4 wheel speeds
    :param dt: A timestep
    :param max_speed: A positive real value indicating the maximum angular speed of the arm joints and the wheels
    :param grip: gripper is 0 or 1, representing value of gripper state
    :return: A 12-vector representing the configuration of the robot time Δt later
    """
    # Extracting the various configurations and speeds of the robot:::
    chassis_config = current_config[:3]
    arm_config = current_config[3:8]
    wheel_ang = current_config[8:12]

    # Capping speeds to Max Speed
    for i, speed in enumerate(speeds):
        if np.abs(speed) > max_speed:
            speeds[i] = max_speed
    joint_speeds = speeds[:5]
    wheel_speeds = speeds[5:]

    # Calculating new arm joint angles and new wheel angles:::
    arm_config_1 = arm_config + joint_speeds * dt
    wheel_config_1 = wheel_ang + wheel_speeds * dt

    # New chassis configuration Tsb(k+1) = Tsb(k)e[Vb6]:::
    d_theta = wheel_speeds * dt
    Vb = np.dot(F, d_theta)
    Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])
    e_vb6 = mr.MatrixExp6(mr.VecTose3(Vb6))
    Tsb = Tsb_q(chassis_config[0], chassis_config[1], chassis_config[2])
    # Final config = Initial Config (dot) Change in Config
    Tsb_k_plus_1 = np.dot(Tsb, e_vb6)

    # Creating output for NextState (new configuration)
    config_new = np.array([arccos(Tsb_k_plus_1[0][0]), Tsb_k_plus_1[0][3], Tsb_k_plus_1[1][3]])
    config_new = np.append(config_new, arm_config_1)
    config_new = np.append(config_new, wheel_config_1)
    config_new = np.append(config_new, grip)

    return config_new


def Tsb_q(phi, x, y):
    """
    Helper function for NextState solving for current configuration given phi, x, y
    :param phi: Chassis angle
    :param x: Chassis x
    :param y: Chassis y
    :return: Current Configuration Ts
    """
    Tsb = np.array([[cos(phi), -sin(phi), 0, x],
                   [sin(phi), cos(phi), 0, y],
                   [0, 0, 1, 0.0963],
                   [0, 0, 0, 1]])
    return Tsb


def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s, k=1, Tf=3):
    """
    MILESTONE 2:::
    Generates the reference trajectory for the end-effector frame {e}
    :param Tse_i: The initial configuration of the end-effector in the reference trajectory
    :param Tsc_i: The cube's initial configuration
    :param Tsc_f: The cube's desired final configuration
    :param Tce_g: The end-effector's configuration relative to the cube when it is grasping the cube
    :param Tce_s: The end-effector's standoff configuration above the cube, before/after grasping, relative to the cube
    :param k: The number of trajectory reference configurations per 0.01 seconds
    :param Tf: Time for each trajectory segment
    :return: A representation of the N configurations of the end-effector along the entire concatenated
             eight-segment reference trajectory.
    """
    trajectory = np.zeros(13)

    N = Tf*k/0.01
    method = 5

    # The first step of the trajectory is to move form initial config to initial standoff config
    # Initial standoff Config (Tsi) = dot product(Space -> cube, cube -> initial standoff)
    Tsi = np.dot(Tsc_i, Tce_s)
    initial_to_initial_standoff = mr.CartesianTrajectory(Tse_i, Tsi, Tf, N, method)
    trajectory = add_trajectory(trajectory, initial_to_initial_standoff, 0)
    trajectory = trajectory[1:] # Remove initial trajectory vector

    # The second step of the trajectory is to move from initial standoff config to initial grasp config
    # End Effector Gripper Config at Cube = dot project(space -> cube, cube -> end effector @ cube)
    Tgi = np.dot(Tsc_i, Tce_g)
    initial_standoff_to_grasp = mr.CartesianTrajectory(Tsi, Tgi, Tf, N, method)
    trajectory = add_trajectory(trajectory, initial_standoff_to_grasp, 0)

    grip_conversion = trajectory[-1]
    grip_conversion[-1] = 1
    for i in range(63):
        trajectory = np.vstack((trajectory, grip_conversion))

    # The third step of the trajectory is to move from cubes initial grasp config to initial standoff config
    initial_grasp_to_standoff = mr.CartesianTrajectory(Tgi, Tsi, Tf, N, method)
    trajectory = add_trajectory(trajectory, initial_grasp_to_standoff, 1)

    # The fourth step of the trajectory is to move from initial standoff config to final standoff config
    # End Effector Gripper Config at Cube = dot project(space -> cube, cube -> end effector @ cube)
    Tsf = np.dot(Tsc_f, Tce_s)
    initial_to_final_standoff = mr.CartesianTrajectory(Tsi, Tsf, Tf, N, method)
    trajectory = add_trajectory(trajectory, initial_to_final_standoff, 1)

    # The fifth step of the trajectory is to move from final standoff config to final grasp config
    Tgf = np.dot(Tsc_f, Tce_g)
    final_standoff_to_grasp = mr.CartesianTrajectory(Tsf, Tgf, Tf, N, method)
    trajectory = add_trajectory(trajectory, final_standoff_to_grasp, 1)

    grip_conversion = trajectory[-1]
    grip_conversion[-1] = 0
    for i in range(63):
        trajectory = np.vstack((trajectory, grip_conversion))

    # The sixth step of the trajectory is to move from final standoff config to cubes final config
    final_grasp_to_standoff = mr.CartesianTrajectory(Tgf, Tsf, Tf, N, method)
    trajectory = add_trajectory(trajectory, final_grasp_to_standoff, 0)

    np.savetxt('trajectory.csv', trajectory, delimiter=',')
    return trajectory


def add_trajectory(trajectory_array, trajectory_to_add, gripper):
    """
    Helper function for Trajectory generator
    :param trajectory_array: Trajectory array to be returned for Trajectory generator function
    :param trajectory_to_add: Trajectory that needs to be added to trajectory_array
    :param gripper: 0 or 1 with respect to gripper state
    :return: Trajectory_array with trajectory_to_add added
    """
    for config in trajectory_to_add:
        trajectory = np.zeros(13)
        i = 0
        for row in range(3):
            for col in range(3):
                trajectory[i] = config[row][col]
                i += 1
        for row in range(3):
            trajectory[i] = config[row][3]
            i += 1
        trajectory[i] = gripper
        trajectory_array = np.vstack((trajectory_array, trajectory))
    return trajectory_array


def FeedbackControl(X, Xd, Xd_n, Kp, Ki, dt, Je, integral):
    """
     Calculates the kinematic task-space feedforward plus feedback control law:::
     V(t) = [Ad(X^-1Xd)]Vd(t) + KpXerr(t) + Ki integral(0->t)(Xerr(t))dt
    :param X: The current actual end-effector configuration X
    :param Xd: The current end-effector reference configuration Xd
    :param Xd_n: The end-effector reference configuration at the next timestep in the reference trajectory, Xd,next
    :param Kp: The PI gain matrices Kp
    :param Ki: The PI gain matrices Ki
    :param dt: The timestep Δt between reference trajectory configurations
    :param Je: Mobile Manipulator Jacobian
    :param integral: Returns integral
    :return: The commanded end-effector twist V expressed in the end-effector frame {e}
    """

    # First calculate [Ad(X^-1Xd)] then calculate Vd(t) and find the dot product of terms
    x_minus1_xd = np.dot(np.linalg.inv(X), Xd)
    adjoint_x_xd = mr.Adjoint(x_minus1_xd)
    xd_minus1_xd_n = np.dot(np.linalg.inv(Xd), Xd_n)
    Vd_t = mr.se3ToVec(1/dt * mr.MatrixLog6(xd_minus1_xd_n))

    # Second calculate KpXerr(t)
    Xerr = mr.se3ToVec(mr.MatrixLog6(x_minus1_xd))

    # Third calculate Ki integral(0->t)(Xerr(t))dt
    integral = integral + Xerr * dt

    V_t = np.dot(adjoint_x_xd, Vd_t) + np.dot(Kp, Xerr) + np.dot(Ki, integral)
    vel = np.dot(np.linalg.pinv(Je, 1e-4), V_t)

    return vel, Xerr, integral


"""
CALLING FUNCTION:::
"""
Tsc_i = np.array([[cos(pi/3), -sin(pi/3), 0, 0.5], [sin(pi/3), cos(pi/3), 0, 0.5], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_g = np.array([[cos(-pi), -sin(-pi), 0, 1], [sin(-pi), cos(-pi), 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tse_i = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

robot_config = np.array([-pi/6, 0, 0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])
Ki = 0.1 * np.identity(6)
Kp = 2 * np.identity(6)

print("Result: NEWTASK\n")
mobile_manipulation(Tsc_i, Tsc_g, Tse_i, robot_config, Ki, Kp)
