import numpy as np

from scipy import optimize
from numpy import cos
from numpy import sin
from numpy import pi
from numpy import arctan


"""
FUNCTION:::
"""


def assembly_stability(bodies, contacts, g=9.81):
    """
        :param bodies: A numpy array containing all bodies as [body, x, y, total mass]
        :param contacts: A description of contacts (body 1, body 2, x, y, contact normal (rad), friction coeff)
        :param g: Gravity
        :return: String if solution is valid or invalid
    """
    F = np.zeros((3*len(bodies), 2*len(contacts)))
    k_num = 0
    for i, contact in enumerate(contacts):
        body = contact[0]
        theta = contact[4]
        mu_angle = arctan(contact[5])
        point = np.array([contact[2], contact[3], 0])
        F1, F2 = contact_wrench(theta, mu_angle, point)
        if int(contact[1]) != 0:
            F1_1, F2_1 = contact_wrench(theta, mu_angle, point, pi)
            F1 = np.append(F1, F1_1)
            F2 = np.append(F2, F2_1)
        row = int((body - 1)*3)
        col = k_num
        for n, val in enumerate(F1):
            r = n + row
            F[r, col] = val
        k_num += 1
        col = k_num
        for n, val in enumerate(F2):
            r = n + row
            F[r, col] = val
        k_num += 1
    Aeq = F
    c = np.ones(2*len(contacts))
    beq = np.array([])
    for body in bodies:
        CoM = np.array([body[0], body[1], 0])
        m = np.array([0, -1 * g * body[2], 0])
        F_ext = np.array([np.cross(CoM, m)[2], 0, m[1]])
        beq = np.append(beq, F_ext * -1)

    k = optimize.linprog(c, A_eq=Aeq, b_eq=beq, method='interior-point')

    if k["success"]:
        print("The Assembly has achieved Stability!")
    else:
        print("The Assembly has not achieved Stability under current parameters")


def contact_wrench(theta, mu_angle, point, pi_val=0):
    theta_1 = theta + mu_angle + pi_val
    theta_2 = theta - mu_angle + pi_val
    normal_1 = np.array([cos(theta_1), sin(theta_1), 0])
    normal_2 = np.array([cos(theta_2), sin(theta_2), 0])
    F1 = np.array([np.cross(point, normal_1)[2], normal_1[0], normal_1[1]])
    F2 = np.array([np.cross(point, normal_2)[2], normal_2[0], normal_2[1]])
    return F1, F2


"""
INPUT:::
"""

print("Test 1: Collapsing assembly:::")
body_1 = np.array([[25, 35, 2], [66, 42, 10]])
contacts_1 = np.array([[1, 2, 60, 60, pi, 0.5], [1, 0, 0, 0, pi/2, 0.5],
                        [2, 0, 72, 0, pi/2, 0.5], [2, 0, 60, 0, pi/2, 0.5]])
assembly_stability(body_1, contacts_1)

print("\nTest 2: Assembly that can continue to stand:::")
body_2 = np.array([[25, 35, 2], [66, 42, 5]])
contacts_2 = np.array([[1, 2, 60, 60, pi, 0.5], [1, 0, 0, 0, pi/2, 0.1],
                        [2, 0, 72, 0, pi/2, 0.5], [2, 0, 60, 0, pi/2, 0.5]])
assembly_stability(body_2, contacts_2)

print("\nMy test 1: Collapsing assembly:::")
body_3 = np.array([[25, 35, 2], [66, 42, 40], [80.66, 7.5, 1]])
contacts_3 = np.array([[1, 2, 60, 60, pi, 0.5], [1, 0, 0, 0, pi/2, 0.1],
                       [2, 0, 72, 0, pi/2, 0.5], [2, 0, 60, 0, pi/2, 0.5],
                       [3, 0, 89.32, 0, pi/2, 0.5], [2, 3, 72, 10, pi, 0.5]])
assembly_stability(body_3, contacts_3)

print("\nMy test 2: Collapsing assembly:::")
# The coefficients for body 2 with ground were lowered, and coefficients for body 3 with ground were increased
contacts_4 = np.array([[1, 2, 60, 60, pi, 0.5], [1, 0, 0, 0, pi/2, 0.5],
                       [2, 0, 72, 0, pi/2, 0.2], [2, 0, 60, 0, pi/2, 0.2],
                       [3, 0, 89.32, 0, pi/2, 0.9], [2, 3, 72, 10, pi, 0.5]])
assembly_stability(body_3, contacts_4)
