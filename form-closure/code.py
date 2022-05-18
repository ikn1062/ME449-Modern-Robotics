import numpy as np

from scipy import optimize
from numpy import cos
from numpy import sin
from numpy import pi


def form_closure(matrix_contacts, n=3):
    """
    :param matrix_contacts: A numpy array containing all contacts as [x, y, normal_angle (rad)]
    :param n: Represents planar (n=3), spatial (n=6)
    :return: Array of k and print statement if successful, returns None if not
    """
    # Creating F from input
    F = np.array([])
    for i, contact in enumerate(matrix_contacts):
        theta = contact[2]
        normal = np.array([cos(theta), sin(theta), 0])
        point = np.array([contact[0], contact[1], 0])
        moment = np.cross(point, normal)[2]
        Fi = np.around(np.array([moment, normal[0], normal[1]]), decimals=3)
        F = np.append(F, Fi)

    r, c = matrix_contacts.shape
    F = F.reshape(r, 3)
    F = F.transpose()

    # Checking rank of F
    rank = np.linalg.matrix_rank(F)
    if rank < n:
        print("Rank of F is not full")
        return None

    # Running linear program
    c = np.ones(r)
    A = np.identity(r) * -1
    b = c * -1
    Aeq = F
    beq = np.zeros(n)
    k = optimize.linprog(c, A, b, Aeq, beq, method='interior-point')

    # Checking and returning output
    for i in k['x']:
        if i <= 0:
            print("k values are not strictly positive")
            return None

    print("Form Closure")
    return k['x']


# Input matrix
contact_case_1 = np.array([[1, 0, pi/2], [2, 0, pi/2], [3, 3, pi], [0, 6, pi/2]])
contact_case_2 = np.array([[1, 0, pi/2], [2, 0, pi/2], [3, 3, pi], [0, 6, -pi/6]])
print("contact_case_1: ")
print(form_closure(contact_case_1))
print("\ncontact_case_2: ")
print(form_closure(contact_case_2))

