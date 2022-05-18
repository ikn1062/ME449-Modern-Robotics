Mobile Manipulation Capstone
Submission date: 12/08/2020
Author initials = IKN


Results: OVERSHOOT

controller: feedforward-plus-PI

INPUT:::

Tsc_i = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_g = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tse_i = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

robot_config = np.array([-pi/6, 0, 0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

Ki = 20 * np.identity(6)
Kp = 2 * np.identity(6)


