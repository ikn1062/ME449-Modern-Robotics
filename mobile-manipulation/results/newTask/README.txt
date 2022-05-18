Mobile Manipulation Capstone
Submission date: 12/08/2020
Author initials = IKN


Results: newTask

initial cube config: (x,y,theta)=(0.5,0.5,pi/3)
final cube config: (x,y,theta)=(1,0,-pi)

controller: feedforward-plus-PI

INPUT:::

Tsc_i = np.array([[cos(pi/3), -sin(pi/3), 0, 0.5], [sin(pi/3), cos(pi/3), 0, 0.5], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_g = np.array([[cos(-pi), -sin(-pi), 0, 1], [sin(-pi), cos(-pi), 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tse_i = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

robot_config = np.array([-pi/6, 0, 0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

Ki = 0.1 * np.identity(6)
Kp = 2 * np.identity(6)


