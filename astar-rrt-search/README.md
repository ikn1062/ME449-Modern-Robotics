# Modern Robotics, Course 4: Robot Motion Planning and Control

For this assignment, I created an RRT planner using Python (in "code" directory).


The implementation
------------------- 
The RRT algorithm is implemented as a function, with several helper functions.

The input to the RRT algorithm include:
    :param start_position: Starting position, np.array[x, y]
    :param end_position: End position (goal), np.array[x, y]
    :param obstacles: Obstacles as circles, taken from obstacles.csv
    :param window: Window is the C-space, given by np.array[[Top-left.x, Top-left.y], [Bottom-Right.x, Bottom-Right.y]]
    :param rad: Radius of final point
    :param stepsize: The length of each edge
    :param n_iter: Max number of iterations
    :return: nodes, edge, and path, in terms of a numpy array and saves files to a csv

The RRT algorithm makes use of a graph tree and nodes, and creates a graph based on a random selection of nodes. Before 
each new node that is added to the graph, it is checked for collisions with obstacles. The process of adding nodes
continues until the goal is reached. The path is then determined from the A-Star path planner from the previous submission.

