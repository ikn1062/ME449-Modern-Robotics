import numpy as np
import random as r
from astar import a_star

"""
Function: RRT_Planner
Inputs: obstacles.csv, start node, end node, boundaries for area
Outputs: edges.csv, nodes.csv, path.csv

The function takes in a list of obstacles, defined in the csv file, and uses an RRT planning algorithm to define a path
from the start node to the end node. 
"""

"""
HELPER FUNCTIONS:::
"""


class Node:
    def __init__(self, x_pos, y_pos):
        self.x = x_pos
        self.y = y_pos
        self.parent = None

    def pos(self):
        return np.array([self.x, self.y])


class Graph:
    def __init__(self, node_start, node_end, window, goal_samp=5):
        self.start = node_start
        self.end = node_end

        self.nodes = [node_start]
        self.edges = []
        self.goal = False

        self.node2idx = {node_start: 0}

        self.tl = window[0]
        self.br = window[1]
        self.g_samp = goal_samp

    def add_node(self, node_new, node_near):
        idx = len(self.node2idx)
        self.nodes.append(node_new)
        node_new.parent = node_near
        self.node2idx[node_new] = idx

    def add_edge(self, node_new, node_near):
        dist = distance(node_new, node_near)
        idx2 = self.node2idx[node_near]
        idx1 = self.node2idx[node_new]
        self.edges.append([idx1, idx2, dist])

    def random_pos(self):
        if r.randint(0, 100) > self.g_samp:
            x = r.randrange(self.tl[0]*20, self.br[0]*20)/20
            y = r.randrange(self.br[1]*20, self.tl[1]*20)/20
            return np.array([x, y])
        else:
            return self.end.pos()


class Line:
    def __init__(self, pos1, pos2):
        """
        Helper Class
        :param pos1: Position of first point, np.array[x, y]
        :param pos2: Position of second point, np.array[x, y]
        """
        self.p = np.array(pos1)
        self.dist = distance(pos1, pos2)
        self.direction = (np.array(pos2) - np.array(pos1))/self.dist


def distance(pos_1, pos_2):
    """
    Helper function used to calculate distance from two points/nodes
    :param pos_1: Position of first point, np.array[x, y]
    :param pos_2: Position of second point, np.array[x, y]
    :return: distance from point 1 to point 2, np.float
    """
    if type(pos_1) == Node and type(pos_2) == Node:
        pos1 = pos_1.pos()
        pos2 = pos_2.pos()
    else:
        pos1 = np.array(pos_1)
        pos2 = np.array(pos_2)
    return np.linalg.norm(pos1 - pos2)


def line_intersection(line, obstacles):
    """
    Helper function to determine intersection of a line with a circle
    :param line: A line defined by class Line, type: Line
    :param obstacles: obstacles: A list of obstacles provided by the csv [x, y, rad]
    :return: boolean if line intersects with obstacle
    """
    # https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm

    for obstacle in obstacles:

        center = np.array(obstacle[:2])
        radius = np.array(obstacle[2])

        a = np.dot(line.direction, line.direction)
        b = 2 * np.dot(line.direction, line.p - center)
        c = np.dot(line.p - center, line.p - center) - radius * radius

        disc = b * b - 4 * a * c

        if disc < 0:
            continue

        t1 = (-b + np.sqrt(disc)) / (2 * a)
        t2 = (-b - np.sqrt(disc)) / (2 * a)

        if t1 < 0 and t2 < 0:
            continue

        if t1 > line.dist and t2 > line.dist:
            continue
        return True
    return False


def point_intersection(point, obstacles):
    """
    Helper function to determine if a given point is in an obstacle
    :param point: A point of a given node, np.array[x, y]
    :param obstacles: A list of obstacles provided by the csv [x, y, rad]
    :return: boolean if point intersects with obstacle
    """
    for obstacle in obstacles:
        if distance(point, obstacle[:2]) < obstacle[2]:
            return True
    return False


def node_nearest(graph, node_new, obstacles):
    """
    Helper function that finds the nearest node to the point inputted
    :param graph: A graph given by class Graph
    :param node_new: A new_node position given by np array, np.array[x, y]
    :param obstacles: A list of obstacles provided by the csv [x, y, rad]
    :return: The nearest node to the point
    """
    node_near = None
    min_dist = float("inf")

    for i, node in enumerate(graph.nodes):

        if distance(node.pos(), node_new) != 0:
            line = Line(node.pos(), node_new)
        else:
            continue

        if line_intersection(line, obstacles):
            continue

        dist = distance(node.pos(), node_new)
        if dist < min_dist and dist != 0:
            min_dist = dist
            node_near = node
    return node_near


def make_new_node(rand_point, near_node, stepsize=0.05):
    """
    Helper function that creates a new node based on a point
    :param rand_point: Random point generated from the Graph.random_pos, np.array[x, y]
    :param near_node: Near node taken from node nearest function, type: Node
    :param stepsize: Step size used to define the length of the line
    :return: A new node with position rand_point in type Node, type: Node
    """
    n_node = near_node.pos()
    direct = rand_point - n_node
    leng = np.linalg.norm(direct)
    
    direct = (direct/leng) * stepsize

    node = np.array([n_node[0] + direct[0], n_node[1] + direct[1]])
    node = Node(node[0], node[1])
    return node


def save_files(nodes, edges, path):
    """
    Converts output of RRT algorithm into csv file
    :param nodes: nodes output from RRT alg
    :param edges: edges output from RRT alg
    :param path: path output from RRT alg
    :return: N/A, csv files are created
    """
    edges = np.around(edges, decimals=5)
    nodes = np.around(nodes, decimals=5)
    np.set_printoptions(suppress=True)
    path = np.flip(np.array(path))[:, None]
    for i in nodes:
        i[0] += 1
    for i in path:
        i[0] += 1
    for i in edges:
        i[0] += 1
        i[1] += 1
    edges = np.flip(edges, 0)

    np.savetxt('edges.csv', edges, delimiter=',')
    np.savetxt('nodes.csv', nodes, delimiter=',')
    a_star(nodes, edges)
    # np.savetxt('path.csv', np.transpose(path), delimiter=',')


"""
RRT ALGORITHM:::
"""


def RRT_alg(start_position, end_position, obstacles, window, rad=0.05, stepsize=0.05, n_iter=100):
    """
    RRT planner
    :param start_position: Starting position, np.array[x, y]
    :param end_position: End position (goal), np.array[x, y]
    :param obstacles: Obstacles as circles, taken from obstacles.csv
    :param window: Window is the C-space, given by np.array[[Top-left], [Bottom-Right]]
    :param rad: Radius of final point
    :param stepsize: The length of each edge
    :param n_iter: Max number of iterations
    :return: nodes, edge, and path, in terms of a numpy array
    """
    start_node = Node(start_position[0], start_position[1])
    end_node = Node(end_position[0], end_position[1])
    g = Graph(start_node, end_node, window)

    for obstacle in obstacles:
        obstacle[2] /= 2

    n = 0
    d2goal = 2
    while n < n_iter:

        rand_pos = g.random_pos()

        if point_intersection(rand_pos, obstacles):
            continue

        near_node = node_nearest(g, rand_pos, obstacles)
        if near_node is None:
            continue

        new_node = make_new_node(rand_pos, near_node, stepsize)

        g.add_node(new_node, near_node)
        g.add_edge(new_node, near_node)

        if distance(new_node.pos(), g.end.pos()) < d2goal:
            d2goal = distance(new_node.pos(), g.end.pos())

        n += 1

        if d2goal < rad:
            g.add_node(end_node, new_node)
            g.add_edge(end_node, new_node)
            g.goal = True
            print('SUCCESS in {} iterations!'.format(n))
            break

    nodes_arr = []
    for node in g.nodes:
        idx = g.node2idx[node]
        node_x = node.pos()[0]
        node_y = node.pos()[1]
        otg_dst = distance(node.pos(), end_node.pos())
        nodes_arr.append([idx, node_x, node_y, otg_dst])

    path = [g.node2idx[end_node]]
    path_node = end_node
    while path_node.parent is not None:
        path_node = path_node.parent
        path.append(g.node2idx[path_node])

    save_files(nodes_arr, g.edges, path)
    return nodes_arr, g.edges, path


"""
INPUTS:::

These inputs are defined in the RRT function above
"""


start_pos_in = np.array([-0.5, -0.5])
end_pos_in = np.array([0.5, 0.5])
obstacles_in = np.genfromtxt("obstacles.csv", delimiter=",").astype("float")
window_in = np.array([[-0.5, 0.5], [0.5, -0.5]])

RRT_alg(start_pos_in, end_pos_in, obstacles_in, window_in, 0.05, 0.2, 1000)
