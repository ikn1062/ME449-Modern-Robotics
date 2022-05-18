import numpy as np


# FUNCTION :::
def a_star(nodes, edges):
    # The function A* computes a path based on the weights between undirected edges from start to goal
    # INPUT: CSV file of nodes and edges (taken from sample)
    # OUTPUT: CSV file of path

    # Creates an OPEN and CLOSED array to store values
    OPEN = np.array([])
    CLOSED = np.array([])
    mat_a_star = np.array(nodes[:, 0]).copy()

    # Initialises past cost table
    past_cost = np.empty_like(mat_a_star)
    i = 0
    while i < len(mat_a_star):
        past_cost[i] = 9999
        i = i + 1
    past_cost[0] = 0

    # Initialises optimistic cost to go based off data
    o_ctg = np.array(nodes[:, 3]).copy()

    # Initialises estimate cost based off data
    est_cost = np.array(past_cost).copy()
    est_cost[0] = o_ctg[0]

    # Initialises parent node table
    p_node = np.zeros_like(mat_a_star)
    p_node = np.vstack((np.array(nodes[:, 0]), p_node))
    p_node = p_node.transpose()

    # While loop that loops through OPEN variables
    OPEN = np.append(OPEN, 1)
    while len(OPEN) > 0:
        # Moves first var in OPEN to current and CLOSED, and removes it from OPEN
        current = OPEN[0]
        OPEN = np.delete(OPEN, 0)
        CLOSED = np.append(CLOSED, current)

        # If current = goal, the path.csv file is created using the path generated from parent node table
        if current == mat_a_star[-1]:
            path = np.array([])
            i = mat_a_star[-1]
            while i != 0:
                path = np.append(path, i)
                i = p_node[int(i)-1, 1]
            path = np.sort(path)[:, None]
            np.savetxt('path.csv', np.transpose(path), delimiter=',')
            return path

        # Finds neighbor of current using data from edge csv
        nbr = np.array([])
        for i, item in enumerate(edges[:, 0]):
            if edges[i, 1] == current:
                nbr = np.append(nbr, edges[i, 0])

        # Loops through all neighbors of current, finding the best path
        for i in nbr:
            if i in CLOSED:
                continue
            # Calculates cost from current to neighbor based off data
            for x, item in enumerate(edges[:, 0]):
                if edges[x, 0] == i and edges[x, 1] == current:
                    cost = edges[x, 2]
            ten_past_cost = cost + past_cost[int(current) - 1]
            # Replaces past cost and parent node, if the tentative past cost is less than current past cost
            if ten_past_cost < past_cost[int(i) - 1]:
                past_cost[int(i) - 1] = ten_past_cost
                p_node[int(i) - 1, 1] = current
                est_cost[int(i) - 1] = past_cost[int(i) - 1] + o_ctg[int(i) - 1]
                # Places neighbor in OPEN (if not already) and sorts OPEN based on estimated cost
                if i not in OPEN:
                    OPEN = np.append(OPEN, i)
                for j in range(len(OPEN)-1, 0, -1):
                    for k in range(j):
                        a = OPEN[k] - 1
                        b = OPEN[k + 1] - 1
                        if est_cost[int(a)] > est_cost[int(b)]:
                            temp = OPEN[k]
                            OPEN[k] = OPEN[k+1]
                            OPEN[k+1] = temp


# INPUT::::
# These take in the nodes and edges csv file and inputs them into the A* function above
#nodes_1 = np.genfromtxt("nodes.csv", delimiter=",").astype("float")
# edges_1 = np.genfromtxt("edges.csv", delimiter=",").astype("float")
# print(edges_1)
#a_star(nodes_1, edges_1)
