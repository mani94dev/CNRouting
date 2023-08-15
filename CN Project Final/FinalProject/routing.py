import csv
import sys
import math

"""Using a graph data structure to store each vertex and the associated edge cost"""


class Graph:
    def __init__(self, vertices):
        self.V = vertices  # Total number of vertices in the graph
        self.graph = []  # Array of edges

    # Add edge to the graph
    def add_edge(self, s, d, w):
        self.graph.append([s, d, w])

    # Using for printing the solution
    def print_solution(self, dist, src, variable_names):
        variable_map = {i: v for i, v in enumerate(variable_names)}
        output = " ".join(str(d) for d in dist)
        print(f"Distance vector for node {variable_map[src]}: {output}")

    def bellman_ford(self, src, variable_names):

        # Fill the distance array and predecessor array
        dist = [float("Inf")] * self.V
        # Mark the source vertex
        dist[src] = 0

        # Relax edges |V| - 1 times
        for _ in range(self.V - 1):
            for s, d, w in self.graph:
                if dist[s] != float("Inf") and dist[s] + w < dist[d]:
                    dist[d] = dist[s] + w

        # Detect negative cycle if value changes then we have a negative cycle in the graph
        # and we cannot find the shortest distances
        for s, d, w in self.graph:
            if dist[s] != float("Inf") and dist[s] + w < dist[d]:
                print("Graph contains negative weight cycle")
                return

        # No negative weight cycle found
        # Print the distance and predecessor array
        self.print_solution(dist, src, variable_names)


""" function for generating least cost&path dictionary with src node,
 list of nodes and costs as input"""


def dijAlgol(src, varList, costDict):
    distDict = {v: [math.inf, None] for v in varList}
    distDict[src] = [0, src]
    unvisited = set(varList)

    while unvisited:
        curr = min(unvisited, key=lambda v: distDict[v][0])
        unvisited.remove(curr)

        for neighbor in varList:
            if neighbor not in unvisited:
                continue

            cost = costDict[curr][neighbor]
            alt = distDict[curr][0] + int(cost)
            if alt < distDict[neighbor][0]:
                distDict[neighbor] = [alt, distDict[curr][1] + neighbor]

        if curr == src and not unvisited:
            break

    printPath(src, varList, distDict)
    return distDict


"""Reading the csv file for getting the header and costs for each edge"""


def read_csv(filename):
    with open(filename, mode='r') as csv_file:
        reader = csv.reader(csv_file, delimiter=',')
        headers = next(reader)[1:]  # extract variable names
        costs = {}
        for row in reader:
            node = row[0]
            cost_values = [int(cost) for cost in row[1:]]
            costs[node] = dict(zip(headers, cost_values))  # create dictionary with costs for each node
        return headers, costs


"""For getting the input node"""


def get_input_node(variables):
    while True:
        src = input("Please, provide the source node: ")
        if src in variables:
            return src
        else:
            print("Wrong input node given.")


""" Function for printing the least cost path and with src node,
 variable list and least cost & path dictionary as input """


def printPath(src, varList, distDict):
    print("Shortest path tree for node " + src + ":")
    path = ""
    list = []
    for index in range(0, len(varList)):
        if src == varList[index]:
            pass
        else:
            list.append(distDict[varList[index]][-1])
    list = sorted(list, key=len)  # sorting the shortest path by length
    for index in range(0, len(list)):
        path += list[index] + ", "
    print(path[:-2])
    path = ""
    print("Costs of least-cost paths for node " + src + " : ")
    for index in range(0, len(varList)):
        path += (varList[index] + ":" + str(distDict[varList[index]][0]) + ", ")
    print(path[:-2])  # printing distance of all nodes from source node


"""This method takes three arguments: the indices of the two nodes that 
form the edge, and the weight of the edge. The indices are obtained from
 the val dictionary, and the weight is obtained from the cost_dict dictionary. """


def add_nodes(g, cost_dict, headers):
    val = {var: idx for idx, var in enumerate(sorted(headers))}
    for node in cost_dict:
        for innode in cost_dict[node]:
            g.add_edge(val[node], val[innode], cost_dict[node][innode])


""" This code reads in a CSV file of costs and variable names,
 and then performs the Dijkstra algorithm and Bellman-Ford algorithm on
 a graph constructed from the data in the CSV file. """
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python script.py <filename>")
        sys.exit(1)

    filename = sys.argv[1]
    variable_names, cost_dict = read_csv(filename)

    print(f"Variable names : {variable_names}")
    src = get_input_node(variable_names)
    dijAlgol(src, variable_names, cost_dict)
    g = Graph(len(variable_names))
    add_nodes(g, cost_dict, variable_names)
    for i in range(len(variable_names)):
        g.bellman_ford(i, variable_names)
