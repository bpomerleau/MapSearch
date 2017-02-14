# server.py - by Jose Ramirez and Brady Pomerleau- CMPUT 275 Section EB1

from adjacencygraph import AdjacencyGraph
import math
import sys

# this python program is able to use functions that read in data input from a
# .txt file containing latitudes and longitudes of various parts of the city of
# Edmonton

# Here we are building the digraph containing all the edges and vertices inside
# the Edmonton Graph

# create graph g with directed AdjacencyGraph class
g = AdjacencyGraph()

# open the Edmonton Grph .txt file
Edmonton_Graph = open("edmonton-roads-2.0.1.txt")  # read entire .csv file
# read each line to determine whether to add vertices or add edges
vertex_coord = {}
edge_street_name = {}
for i in Edmonton_Graph:
    # split by commas
    i = i.strip().split(",")
    if i[0] == 'V':  # when the line is describing a vertex
        g.add_vertex(int(i[1]))  # add the vertex to the graph
        # Store the latitudes and longitudes into a tuple
        vertex_coord[int(i[1])] = (int(float(i[2]) * 100000),
                                   int(float(i[3]) * 100000))
    elif i[0] == 'E':  # when the line is describing an edge
        g.add_edge((int(i[1]), int(i[2])))  # add the edge to the graph
        # Store the street name into the dictionary
        edge_street_name[i[1] + ", " + i[2]] = i[3]

Edmonton_Graph.close()

def wait_response():
    # create a stdin stdout program to communicte and create pathways
    command = ""
    i = sys.stdin.readline()
    i = i.strip().split()
    command = i[0]
    if command == 'R':
        # read vertex coordinates and return coordinates
        return [(int(i[1]), int(i[2])), (int(i[3]), int(i[4]))]
    elif command == 'A':
        # return True and continue
        return True


def closest_vertex(lat1, lat2):
    ''' Finds the nearest vertex to these coordinates
    returns: vertex value'''
    min_cost = 1000000000000000  # Some arbitrary large number
    close_vertex = 0
    for i in vertex_coord:
        vertex_tuple = vertex_coord[i]
        first_squared = (lat1 - vertex_tuple[0]) * (lat1 - vertex_tuple[0])
        second_squared = (lat2 - vertex_tuple[1]) * (lat2 - vertex_tuple[1])
        test_cost = math.sqrt(first_squared + second_squared)
        if test_cost <= min_cost:
            min_cost = test_cost
            close_vertex = i
    return close_vertex


def least_cost_path(graph, start, dest, cost):
    """Find and return a least cost path in graph from start vertex to dest vertex.
    Efficiency: If E is the number of edges, the run-time is
      O( E log(E) ).
    Args:
      graph (Graph): The digraph defining the edges between the
        vertices.
      start: The vertex where the path starts. It is assumed
        that start is a vertex of graph.
      dest:  The vertex where the path ends. It is assumed
        that dest is a vertex of graph.
      cost:  A function, taking the two vertices of an edge as
        parameters and returning the cost of the edge. For its
        interface, see the definition of cost_distance.
    Returns:
      list: A potentially empty list (if no path can be found) of
        the vertices in the graph. If there was a path, the first
        vertex is always start, the last is always dest in the list.
        Any two consecutive vertices correspond to some
        edge in graph.
    """
    import heapq

    reached = dict()  # (empty dictionary)

    runners = []
    # a runner is an event from one vertex to another.
    # runners is list of tuples organized as follows:
    #     (time + cost(origin, goal), origin, goal)
    # origin is the vertex at which the runner started.
    # goal is the next vertex to be visited.
    # time is the amount of 'time' taken from start to origin. amount of
    # time is determined by the cost function

    # start the first runner at vertex 'start'
    heapq.heappush(runners,(0, start, start))

    while runners:
        #evaluate the runner with the shortest
        (time, origin, goal) = heapq.heappop(runners)


        if goal in reached:
            # if goal is in reached another runner has already made it there in
            # less time. Ignore this runner and continue to the next
            continue

        reached[goal] = (origin, time) # runner got to dict key from dict value
        if goal == dest:
            path = []
            while goal != start:
                path.append(goal)
                (goal, v) = reached[goal]
            path.append(start)
            path.reverse()
            return path
        for v_next in graph.neighbours(goal):
            if v_next in reached:
                #some other runner has made it to vertex faster, do not send
                #runner
                continue
            heapq.heappush(runners, (time + cost(goal, v_next), goal, v_next))
    return [] # if dest not reached, return empty list


def cost_distance(u, v):
    '''Computes and returns the straight-line distance between the two vertices
       u and v.
    Args:
        u, v: The ids for two vertices that are the start and
          end of a valid edge in the graph.
        Returns:
          numeric value: the distance between the two vertices.
    '''
    coord1_tuple = vertex_coord[u]
    coord2_tuple = vertex_coord[v]
    first_squared = (coord2_tuple[0] - coord1_tuple[0]) * \
        (coord2_tuple[0] - coord1_tuple[0])
    second_squared = (coord2_tuple[1] - coord1_tuple[1]) * \
        (coord2_tuple[1] - coord1_tuple[1])

    return math.sqrt(first_squared + second_squared)

if __name__ == '__main__':
    # beginning communication to the arduino
    while True:
        # the program will first recieve the coordinates to calculate
        start_end_coord = wait_response()

        # get the closest vertex near the starting coordinate
        vertex1 = closest_vertex(start_end_coord[0][0], start_end_coord[0][1])
        # get the closest vertex near the ending coordinate
        vertex2 = closest_vertex(start_end_coord[1][0], start_end_coord[1][1])

        # find the minimum path to get from start to end
        destination_path_list = least_cost_path(g, vertex1, vertex2, cost_distance)
        # find the number of waypoints the path takes
        num_waypoints = len(destination_path_list)
        # print number of waypoints to arduino
        sys.stdout.write("N " + str(num_waypoints) + "\n")
        if num_waypoints > 0:
            arduino_continue = wait_response()
            for i in destination_path_list:
                arduino_continue = False
                sys.stdout.write(
                    "W " + str(vertex_coord[i][0]) + " " + str(vertex_coord[i][1])
                    + "\n")
                while not arduino_continue:
                    arduino_continue = wait_response()
            sys.stdout.write("E" + "\n")
        # else:
        #     break
