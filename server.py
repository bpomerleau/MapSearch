from adjacencygraph import AdjacencyGraph as g
import math
import sys

# this python program is able to use functions that read in data input from a
# .txt file containing latitudes and longitudes of various parts of the city of
# Edmonton

# Here we are building the digraph containing all the edges and vertices inside
# the Edmonotn Graph

# create graph g with UndirectedAdjacencyGraph class



# open the Edmonton Grph .txt file
Edmonton_Graph = open("edmonton-roads-2.0.1.txt")  # read entire .csv file
# read each line to determine whether to add vertices or add edges
vertex_coord = {}
edge_street_name = {}
for i in Edmonton_Graph:
    # split by commas
    i = i.strip().split(",")
    if i[0] == 'V':  # when the line is describing a vertex
        g.add_vertex(int(i[1]))
        # Store the latitudes and longitudes into a tuple
        vertex_coord[int(i[1])] = (int(float(i[2]) * 100000),
                                   int(float(i[3]) * 100000))
    elif i[0] == 'E':  # when the line is describing an edge
        g.add_edge((int(i[1]), int(i[2])))
        # Store the street name into the dictionary
        edge_street_name[i[1] + ", " + i[2]] = i[3]


def wait_response():
    # create a stdin stdout program to communicte and create pathways
    command = ""
    while True:
        if sys.stdin:
            for i in sys.stdin:
                i = i.split().strip(",")
                command = i[0]
                if commmand == 'R':
                    #read coordinates and return coordinates
                elif command == 'A':
                    #return nothing and continue
                elif command == 'E':
                    #end the program

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
        that start is a vertex of graph.
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
    reached = dict()  # (empty dictionary)

    runners = Minheap()
    # a runner is an event from one vertex to another.
    # runners is an instance of the Minheap() class developed in class. Data in
    # the runners class is stored as follows:
    #     (time, (time + cost(origin, goal), origin, goal)
    # origin is the vertex at which the runner started.
    # goal is the next vertex to be visited.
    # time is the amount of 'time' taken since start. amount of time is
    # determined by the cost function

    runners.add(0, (0, start, start)) # start the first runner at vertex 'start'

    while not runners.isempty():
        #evaluate the runner with the shortest
        (t, (time, origin, goal)) = runners.pop_min()

        if goal in reached:
            # if goal is in reached another runner has already made it there in
            # less time. Ignore this runner and continue to the next
            continue

        reached[goal] = (origin, time) # runner got to dict key from dict value
        if goal == dest:
            path = []
            while goal != start:
                path.append(goal)
                goal = reached[goal]
            path.append(start)
            path.reverse()
            return path
        for v_next in graph.neighbours(goal):
            if v_next in reached:
                #some other runner has made it to vertex faster, do not send
                #runner
                continue
            runners.add(time, (time + cost(goal, v_next), goal, v_next))
                    #    for each succ in goal
    #       add runner (time + cost(goal, succ), succ, goal) to runners
    #          (this new runner will reach succ at the given time)
    # return reached

    return []


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


if __name__ == __main__:
    
