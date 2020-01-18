"""
Formulation of search problem
the dataset was converted to graph of nodes and edges having varying weights

State Space:
the state space is just the list reference of the node which is accesed with the Graph class

Successor Function:
successors are all the neighboring nodes of the current node which is being explored, except the visited node in BFS, DFS and IDS. In Astar the cost of visited node can be updated if better cost is incurred
the successor function choses the desired successor as per the algorithm and routing option chosen by user

Edge Weights:
it depends on the routing option which is selected by user, explained in Question 4 below

Heuristic Function
Explained in Question 4 below

Description:
The user gives the source and destination the routing option and the algorithm to be used.
for BFS DFS and IDS routing option does not matter as they are blind search techniques
for ASTAR search different heuristic and cost functions were used which is discussed in detail in question 4 below
I created 2 classes Node which is used for storing the node of the graph and the Graph class used to access the Node class. and a Node Dictionary whcich contained the references of all Node objects
I created a dictionary for storing miles, distance speed limit and road name betwwen 2 nodes eg for distance node1.miles[node2] gives distance between node1 and node2 thus helping in segregation of data
and then I applied various algorithms for different cases.

Problems Faced:
The major problems was the making the dataset consistent as
    -there were cases where speed limit was 0 or not mentioned in this case I selected 30 as default
    -there were places whose co-ordinates were not given so I averaged latitude and longitude of the neighbors in some cases even the neighbors din't have co ordinates so I selected them as 0.
     for the purpose of time efficiency I did not generate for all, only when I encounter such node i generated these parameters.
Formulation of Heuristic was also a problem in some cases, but by trial and error I got the heuristics which gave me the optimal solution


Question (1) Which search algorithm seems to work best for each routing options?
Solution: The A* search algorithm seems to work best for each routing options and gives optimal solution for all kind of routes

Question (2) Which algorithm is fastest in terms of the amount of computation time required
by your program, and by how much, according to your experiments?
Solution: BFS is the fastes computationally than any other algorithms it is faster than DFS more than twice of its computational time on an average
DFS is slightly less faster than BFS
for Astar there is no specific order it depends on the arrangement of the graph hence the computational time can be ordered by any way
IDS is the slowest as it executes DFS d number of times where d is the depth of the destination

Question (3) Which algorithm requires the least memory, and by how much, according to your experiments?
Solution: I have used memory profiler in python for the getting the memory usage as per time the respective graphs are present in the directory.
According to my experiments BFS takes the lowest memory than other algorithms as we can see in the graphs. it takes almost 1 Mb less than DFS. DFS and IDS take almost the same memory
but for ASTAR it differs for different experiments, it depends on the graph sometime scenic gives the lowest memory consumption sometimes others.

Question (4) Which heuristic function did you use, how good is it, and how might you make it better?
Solution:
    -for distance : heuristic function is the Haversine distance between the current node and the destination and the cost function is the miles from current node to source. This is admissible because the distance from node cant be greater than this
                    I also tried euclidean distance as heuristic but sometimes it does not give the optimal solution hence used haversine distance which gives optimal solution
                    If we get all data about the latitude and longitude of each and every node or junction we can improve it. I have averaged the latitude and longitude for the nodes whose values are not present with its neighbors

    -for time : heuristic function is the Haversine distance/85=time between the current node and the destination considering 85 is the highest speed limit. This is admissible because the time of reaching the destination by road can't be smaller than this. The cost function is the distance/speed limit=time from current node to source
                I also tried using speed limit as 35 and 45 for getting time for heuristic that does not provide the optimal solution. 65 provides the optimal solution as it is the highest speed limit in the given dataset but to be on a safer side I used 85.
                If I get the information of traffic information of atleast the neighboring roads towards destination I can help reduce the time by using a suitable heuristic and also the speed of the current vehicle can help in the cause

    -for scenic : heuristic function is the tuple (penalty,Haversine distance) and the cost function is a penalty of 1 if it takes road having speed limit>=55 and 0 otherwise if penalty is equal then we consider the lowest distance as secondary priority.
                  generally for all the cases the penalty is 0 to avoid any roads with greater speed limits hence it comes down to distance as the distance should be the least the haversine distance will give the lowest possible value than the actual incurred distance via roads hence it is admissible
                  distance function had to be used as a heuristic to guide the search and take the current state near to destination if I had more information about the beautiful surroundings and tourist attractions nearby from current state to neighboring state could have made the heuristic better

    -for segments : heuristic function is haversine distance/4000.0 as 4000 was chosen by trial and error as it is going to be among the largest distance in the map and so no. of segments to the destination would be definitely higher than the division hence it is admissible
                    cost function is the number of segments travelled, the heuristic is better as I am considering the least number of segments that is less than 1 in all but one case to destination
                    If I take the heuristic as the distance between current node and destination the algorithm doesnot give me an optimal solution hence this heuristic
                    I guess this is the best heuristic I can get as it is less than 1 most of the times


Question (5) Supposing you start in Bloomington, which city should you travel to if you want to take the longest possible drive (in miles)
 that is still the shortest path to that city? (In other words, which city is furthest from Bloomington?)
Solution: 'Skagway,_Alaska' is around 2522.37 haversian miles away but by road it goes upto 4542 miles by which is still the shortest path to city by using Astar algorithm optimised for distance
"""

import sys


# Node class for creating Node object of Graph
# Node class cannot be accessed directly, has to be accesed from the graph
class Node:
    def __init__(self, name):
        self.id = name
        self.latitude = 0
        self.longitude = 0
        # Dictionaries to create a links between other neighboring nodes
        self.miles = {}
        self.speedLimit = {}
        self.roadName = {}
        self.time = {}

    def setLatLong(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def addNeighbor(self, neighbor, miles, speedLimit, roadName):
        self.miles[neighbor] = miles
        self.speedLimit[neighbor] = speedLimit
        self.roadName[neighbor] = roadName
        self.time[neighbor] = float(self.miles[neighbor]) / 30 if self.speedLimit[neighbor] == '0' or self.speedLimit[
            neighbor] == '' else float(self.miles[neighbor]) / float(self.speedLimit[neighbor])

    def getNeighbors(self):
        return self.miles.keys()


# Graph class to access Node object through the interface
class Graph:
    def __init__(self):
        self.nodeDict = {}  # Dictionary for creating master for all the nodes present and their respective pointers basically for ease of access

    # Adding Node to nodeDict
    def addNode(self, name):
        newNode = Node(name)
        self.nodeDict[name] = newNode
        return newNode

    def setNodeLatLong(self, name, latitude, longitude):
        # if name in self.nodeDict:
        self.nodeDict[name].setLatLong(latitude, longitude)

    # Creating edge between two Nodes
    def addEdge(self, frm, to, miles, speedLimit, roadName):
        if frm not in self.nodeDict:
            self.addNode(frm)
        if to not in self.nodeDict:
            self.addNode(to)
        # add to and fro edge for both the connecting nodes
        self.nodeDict[frm].addNeighbor(to, miles, speedLimit, roadName)  # give node object as key or just name
        self.nodeDict[to].addNeighbor(frm, miles, speedLimit, roadName)

    def getNodeLatLong(self, nodeName):
        return float(self.nodeDict[nodeName].latitude), float(self.nodeDict[nodeName].longitude)

    def getAllNeighbors(self, nodeName):
        if nodeName in self.nodeDict:
            return self.nodeDict[nodeName].getNeighbors()
        return None


# reading Dataset and creating datastructure
def readDatasets(g):
    path = "road-segments.txt"
    f = open(path)
    print
    "reading file road-segments.txt..."
    line = "text"
    while line:
        line = f.readline().strip()
        try:
            frm, to, miles, speed, roadName = line.split(" ")
        except ValueError:  # Reached EOF
            continue
        g.addEdge(frm, to, miles, speed, roadName)
    f.close()
    print
    "Graph Created for road-segments.txt"
    path = "city-gps.txt"
    f = open(path)
    print
    "Feeding GPS Data..."
    line = "text"
    while line:
        line = f.readline().strip()
        try:
            name, latitude, longitude = line.split(" ")
        except ValueError:  # Reached EOF
            continue
        g.setNodeLatLong(name, latitude, longitude)
    f.close()
    print
    "GPS data in memory..."
    return g


def printPath(visitedFrom, source, destination, g,
              flag):  # flag=1 for printing path and 0 for getting total number of previous hops
    totalDistance = 0
    time = 0
    path = [destination]
    jump = destination
    while True:
        nextStop = visitedFrom[jump]
        path.append(nextStop)
        if nextStop == source:
            path = (path[::-1])
            if flag == 1:
                print
                '\nTake the following path for {} to {}\n'.format(source, destination)
                for i in range(0, len(path) - 1):
                    print
                    "-" * 170
                    print
                    'From {:43} via {:24} to {:43} for {:4} miles with maximum speed:{} mph'.format(path[i], g.nodeDict[
                        path[i]].roadName[path[i + 1]], path[i + 1], g.nodeDict[path[i]].miles[path[i + 1]], g.nodeDict[
                                                                                                        path[
                                                                                                            i]].speedLimit[
                                                                                                        path[i + 1]])
                    totalDistance = totalDistance + int(g.nodeDict[path[i]].miles[path[i + 1]])
                    # Take speed limit as 30mph if speed limit not given
                    time = time + (float(g.nodeDict[path[i]].miles[path[i + 1]]) / 30 if g.nodeDict[path[i]].speedLimit[
                                                                                             path[i + 1]] == "0" or
                                                                                         g.nodeDict[path[i]].speedLimit[
                                                                                             path[
                                                                                                 i + 1]] == "" else float(
                        g.nodeDict[path[i]].miles[path[i + 1]]) / float(g.nodeDict[path[i]].speedLimit[path[i + 1]]))
                print
                "=" * 170
                print
                "\nThis total journey of {} miles will take {:4.4f} hours with {} places visited ".format(totalDistance,
                                                                                                          time,
                                                                                                          len(path) - 1)
                print
                "=" * 170
                print
                "\nNOTE: if the maximum speed is 0 or blank then I have selected the speed to be 30 mph\n"
                print
                "\n{} {:4.4f} {}".format(totalDistance, time, " ".join(map(str, path)))

                return
            else:  # Flag=0 for getting number of hops from source
                return len(path) - 1
        jump = nextStop


# combined function of dfs bfs and also ids
def bfsdfs(source, destination, g, algo, depth):
    visited = {}
    fringe = [source]
    visitedFrom = {}
    while len(fringe) > 0:
        if algo == "bfs":
            start = fringe.pop(0)
        else:
            start = fringe.pop()
        if depth == 0:
            if start == destination:
                visitedFrom[destination] = start
                return True
            return False
        visited[start] = 1
        for neighbor in g.getAllNeighbors(start):
            if neighbor in visited:  # do not explore if the node is visited before
                continue
            visitedFrom[neighbor] = start
            if neighbor == destination:
                printPath(visitedFrom, source, destination, g, 1)
                return True
            if depth != None and printPath(visitedFrom, source, neighbor, g, 0) >= depth:
                continue
            if neighbor not in fringe:
                fringe.append(neighbor)
    return False


def ids(source, destination, g, algo):
    depth = 0
    while not (bfsdfs(source, destination, g, "dfs", depth)):  # call DFS algorithm with varying depths
        depth = depth + 1


# update co ordinates by averaging all Neighbors
def updateCoOrdinates(city, g):
    latTotal = lonTotal = 0
    neighbors = g.getAllNeighbors(city)
    legitimateNeighbors = len(neighbors)
    for neighbor in neighbors:
        if g.getNodeLatLong(neighbor)[0] != 0.0:  # if the neighboring city is not junction
            latTotal = latTotal + getLatLong(neighbor, g)[0]
            lonTotal = lonTotal + getLatLong(neighbor, g)[1]
        else:
            legitimateNeighbors = legitimateNeighbors - 1
            if legitimateNeighbors == 0:
                return -1, -1
    x = latTotal / float(legitimateNeighbors)
    y = lonTotal / float(legitimateNeighbors)
    g.setNodeLatLong(city, x, y)
    return x, y


def getLatLong(city, g):
    cityX, cityY = g.getNodeLatLong(city)
    if (cityX == 0.0):
        return updateCoOrdinates(city, g)
    if (cityX == -1):  # if no coordinates for neighbors as well then set them to 0,0
        cityX = cityY = 0.0
    return cityX, cityY


from math import radians, cos, sin, asin, sqrt


# Euclidean Distance may be misleading as we travel vertically as the earths shape is spherical hence using Haversine Distance
# reference: http://www.movable-type.co.uk/scripts/latlong.html
def haversineDistance(frm, to, g):
    lat1, lon1 = getLatLong(frm, g)
    lat2, lon2 = getLatLong(to, g)
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2.0) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2.0) ** 2
    c = 2 * asin(sqrt(a))
    r = 3956  # Radius of earth in miles 3956 for miles
    return c * r


def costFunction(visitedFrom, source, currentState, g, routingOption):
    cost = 0
    jump = currentState
    while True:
        if jump == source:
            return 0
        nextStop = visitedFrom[jump]
        if routingOption == 'distance':
            cost = cost + float(g.nodeDict[jump].miles[nextStop])
        elif routingOption == 'time':
            cost = cost + float(g.nodeDict[jump].time[nextStop])
        elif routingOption == 'scenic':
            cost = cost + (0 if g.nodeDict[jump].speedLimit[nextStop] == "" or float(
                g.nodeDict[jump].speedLimit[nextStop]) < 55.0 else float(g.nodeDict[jump].miles[nextStop]))
        elif routingOption == 'segments':
            cost = cost + 1
        if nextStop == source:
            return cost
        jump = nextStop


def heuristicFunction(currentState, destination, g, routingOption):
    if routingOption == 'distance' or routingOption == 'scenic':
        return haversineDistance(currentState, destination, g)
    elif routingOption == 'time':
        return haversineDistance(currentState, destination,
                                 g) / 85.0  # considering 85 is the highest speed limit in USA
    elif routingOption == 'segments':  # considering 1 as constant heuristic for segments
        return haversineDistance(currentState, destination, g) / 4000.0


# References: for algorithm https://en.wikipedia.org/wiki/A*_search_algorithm
from Queue import PriorityQueue


def aStar(source, destination, g, routingOption):
    pq = PriorityQueue()
    # priority is tuple (priority,state name) except for scenic
    if routingOption == 'scenic':
        pq.put((0, 0,
                source))  # use 2 priorities: first is penalty(which is 0 if speed<55 else 1) second is distance for scenic route (penalty,distance,state name)
    else:
        pq.put((0, source))
    visitedFrom = {}
    cost = {}
    visitedFrom[source] = None
    cost[source] = 0
    while not pq.empty():
        if routingOption == 'scenic':  # as there is primary and secondary priority for scenic case
            currentState = pq.get()[2]
        else:
            currentState = pq.get()[1]
        if currentState == destination:
            printPath(visitedFrom, source, destination, g, 1)
            return True
        for neighbor in g.getAllNeighbors(currentState):
            if routingOption == "distance":
                neighborCost = costFunction(visitedFrom, source, currentState, g, routingOption) + float(
                    g.nodeDict[currentState].miles[neighbor])
            elif routingOption == "time":
                neighborCost = costFunction(visitedFrom, source, currentState, g, routingOption) + float(
                    g.nodeDict[currentState].time[neighbor])
            elif routingOption == "segments":
                neighborCost = costFunction(visitedFrom, source, currentState, g, routingOption) + 1
            elif routingOption == "scenic":
                neighborCost = costFunction(visitedFrom, source, currentState, g, routingOption) + (
                    0 if g.nodeDict[currentState].speedLimit[neighbor] == "" or float(
                        g.nodeDict[currentState].speedLimit[neighbor]) < 55.0 else float(
                        g.nodeDict[currentState].miles[neighbor]))
            else:
                print
                "proper route option not selected, select the root option among distance, time, segments, scenic"
                return False
            if neighbor not in cost or neighborCost < cost[neighbor]:  # update cost
                cost[neighbor] = neighborCost
                evaluationVal = neighborCost + heuristicFunction(neighbor, destination, g, routingOption)
                if routingOption == 'scenic':
                    pq.put((neighborCost, evaluationVal, neighbor))
                else:
                    pq.put((evaluationVal, neighbor))
                if visitedFrom[currentState] != neighbor:  # condition check for avoiding creation of cycles
                    visitedFrom[neighbor] = currentState
    return False


if __name__ == '__main__':
    import time

    start = time.time()
    g = Graph()
    g = readDatasets(g)  # Convert Dataset to Graph
    source = sys.argv[1]
    destination = sys.argv[2]
    routingOption = sys.argv[3]
    algo = sys.argv[4]
    if algo == "bfs" or algo == "dfs":
        print
        "Warning!! {} is a blind search technique and it may not give an optimal solution".format(algo)
        path = bfsdfs(source, destination, g, algo, None)
    elif algo == "ids":
        print
        "Warning!! {} is a blind search technique and it may not give an optimal solution".format(algo)
        path = ids(source, destination, g, algo)
    elif algo == "astar":
        path = aStar(source, destination, g, routingOption)
    else:
        print
        "Algorithm not properly selected, select the option among bfs, dfs, ids, astar"
        exit(1)
    if path == False:
        print
        "No path found"
    print
    "\n Total Time Taken:" + str(time.time() - start) + " Seconds"
