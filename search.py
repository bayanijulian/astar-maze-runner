# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018
import collections
import math
from itertools import combinations
import copy
import heapq
import time
"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,greedy,astar)

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "dfs": dfs,
        "greedy": greedy,
        "astar": astar,
    }.get(searchMethod)(maze)

def oldbfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    # TODO: Write your code here
    # return path, num_states_explored
    queue = []
    visited = []
    parent = {}
    curr_coord = (maze.getStart()[0], maze.getStart()[1])
    queue.append(curr_coord)
    visited.append(curr_coord)
    num_states_explored = 1
    while (len(queue) > 0) and not (maze.isObjective(curr_coord[0], curr_coord[1])):
        curr_coord = queue.pop(0)
        for neighbor in maze.getNeighbors(curr_coord[0], curr_coord[1]):
            if neighbor not in visited:
                queue.append(neighbor)
                visited.append(neighbor)
                num_states_explored = num_states_explored + 1
                parent[neighbor] = curr_coord
        #print(maze.isObjective(curr_coord[0], curr_coord[1]))
    path = []
    while not (curr_coord == maze.getStart()):
        path.append(curr_coord)
        curr_coord = parent[curr_coord]
    path.append(curr_coord)
    path.reverse()
    return path, num_states_explored

def dfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    stack = []
    visited = []
    parent = {}
    curr_coord = (maze.getStart()[0], maze.getStart()[1])
    stack.append(curr_coord)
    visited.append(curr_coord)
    num_states_explored = 1
    while (len(stack) > 0) and not (maze.isObjective(curr_coord[0], curr_coord[1])):
        curr_coord = stack.pop()
        for neighbor in maze.getNeighbors(curr_coord[0], curr_coord[1]):
            if neighbor not in visited:
                stack.append(neighbor)
                visited.append(neighbor)
                num_states_explored = num_states_explored + 1
                parent[neighbor] = curr_coord
        #print(maze.isObjective(curr_coord[0], curr_coord[1]))
    path = []
    while not (curr_coord == maze.getStart()):
        path.append(curr_coord)
        curr_coord = parent[curr_coord]
    path.append(curr_coord)
    path.reverse()
    return path, num_states_explored

def greedy(maze):
    priroity_queue = []
    visited = []
    parent = {}

    #assuming only one objective for now
    objective = maze.getObjectives()[0]
    curr_coord = maze.getStart()
    distance = manhattan_dist(curr_coord, objective)

    #uses first part of tuple for priority, tuple is (number, tuple) 
    priroity_queue.append((distance, curr_coord))

    visited.append(curr_coord)
    num_states_explored = 1
    
    while (len(priroity_queue) > 0) and not (maze.isObjective(curr_coord[0], curr_coord[1])):
        priroity_queue.sort(reverse=True)
        curr_coord = priroity_queue.pop()[1]

        for neighbor in maze.getNeighbors(curr_coord[0], curr_coord[1]):
            if neighbor not in visited:
                distance = manhattan_dist(neighbor, objective)
                priroity_queue.append((distance, neighbor))

                visited.append(neighbor)
                num_states_explored = num_states_explored + 1
                parent[neighbor] = curr_coord

    #solution function
    path = []
    while not (curr_coord == maze.getStart()):
        path.append(curr_coord)
        curr_coord = parent[curr_coord]
    
    path.append(curr_coord)
    path.reverse()
    
    return path, num_states_explored

def bfs(maze):
    frontier = []
    explored = set([])
    initialState = State(maze.getStart(), set([]))

    initialNode = Node(None, 0, 0, initialState)
    frontier.append(initialNode)
    currentNode = None
    
    objectives = set(maze.getObjectives())
    mstObj = copy.deepcopy(objectives)
    mstLookupTable = createPreComputedMST(mstObj)
    while (len(frontier) > 0):
        
        currentNode = frontier.pop(0)
        explored.add(currentNode)
        
        if(currentNode.isGoal(objectives)):
            break

        row = currentNode.state.current[0]
        col = currentNode.state.current[1]
        for neighbor in maze.getNeighbors(row, col):
            node = currentNode.addChild(neighbor, maze, objectives, mstLookupTable)
            if node not in explored and node not in frontier:
                frontier.append(node)

    return solution(currentNode, maze, len(explored))

def astar(maze):
    startTime = time.clock()
    
    frontier = []
    explored = set([])
    costLookUp = {}
    
    initialState = State(maze.getStart(), set([]))
    initialNode = Node(None, 0, 0, initialState)
    
    costLookUp[initialState] = 0
    heapq.heappush(frontier, initialNode)
    
    currentNode = None
    
    objectives = set(maze.getObjectives())
    
    astarPathsLookUp = createPreCopmutedAstarPaths(maze)
    print(astarPathsLookUp)
    mstLookUp = createPreComputedMST(objectives)
    
    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)
        # print("now exploring new node with cost of {}".format(currentNode.cost))
        # print("node's current pos is {}".format(currentNode.state.current))
        # print("node's current objectives gotten is {}".format(currentNode.state.objectives))
        # if not currentNode.parent is None:
        #     print("this node came from {}".format(currentNode.parent.state.current))
        # print(len(explored))
        if(currentNode.isGoal(objectives)):
            break

        row = currentNode.state.current[0]
        col = currentNode.state.current[1]
        for neighbor in maze.getNeighbors(row, col):
            node = currentNode.addChild(neighbor, maze, objectives, mstLookUp)
            if node not in explored and node not in frontier:
                heapq.heappush(frontier, node)
                costLookUp[node.state] = node.total_cost
            else:
                currentCost = node.total_cost
                oldCost = costLookUp[node.state]
                if(currentCost < oldCost):
                    costLookUp[node.state] = currentCost
                    if node in explored:
                        explored.remove(node)
                        frontier.append(node)
                    else:
                        frontier.append(node)
                        
    endTime = time.clock()
    print("total time took: {}".format(endTime-startTime))
    return solution(currentNode, maze, len(explored))

def createPreCopmutedAstarPaths(maze):
    startTime = time.clock()
    astarPathsLookUp = {}
    
    objectives = maze.getObjectives()
    objectives.append(maze.getStart())
    print("objectives are {}".format(objectives))
    for combo in combinations(objectives, 2):
        start, end = combo
        #path is from end -> start
        path, cost = astarStartEnd(maze, start, end)
        astarPathsLookUp[(end, start)] = (path, cost)
        astarPathsLookUp[(start, end)] = (path.reverse(), cost)
    endTime = time.clock()
    print("total time took for astar paths is: {}".format(endTime-startTime))
    return astarPathsLookUp
### function takes in all of the objectives
### creates every single possible combination and creates a MST
### stores this value in a dictionary to be looked up later in the heuristic
def createPreComputedMST(objectives):
    results = {}
    for x in range(len(objectives)):
        for combo in combinations(objectives,x+1):
            mstValue = createMST(combo)
            key = frozenset(copy.deepcopy(combo))
            results[key] = mstValue
    return results

## function takes in a set of vertices converts to a list
## creates a list of indices that corresponds to the index of a vertex in the list
## only did list of indices to work with Kruskal's MST
def createMST(vertices):
    graph = []
    vertexList = list(vertices)
    indexes = []

    for x in range(len(vertices)):
        indexes.append(x)
    edges = list(combinations(indexes, 2))
    
    for edge in edges:
        vertexA = vertexList[edge[0]]
        vertexB = vertexList[edge[1]]
        weight = manhattan_dist(vertexA, vertexB)
        addEdge(graph, edge[0], edge[1], weight)
    return KruskalMST(graph, len(vertices))

### function used just for one goal in astar
### used to precompute all paths to every combination of goals
### only returns paths from one direction end to start, and actual cost(should be just length)
def astarStartEnd(maze, start, end):
    frontier = []
    explored = set([])
    costLookUp = {}
    
    startNode = NodeOneGoal(None, 0, 0, start)
    
    costLookUp[startNode.state] = 0
    heapq.heappush(frontier, startNode)
    
    currentNode = None
    
    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)
    
        if(currentNode.isGoal(end)):
            break

        row = currentNode.state[0]
        col = currentNode.state[1]
        for neighbor in maze.getNeighbors(row, col):
            node = currentNode.addChild(neighbor, maze, end)
            if node not in explored and node not in frontier:
                heapq.heappush(frontier, node)
                costLookUp[node.state] = node.total_cost
            else:
                currentCost = node.total_cost
                oldCost = costLookUp[node.state]
                if(currentCost < oldCost):
                    costLookUp[node.state] = currentCost
                    if node in explored:
                        explored.remove(node)
                        frontier.append(node)
                    else:
                        frontier.append(node)
    cost = currentNode.actual_cost
    path = getPathOneGoal(currentNode)
    return path, cost

### only returns paths from one direction end to start
def getPathOneGoal(goalNode):
    currentNode = goalNode
    path = []
    curr_coord = currentNode.state
    while not (currentNode.parent is None):
        path.append(curr_coord)
        currentNode = currentNode.parent
        curr_coord = currentNode.state
    path.append(curr_coord)
    
    return path

def solution(goalNode, maze, statesExplored):
    currentNode = goalNode
    path = []
    curr_coord = currentNode.state.current
    objectivesTraveledTo = set([])
    while not (currentNode.parent is None):
        path.append(curr_coord)
        if(maze.isObjective(curr_coord[0], curr_coord[1])):
            objectivesTraveledTo.add(curr_coord)
        currentNode = currentNode.parent
        curr_coord = currentNode.state.current
    path.append(curr_coord)
            
    path.reverse()
    # print("path")
    # for coord in path:
    #     print(coord)
    return path, statesExplored

### function takes in all the objectives and a state 
### uses mstLookup based on state to estimate distance to all other goals
### distance to nearest goal + distance to all other goals
def heuristic(curr_coord, objectives, state, mstLookUp):
    distances = []
    
    visited = state.objectives
    unvisited = objectives - visited
    
    for objective in unvisited:
        dist = manhattan_dist(curr_coord, objective)
        distances.append((dist, objective))
    distances.sort(reverse=True)

    if(len(unvisited)==1):
        return distances.pop()[0]

    if(len(distances) > 0):
        mindist = distances.pop()
        return mindist[0] + mstLookUp[frozenset(unvisited)]

    return 0

def manhattan_dist(start, end):
    delta_row = start[0] - end[0]
    delta_col = start[1] - end[1]
    distance = abs(delta_row) + abs(delta_col)
    return distance

class Node:
    def __init__(self, parent, actual_cost, total_cost, state):
        self.parent = parent
        self.total_cost = total_cost
        self.actual_cost = actual_cost
        self.state = state

    def addChild(self, newCoord, maze, objectives, mstLookupTable):
        state = self.createState(newCoord, maze)
        actual_cost = self.actual_cost + 1
        total_cost = actual_cost + heuristic(newCoord, objectives, state, mstLookupTable)
        node = Node(self, actual_cost, total_cost, state)
        return node

    def createState(self, newCoord, maze):
        objectives = copy.deepcopy(self.state.objectives)
        if maze.isObjective(newCoord[0], newCoord[1]):
            objectives.add(newCoord)
        newState = State(newCoord, objectives)
        return newState

    def isGoal(self, objectives):
        return self.state.isGoal(objectives)

    def __lt__(self, other):
        if(self.total_cost == other.total_cost):
            return self.actual_cost > other.actual_cost
        return self.total_cost < other.total_cost

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash((self.state))

class State:
    def __init__(self, current, objectives):
        self.current = current
        self.objectives = objectives
        self.frozenobjectives = frozenset(objectives)

    def __eq__(self, other):
        if self.current == other.current:
            if len(self.objectives) == len(other.objectives):
                    if self.objectives == other.objectives:
                        return True
        return False

    def isGoal(self, objectives):
        if self.objectives == objectives:
            return True
        return False

    def __hash__(self):
        return hash((self.current, self.frozenobjectives))

class NodeOneGoal:
    def __init__(self, parent, actual_cost, total_cost, state):
        self.parent = parent
        self.total_cost = total_cost
        self.actual_cost = actual_cost
        self.state = state

    def addChild(self, newCoord, maze, goal):
        state = newCoord
        actual_cost = self.actual_cost + 1
        total_cost = actual_cost + manhattan_dist(newCoord, goal)
        node = NodeOneGoal(self, actual_cost, total_cost, state)
        return node

    def isGoal(self, goal):
        return self.state == goal

    def __lt__(self, other):
        if(self.total_cost == other.total_cost):
            return self.actual_cost > other.actual_cost
        return self.total_cost < other.total_cost

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash((self.state))

from collections import defaultdict

def addEdge(graph,u,v,w):
    graph.append([u,v,w])

# includes path compressions
def find(parent, i):
    if parent[i] == i:
        return i
    return find(parent, parent[i])

def union_by_rank(x, y, parent, rank):
    xroot = find(parent, x)
    yroot = find(parent, y)

    if rank[xroot] < rank[yroot]:
        parent[xroot] = yroot
    elif rank[xroot] > rank[yroot]:
        parent[yroot] = xroot
    else :
        parent[yroot] = xroot
        rank[xroot] += 1

def KruskalMST(graph, num_vertices):
    result_mst = []
    i = 0
    edgeCount = 0

    graph = sorted(graph,key=lambda item: item[2])
    parent = [] ; rank = []

    for node in range(num_vertices):
        parent.append(node)
        rank.append(0)
    totalWeight = 0

    while edgeCount < num_vertices - 1 :

        u,v,w = graph[i]
        i = i + 1
        x = find(parent, u)
        y = find(parent ,v)

        if x != y:
            edgeCount = edgeCount + 1
            result_mst.append([u,v,w])
            totalWeight += w
            union_by_rank(x, y, parent, rank)
    return totalWeight
        
        
            