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
from collections import defaultdict
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

def bfs(maze):
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
    path = []
    while not (curr_coord == maze.getStart()):
        path.append(curr_coord)
        curr_coord = parent[curr_coord]
    path.append(curr_coord)
    path.reverse()
    return path, num_states_explored

def greedy(maze):
    frontier = []
    explored = set([])
    
    # only gets one goal
    start = maze.getStart()
    goal = maze.getObjectives()[0]

    #parent, state, path_cost, estimated_cost, goal
    startNode = NodeGreedy(None, start, 0, 0, goal)
    heapq.heappush(frontier, startNode)
    currentNode = None
    
    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)
    
        if(currentNode.isGoal()):
            break

        row = currentNode.state[0]
        col = currentNode.state[1]
        for neighbor in maze.getNeighbors(row, col):
            node = currentNode.addChild(neighbor)
            if node not in explored and node not in frontier:
                heapq.heappush(frontier, node)
   
    path = currentNode.get_solution()
    return path, len(explored)

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
    
    astarPathLookUp = createPrecomputedAstarPaths(maze)
    mstLookUp = createPreComputedMST(objectives, astarPathLookUp)
    endTimePrecomp = time.clock()
    print("total time took for precomputations: {}".format(endTimePrecomp-startTime))
    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)
        
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
                        
    endTimeTotal = time.clock()
    print("total time to finish: {}".format(endTimeTotal-startTime))
    return solution(currentNode, maze, len(explored))

def createPrecomputedAstarPaths(maze):
    astarPathsLookUp = {}
    
    objectives = maze.getObjectives()
    objectives.append(maze.getStart())
    
    for combo in combinations(objectives, 2):
        start, end = combo
        cost = astarPathCost(maze, start, end)
        
        astarPathsLookUp[(end, start)] = cost
        astarPathsLookUp[(start, end)] = cost
    
    return astarPathsLookUp

### function takes in all of the objectives
### creates every single possible combination and creates a MST
### stores this value in a dictionary to be looked up later in the heuristic
def createPreComputedMST(objectives, astarLookUp):
    results = {}
    for x in range(len(objectives)):
        for combo in combinations(objectives,x+1):
            mstValue = createMST(combo, astarLookUp)
            key = frozenset(copy.deepcopy(combo))
            results[key] = mstValue
    return results

## function takes in a set of vertices converts to a list
## creates a list of indices that corresponds to the index of a vertex in the list
## only did list of indices to work with Kruskal's MST, where the indices is what makes the graph
def createMST(vertices, weightsLookUp):
    graph = []
    vertexList = list(vertices)
    indexes = []

    for x in range(len(vertices)):
        indexes.append(x)
    edges = list(combinations(indexes, 2))
    
    for edge in edges:
        vertexA = vertexList[edge[0]]
        vertexB = vertexList[edge[1]]
        #weight = manhattan_dist(vertexA, vertexB)
        weight = weightsLookUp[(vertexA, vertexB)]
        graph.append([edge[0],edge[1],weight])

    return getKruskalMSTCost(graph, len(vertices))

### function used just for one goal in astar
### used to precompute all paths to every combination of goals
### and actual cost(should be just length)
def astarPathCost(maze, start, end):
    frontier = []
    explored = set([])
    costLookUp = {}
    # self, parent, state, path_cost, estimated_cost, goal
    startNode = NodeAstar(None, start, 0, 0, end)
    
    costLookUp[startNode.state] = 0
    heapq.heappush(frontier, startNode)
    
    currentNode = None
    
    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)
    
        if(currentNode.is_goal()):
            break

        row = currentNode.state[0]
        col = currentNode.state[1]
        for neighbor in maze.getNeighbors(row, col):
            node = currentNode.add_child(neighbor)
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
                        
    path_cost = currentNode.path_cost
    return path_cost

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
    return path, statesExplored

### function takes in all the objectives and a state 
### uses mstLookup based on state to estimate distance to all other goals
### distance to nearest goal + distance to all other goals
def heuristic(curr_coord, objectives, state, mstLookUp):
    distances = []
    
    visited = state.objectives
    unvisited = objectives - visited
    
    for objective in unvisited:
        dist = manhattan_distance(curr_coord, objective)
        distances.append((dist, objective))
    distances.sort(reverse=True)

    if(len(unvisited)==1):
        return distances.pop()[0]

    if(len(distances) > 0):
        mindist = distances.pop()
        return mindist[0] + mstLookUp[frozenset(unvisited)]
    return 0

def manhattan_distance(start, end):
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

# includes path compressions
def find(parent, vertex):
    if parent[vertex] == vertex:
        return vertex
    return find(parent, parent[vertex])

# used to make sure there are no loops
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

def getKruskalMSTCost(graph, num_vertices):
    
    graph = sorted(graph,key=lambda item: item[2])
    parent = [] ; rank = [] # result_mst = []

    for x in range(num_vertices):
        parent.append(x)
        rank.append(0)

    index = 0
    edge_counter = 0
    total_weight = 0

    while edge_counter < num_vertices - 1 :

        u,v,w = graph[index]
        index += 1
        x = find(parent, u)
        y = find(parent ,v)

        if x != y:
            edge_counter += 1
            # result_mst.append([u,v,w])
            total_weight += w
            union_by_rank(x, y, parent, rank)

    return total_weight

class SimpleNode:
    ### only works for start position to goal position
    ### state is the current position as tuple (row, col)
    ### path cost is path cost from start to this state
    def __init__(self, parent, state, path_cost, goal):
        self.parent = parent
        self.state = state
        self.goal = goal
        self.path_cost = path_cost
    
    def is_goal(self):
        return self.state == self.goal

    def get_solution(self):
        current_node = self
        path = []
        current_position = current_node.state
        
        # stops at start node which has parent set to None
        while not (current_node.parent is None):
            path.append(current_position)
            current_node = current_node.parent
            current_position = current_node.state
        # appends the starts node position
        path.append(current_position)
        
        path.reverse()
        return path

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash(self.state)

class NodeGreedy(SimpleNode):
    ### only works for start position to goal position
    ### state is the current position
    ### cost is estimated cost from current position to goal
    ### goal is objective position
    def __init__(self, parent, state, path_cost, estimated_cost, goal):
        SimpleNode.__init__(self, parent, state, path_cost, goal)
        self.estimated_cost = estimated_cost
        
    def __lt__(self, other):
        return self.estimated_cost < other.estimated_cost

    def add_child(self, position):
        estimated_cost = manhattan_distance(position, self.goal)
        path_cost = self.path_cost + 1
        node = NodeGreedy(self, position, path_cost, estimated_cost, self.goal)
        return node

class NodeAstar(NodeGreedy):
    
    ### actual cost is the cost it took from the start to this position
    ### total cost is estimated cost from current position to goal + actual cost
    ### goal is objective position
    def __init__(self, parent, state, path_cost, estimated_cost, goal):
        NodeGreedy.__init__(self, parent, state, path_cost, estimated_cost, goal)
        self.total_cost = path_cost + estimated_cost

    def add_child(self, postion):
        state = postion
        path_cost = self.path_cost + 1
        estimated_cost = manhattan_distance(postion, self.goal)
        node = NodeAstar(self, state, path_cost, estimated_cost, self.goal)
        return node
    
    def __lt__(self, other):
        if(self.total_cost == other.total_cost):
            return self.path_cost > other.path_cost
        return self.total_cost < other.total_cost