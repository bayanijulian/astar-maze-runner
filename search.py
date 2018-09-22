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
import itertools
import copy
import heapq
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

def old(maze):
    path = []
    objectives = maze.getObjectives()
    num_states_explored = 1
    curr_coord = maze.getStart()
    path.append(curr_coord)

    # perms = list(itertools.permutations(objectives,2))
    # print((perms))

    # return [], 0
    while len(objectives) > 0:
        priroity_queue = []
        visited = []
        parent = {}
        
        cost = 0
        objective = get_next_objective(curr_coord,objectives)
        objectives.remove(objective)
        distance = manhattan_dist(curr_coord, objective)

        #uses first part of tuple for priority, tuple is (number, tuple) 
        priroity_queue.append((distance, curr_coord))
        visited.append(curr_coord)
        start_coord = curr_coord
        while (len(priroity_queue) > 0) and not (objective == curr_coord):
            priroity_queue.sort(reverse=True)
            curr_coord = priroity_queue.pop()[1]

            for neighbor in maze.getNeighbors(curr_coord[0], curr_coord[1]):
                if neighbor not in visited:
                    distance = manhattan_dist(neighbor, objective)
                    cost = cost + 1
                    priroity_queue.append((distance + cost, neighbor))
                    visited.append(neighbor)
                    num_states_explored = num_states_explored + 1
                    parent[neighbor] = curr_coord

        #solution function
        subpath = []
        while not (curr_coord == start_coord):
            subpath.append(curr_coord)
            curr_coord = parent[curr_coord]
        
        subpath.reverse()
        path = path + subpath
        curr_coord = path[-1]#resets curr_coord
    return path, num_states_explored


def astar(maze):
    frontier = []
    explored = set([])
    
    initialState = State(maze.getStart(), set([]))
    initialNode = Node(None, 0, initialState)
    
    heapq.heappush(frontier, initialNode)
    currentNode = None
    
    objectives = set(maze.getObjectives())

    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)
        # print("now exploring new node with cost of {}".format(currentNode.cost))
        # print("node's current pos is {}".format(currentNode.state.current))
        # print("node's current objectives gotten is {}".format(currentNode.state.objectives))
        # if not currentNode.parent is None:
        #     print("this node came from {}".format(currentNode.parent.state.current))
        #print(len(explored))
        if(currentNode.isGoal(objectives)):
            break

        row = currentNode.state.current[0]
        col = currentNode.state.current[1]
        for neighbor in maze.getNeighbors(row, col):
            node = currentNode.addChild(neighbor, maze, objectives)
            if node not in explored and node not in frontier:
                heapq.heappush(frontier, node)

    #solution function
    path = []
    curr_coord = currentNode.state.current
    # print("objectives")
    # for obj in currentNode.state.objectives:
    #     print(obj)
    objectivesTraveledTo = set([])
    while not (currentNode.parent is None):
        path.append(curr_coord)
        if(maze.isObjective(curr_coord[0], curr_coord[1])):
            objectivesTraveledTo.add(curr_coord)
        currentNode = currentNode.parent
        curr_coord = currentNode.state.current
    path.append(curr_coord)
            
    path.reverse()
    print("path")
    for coord in path:
        print(coord)
    return path, len(explored)

def heuristic(curr_coord, objectives):
    distances = []
    distanceTotal = 0
    for objective in objectives:
        dist = manhattan_dist(curr_coord, objective)
        distanceTotal += dist
        distances.append(dist)
    distances.sort(reverse=True)
    return distances.pop()
    #return distanceTotal/len(objectives)

def manhattan_dist(curr_coord, objective):
    delta_row = curr_coord[0] - objective[0]
    delta_col = curr_coord[1] - objective[1]
    dist = abs(delta_row) + abs(delta_col)
    return dist

def get_next_objective(curr_coord, objectives):
    priroity_queue = []
    for objective in objectives:
        dist = manhattan_dist(curr_coord, objective)
        priroity_queue.append((dist, objective))
    priroity_queue.sort(reverse=True)
    return priroity_queue.pop()[1]

class Node:
    def __init__(self, parent, cost, state):
        self.parent = parent
        self.cost = cost
        self.state = state

    def addChild(self, newCoord, maze, objectives):
        state = self.createState(newCoord, maze)
        cost = self.cost + 1 + heuristic(newCoord, objectives)
        node = Node(self, cost, state)
        return node

    def createState(self, newCoord, maze):
        objectives = copy.deepcopy(self.state.objectives)
        if maze.isObjective(newCoord[0], newCoord[1]):
            objectives.add(newCoord)
            #print("found objective")
            #print(newCoord)
        newState = State(newCoord, objectives)
        return newState

    def isGoal(self, objectives):
        return self.state.isGoal(objectives)

    def __lt__(self, other):
        return self.cost < other.cost

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
        if len(self.objectives) == len(objectives):
            if self.objectives == objectives:
                return True
        return False
    def __hash__(self):
        return hash((self.current, self.frozenobjectives))
        
            