# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018
import collections
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
    distance = heuristic(curr_coord, objective)

    #uses first part of tuple for priority, tuple is (number, tuple) 
    priroity_queue.append((distance, curr_coord))

    visited.append(curr_coord)
    num_states_explored = 1
    
    while (len(priroity_queue) > 0) and not (maze.isObjective(curr_coord[0], curr_coord[1])):
        priroity_queue.sort(reverse=True)
        curr_coord = priroity_queue.pop()[1]

        for neighbor in maze.getNeighbors(curr_coord[0], curr_coord[1]):
            if neighbor not in visited:
                distance = heuristic(neighbor, objective)
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

def astar(maze):
    priroity_queue = []
    visited = []
    parent = {}
    
    #assuming only one objective for now
    objective = maze.getObjectives()[0]
    curr_coord = maze.getStart()
    distance = heuristic(curr_coord, objective)

    #uses first part of tuple for priority, tuple is (number, tuple) 
    priroity_queue.append((distance, curr_coord))

    visited.append(curr_coord)
    num_states_explored = 1
    
    while (len(priroity_queue) > 0) and not (maze.isObjective(curr_coord[0], curr_coord[1])):
        priroity_queue.sort(reverse=True)
        curr_coord = priroity_queue.pop()[1]

        for neighbor in maze.getNeighbors(curr_coord[0], curr_coord[1]):
            if neighbor not in visited:
                distance = heuristic(neighbor, objective)
                cost = num_states_explored #since cost = 1 for each step, cost = num_states_explored

                priroity_queue.append((distance + cost, neighbor))
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

def heuristic(curr_coord, objective):
    delta_row = curr_coord[0] - objective[0]
    delta_col = curr_coord[1] - objective[1]
    manhattan_distance = abs(delta_row) + abs(delta_col)
    return manhattan_distance
