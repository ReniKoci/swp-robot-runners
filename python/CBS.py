from multipledispatch import dispatch   #for overloading
import MAPF
import debugpy
from typing import Dict, List, Tuple, Set
from queue import PriorityQueue
import numpy as np
from enum import Enum

# Vertex Conflict is a tuple (a_i, a_j, v, v, t) 
# where agent a_i and a_j occupy vertex v at time point t. 
# In this case vertex_1 = vertex_2

# Edge Conflict is a tuple (a_i, a_j, v1, v2, t) 
# where two agents swap locations beetwen time step t to time step t+1: 
# a_i moves from v1 to v2, while a_j moves from v2 to v1.
class Conflict:
    def __init__(self, a_i, a_j, v_1,v_2, t):
        self.first_agent_id = a_i
        self.second_agent_id = a_j
        self.vertex_1 = v_1
        self.vertex_2=v_2
        self.time = t

# A constraint is a tuple (a_i, v, t) where agent a_i is prohibited from occupying vertex v at time step t.
class Constraint:
    def __init__(self, a_i, v, t ):
        self.agent_id = a_i
        self.position = v
        self.time = t

#Tree Node

class TreeNode:
    def __init__(self):
        self.constraints= []   # List of constraints. A constraint is a tuple (a_i,v,t) where agent a_i is prohibited from occupying v at time step t.
        self.solution=[]      # A set of k paths, one for each agent. The path for agent must be consistent with the constraints of a_i
        self.cost=0            # The total cost of the current solution.

    def getSolution(self):
        return self.solution
    
    def getCost(self):
        return self.cost
    
    def getConstraints(self):
        return self.constraints
    
    def addConstraint(self, constraint):
        self.constraints.append(constraint)
    
    def updateCost(self):
        new_cost = sum(len(path) for path in self.solution)
        self.cost = new_cost

    # def updateSolution(self):    LowLevelSolver
   
class HighLevelSolver:  #CBS High Level
    
    # Returns true if there is a conflict between two given routes (in first n time steps) 
    @dispatch(list, list, int)
    def hasConflict(path1, path2, n):
        min_index = min(len(path1), len(path2), n)
        for i in range(min_index):
            if path1[i]==path2[i]:
                return True
        return False
    
    @dispatch(list, list)
    def hasConflict(path1, path2):
        min_index = min(len(path1), len(path2))
        for i in range(min_index):
            if path1[i]==path2[i]:
                return True
        return False
    

    # Returns true if there is a conflict between two any routes (in first n time steps) in Node of CT (constraint tree)
    @dispatch(TreeNode, int)
    def hasConflictNode(node: TreeNode, n):
        solutions = node.getSolution()
        for i in range (len(solutions)):
            for j in range(i+1, len(solutions)):
                if HighLevelSolver.hasConflict(solutions[i], solutions[j], n):
                    return True
        return False

    @dispatch(TreeNode)
    def hasConflictNode(node : TreeNode):
        solutions = node.getSolution()
        for i in range (len(solutions)):
            for j in range(i+1, len(solutions)):
                if HighLevelSolver.hasConflict(solutions[i], solutions[j]):
                    return True
        return False
        
    # Returns true if there is an edge conflict between two routes (in first n time steps)
    @dispatch(list, list, int)
    def hasEdgeConflict(path1, path2, n):
        min_index = min(len(path1), len(path2), n)
        for i in range(min_index-1):
            if (path1[i]==path2[i+1] and path1[i+1]==path2[i]):
                 return True
        return False
    
    @dispatch(list, list)
    def hasEdgeConflict(path1, path2):
        min_index = min(len(path1), len(path2))
        for i in range(min_index-1):
            if (path1[i]==path2[i+1] and path1[i+1]==path2[i]):
                 return True
        return False
    
    # Returns true if there is an edge conflict between any two routes (in first n time steps) in Node of CT (constraint tree)
    @dispatch(TreeNode, int)
    def hasEdgeConflictNode(node: TreeNode, n):
        solutions = node.getSolution()
        for i in range (len(solutions)):
            for j in range(i+1, len(solutions)):
                if HighLevelSolver.hasEdgeConflict(solutions[i], solutions[j], n):
                    return True
        return False
    
    @dispatch(TreeNode)
    def hasEdgeConflictNode(node: TreeNode):
        solutions = node.getSolution()
        for i in range (len(solutions)):
            for j in range(i+1, len(solutions)):
                if HighLevelSolver.hasEdgeConflict(solutions[i], solutions[j]):
                    return True
        return False
    
                    
    # Look for the first vertex or edge conflict (in first n time steps) 
    # surch for a_1, a_2, a_3 and so on

    @dispatch(TreeNode, int)
    def getConflict(node: TreeNode, n):
        solutions = node.getSolution()
        
        for i in range (len(solutions)):
             for j in range(i+1, len(solutions)):
                min_index = min(len(solutions[i]), len(solutions[j]), n)
                for t in range(min_index-1):
                    if solutions[i][t]==solutions[j][t]: #look for vertex conflict
                        return Conflict(i, j, solutions[i][t], solutions[j][t], t)
                    if (solutions[i][t]==solutions[j][t+1] and solutions[i][t+1]==solutions[j][t]): #look for edge conflict
                        return Conflict(i, j, solutions[i][t], solutions[j][t], t)
                t = min_index-1
                if solutions[i][t]==solutions[j][t]: #look for vertex conflict in last time step
                    return Conflict(i, j, solutions[i][t], solutions[j][t], t)
                
    @dispatch(TreeNode)
    def getConflict(node: TreeNode):
        solutions = node.getSolution()
        
        for i in range (len(solutions)):
             for j in range(i+1, len(solutions)):
                min_index = min(len(solutions[i]), len(solutions[j]))
                for t in range(min_index-1):
                    if solutions[i][t]==solutions[j][t]: #look for vertex conflict
                        return Conflict(i, j, solutions[i][t], solutions[j][t], t)
                    if (solutions[i][t]==solutions[j][t+1] and solutions[i][t+1]==solutions[j][t]): #look for edge conflict
                        return Conflict(i, j, solutions[i][t], solutions[j][t], t)
                t = min_index-1
                if solutions[i][t]==solutions[j][t]: #look for vertex conflict in last time step
                    return Conflict(i, j, solutions[i][t], solutions[j][t], t)
                    
    
#  #Look for the vertex conflict first. Look for the edge conflict if there is no vertex conflicts

#     def getConflict2(node: TreeNode):
#         solutions = node.getSolution()
#         #look for vertex conflict
#         for i in range (len(solutions)):
#              for j in range(i+1, len(solutions)):
#                 min_index = min(len(solutions[i]), len(solutions[j]))
#                 for t in range(min_index):
#                     if solutions[i][t]==solutions[j][t]:
#                         return Conflict(i, j, solutions[i][t], solutions[j][t], t) # in this case solutions[i][t] = solutions[j][t]
            
#         #look for edge conflict
#         for i in range (len(solutions)):
#              for j in range(i+1, len(solutions)):
#                 min_index = min(len(solutions[i]), len(solutions[j]))
#                 for t in range(min_index-1):
#                     if (solutions[i][t]==solutions[j][t+1] and solutions[i][t+1]==solutions[j][t]):
#                         return Conflict(i, j, solutions[i][t], solutions[j][t], t)



    



# TEST

# N = TreeNode()
# N.cost = 0
# N.solution =[[1,2,3], [4,5,6,7], [8,9,10,11],[12,13,11,10]]
# N.constraints = []

# print(HighLevelSolver.hasConflict([1,2,3],[5,6,3],2)) 
# print(HighLevelSolver.hasConflict([1,2,3],[5,6,3])) 

# print(N.cost)
# print(N.getCost())
# print(N.solution)
# print(N.getSolution())
# print(N.constraints)
# print(N.getConstraints())

# print("Has conflict in Node?")
# print(HighLevelSolver.hasConflictNode(N,2))
# print(HighLevelSolver.hasConflictNode(N))

# print("Has Edge conflict in Node?")
# print(HighLevelSolver.hasEdgeConflictNode(N,3))
# print(HighLevelSolver.hasEdgeConflictNode(N))


# test = HighLevelSolver.getConflict(N)
# print(test.first_agent_id)
# print(test.second_agent_id)
# print(test.vertex_1)
# print(test.vertex_2)
# print(test.time)
# print("--------------------")

# test = HighLevelSolver.getConflict(N, 3)
# if test != None:
#     print(test.first_agent_id)
#     print(test.second_agent_id)
#     print(test.vertex_1)
#     print(test.vertex_2)
#     print(test.time)

# print("--------------------")
# test = HighLevelSolver.getConflict2(N)
# print(test.first_agent_id)
# print(test.second_agent_id)
# print(test.vertex_1)
# print(test.vertex_2)
# print(test.time)
