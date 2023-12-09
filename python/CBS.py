from multipledispatch import dispatch   #for overloading
import MAPF
import debugpy
from typing import Dict, List, Tuple, Set
from queue import PriorityQueue
import numpy as np
from enum import Enum


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
    
    # Returns true if there is a conflict between two given routes (in first n steps) 
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
    

    # Returns true if there is a conflict between two any routes (in first n steps) in Node of CT (constraint tree)
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
        
    # Returns true if there is an edge conflict between two routes (in first n steps)
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
    
    # Returns true if there is an edge conflict between any two routes (in first n steps) in Node of CT (constraint tree)
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
    



# TEST

# N = TreeNode()
# N.cost = 0
# N.solution =[[1,6,7], [8,1,1,15], [9,7,8,15],[0,10,15,8]]
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
 
