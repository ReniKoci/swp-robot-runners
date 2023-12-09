
#Tree Node

class TreeNode:
    def __init__(self):
        self.constraints=[]     # List of constraints. A constraint is a tuple (a_i,v,t) where agent a_i is prohibited from occupying v at time step t.
        self.solution=[]        # A set of k paths, one for each agent. The path for agent must be consistent with the constraints of a_i
        self.cost=0             # The total cost of the current solution.

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

    # def updateSolution(self):    
    
    #CBS High Level

class HighLevelSolver:
    # Returns true if there is a conflict between two given routes in first n steps
    def hasCoflict(path1, path2, n = None):
        min_index = min(len(path1), len(path2), n)
        for i in range(min_index):
            if path1[i]==path2[i]:
                return True
        return False
        
    # Returns true if there is an edge conflict between two routes in first n steps
    def hasEdgeConflict(path1, path2, n=None):
        min_index = min(len(path1), len(path2), n)
        for i in range(min_index-1):
            if (path1[i]==path2[i+1] & path1[i+1]==path2[i]):
                 return True
        return False
    

    
        
