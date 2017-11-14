#Astar 3D
"""
#path finder
"""
#import math
#import collections
import time
import vpython
import heapq
import sys
import numpy as np
#try with this and see the shortest path
grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]

grid2 = [[0, 1, 0, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 0]]

grid4 = [[0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0],
        [1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0],
        [1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0],
        [0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0]]

grid5 = [[0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1],
        [0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1],
        [0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1],
        [1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0],
        [1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0],
        [1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]]

grid6 = [[0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1],
        [0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1],
        [0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1],
        [1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0],
        [1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0],
        [1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]]

grid3 = [[0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0],
        [0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0],
        [0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0],
        [0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0],
        [0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0]]

cost = 1
gvalue = 0
delta = [[-1,0], #up
         [0,-1], #left
         [1, 0], #down
         [0, 1]] #right

delta_name = ['^','<','v','>']

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
class graph:
    def __init__(self,height=0,width=0,landmarks=[0],grid=[0]):
        self.height = height
        self.width = width
        self.landmarks = []
        self.grid = grid
        self.graph = self.create_graph()
        
    
    def in_bounds(self, results):
        xs=[]
        ys=[]
        new_results=[]
        
        for i in range(len(results)):
            xs.append(results[i][0])
            ys.append(results[i][1])
        for x in range(len(xs)):
            if 0 <= xs[x] < self.height and 0 <= ys[x] < self.width:
                new_results.append((xs[x],ys[x]))
        return new_results
    
    def passable(self, results):
        new_results = []
        save = []
        for i in self.landmarks:
            for j in results:
                verify = (i[0]==j[0])
                verify2 = i[1]==j[1]
                if verify and verify2:
                    #print "catcha:",i
                    save.append(j)
        if len(save)==0:
            return results            
        for i in range(len(save)):
            if i>0:
                new_results = self.passable(new_results)
            if i==0:                
                for coord in range(len(results)):
                    verify = (results[coord][0]==save[i][0])
                    verify2 = (results[coord][1]==save[i][1])
                    if verify and verify2:
                        pass
                    else:
                        new_results.append(results[coord])        
        #print "new: ", new_results
        return new_results
        

    def create_graph(self):
        graph = {}
        for x in range(self.height):
            for y in range(self.width):
                id = (x, y)
                if self.grid[x][y] !=1:               
                    results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
                    results = self.in_bounds(results)
                    #print id,results
                    results = self.passable(results)
                    graph[id] = results
        
        return graph

    def show_graph(self):       
        for key in self.graph:
            print("%s=%s \n"%(str(key),str(self.graph[key])))

        
def slicer(grid):
    for i in range(len(grid)):
        if len(grid[i]) != len(grid[i-1]):
            print("Your grid is invalid")
            raise ValueError   
    height = len(grid)
    width = len(grid[0])
    landmarks = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] != 0:
                landmarks.append((i,j))
    return height,width,landmarks

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal,cost):
    expand = [[-1 for j in range(len(graph.grid[0]))] for i in range(len(graph.grid))]
    frontier = PriorityQueue()
    frontier.put(start, heuristic(goal,start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    expand[start[0]][start[1]] = 0
    count = 1
    try:
        
        while not frontier.empty():
            #print frontier.elements
            current = frontier.get()
            #print frontier.elements
            if current == goal:
                expand[current[0]][current[1]] = count
                break

            for key in graph.graph:
                verify = (key == current)
                if verify:
                    key_1 = graph.graph[key]

            for next in key_1:
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:    
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
                    if next != goal:
                        expand[next[0]][next[1]] = count #para ver la expansion general cambiar current por next
                        count += 1
    except Exception as e:
        print(sys.exc_info()[0],sys.exc_info()[1],sys.exc_info()[2].tb_lineno)
                
    print("expand map:")            
    for i in range(len(expand)):
        print("%s\n"%(expand[i]))
    print(" ")
    print("camefrom=%s\n,cost_so_far=%s"%(came_from, cost_so_far))
    print(" ")
    for key in cost_so_far:
        verify = (key[0] == goal[0])
        verify2 = (key[1] == goal[1])
        if verify and verify2:
            winner = [cost_so_far[key],goal[0],goal[1]]
            return winner, came_from, cost_so_far
        
def reconstruct_path(came_from, start, goal,grid):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    #path.append(start) # optional
    path.reverse() # optional
    #draw the grid
    draw = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    for way in range(len(path)):
        for i in range(len(draw)):
            for j in range(len(draw[i])):
                verify = (i == path[way][0])
                verify2 = (j == path[way][1])
                if verify and verify2:
                    draw[i][j]= '*'                 
    #map
    for i in range(len(draw)):       
        print("%s\n"%(draw[i]))
    print(" ")
    return path

#this will smooth the path found by A*
def smooth(path, weight_data = 0.1, weight_smooth = 0.1, 
           tolerance = 0.000001):
    if path == []:
        print("Run A* first before smoothing path")
        raise ValueError
        
    #making a copy of path original
    spath = [[0 for row in range(len(path[0]))] for col in range(len(path))]
    for i in range(len(path)):
        for j in range(len(path[0])):
            spath[i][j] = path[i][j]
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1):
            for j in range(len(path[0])):
                aux = spath[i][j]
            
                spath[i][j] += weight_data *(path[i][j] - spath[i][j])                   
                spath[i][j] += weight_smooth *(spath[i-1][j] + spath[i+1][j] - (2.0 * spath[i][j]))
                if i >= 2:
                    spath[i][j] += 0.5 * weight_smooth *(2.0 * spath[i-1][j] - spath[i-2][j]-spath[i][j])
                if i <= len(path) - 3:
                    spath[i][j] += 0.5 * weight_smooth *(2.0 * spath[i+1][j] - spath[i+2][j]- spath[i][j])
                change += abs(aux - spath[i][j])
    return spath

def check_goal(pathway_now,pathway_final):
    if pathway_now ==pathway_final:
        return True
    return False

def representer(grid,path_way,clock,landmarks):
    z =len(grid)*10
    x=len(grid[0])*10
    escena = vpython.canvas(width=1200,height=600)
    escena.title = "Path Finder usando el algoritmo A*"
    escena.background = vpython.color.cyan
    escena.center = vpython.vector(x/2,-1,z/2)
    #escena.forward= vpython.vector(1,-1,0.2)
    escena.forward= vpython.vector(0,-2,-1)
    
    #ejex= vpython.curve(color=vpython.color.blue)
    #ejey=vpython.curve(color=vpython.color.red)
    #ejez=vpython.curve(color=vpython.color.green)
    #vpython.userzoom = True
    #for i in range(x):
    #    pos_x = vpython.vector(i,0,0)
    #    ejex.append(pos_x)
    #    pos_y = vpython.vector(0,i,0)
    #    ejey.append(pos_y)
    #    pos_z=vpython.vector(0,0,i)
    #    ejez.append(pos_z)
    
    suelo = vpython.box(pos=vpython.vector(x/2,-1,z/2),color=vpython.color.yellow,size=vpython.vector(x,0.1,z),texture=vpython.textures.rough)
    high_wall = 20 
    for i in range(len(landmarks)):
        z_wall=(landmarks[i][0]*10)+5
        x_wall=(landmarks[i][1]*10)+5
        muro = vpython.box(pos=vpython.vector(x_wall,10,z_wall),color=vpython.color.white,size=vpython.vector(10,high_wall,10),texture=vpython.textures.wood)
        
    robot = vpython.sphere(pos=vpython.vector(path_way[0][1]+5,3,path_way[0][0]+5),color=vpython.color.black,radius=3)
    trayectoria = vpython.curve(color=vpython.color.white)
    inicio =vpython.text(text="Inicio",align='center',color=vpython.color.green,pos=vpython.vector(5,22,5),height=5,width=12)
    meta = vpython.text(text="Meta",align='center',color=vpython.color.green,pos=vpython.vector((path_way[-1][1]*10)+5,22,(path_way[-1][0]*10)+5),height=5,width=12)
    time.sleep(clock)
    escena.range=150
    for i in range(len(path_way)):
        #escena.center = vpython.vector((path_way[i][1]*10)+5,-100,(path_way[i][0]*10)+5-20)
        escena.center = vpython.vector((path_way[i][1]*10)+5,4,(path_way[i][0]*10)+5)
        #time.sleep(0.3)
        time.sleep(0.1)
        pos = vpython.vector((path_way[i][1]*10)+5,3,(path_way[i][0]*10)+5)
        robot.pos = pos
        trayectoria.append(pos)
        #vpython.rate(100)
        vpython.rate(300)
        
        
def main():
    grapho = graph()
    grapho2 = graph()
    grapho3 = graph()
    grapho4 = graph()
    grapho5 = graph()
    grapho6 = graph()
    graphos = [grapho,grapho2,grapho3,grapho4,grapho5,grapho6]
    grids =[grid,grid2,grid3,grid4,grid5,grid6]
    for i in range(len(grids)):
        height,width,landmarks = slicer(grids[i])
        graphos[i] = graph(height,width,landmarks,grids[i])
        graphos[i].show_graph()
        print(" ")
        #print grapho.graph
        try:
            start = (0,0)
            goal = (len(grids[i])-1,len(grids[i][0])-1)
            winner, came_from, cost_so_far = a_star_search(graphos[i], start, goal,cost)
            print("costo total del camino mas corto")
            print(winner)
            print("El camino que debe tomar:")
            path_way =reconstruct_path(came_from, start, goal,grids[i])
            print(path_way)
            path_way = smooth(path_way)
            clock=2
            representer(grids[i],path_way,clock,landmarks)
        except Exception as e:
            print("fail",str(e),sys.exc_info()[2].tb_lineno)
            sys.exit(0)

        
        if i<len(grids)-1:
            input("\nClick para el siguiente...")
    print("End")
    
if __name__=="__main__":
    main()