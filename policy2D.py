# -*- coding: utf-8 -*-
"""
Created on Thu Jul 13 09:55:13 2017
#Optimum policy 2D
dynamic Programming:
    give us a plan for each position
@author: Eduardo
"""
forward = [[-1,0],  #up
           [0,-1],  #left
           [1, 0],  #down
           [0, 1]]  #right

forward_name = ['up','left','down','right']
#action has 3 values: right,turn, no turn,left turn
action= [-1,0,1]
action_name = ['R','#','L']

grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4,3,0] #row,col,direction
goal = [2,0] #row,col
cost = [2,1,20] #righ,no turn, left

def optimum_policy2D(grid,init,goal,cost):
    #code
    policy2D = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    value = [[[999 for row in range(len(grid[0]))] for col in range(len(grid))],
               [[999 for row in range(len(grid[0]))] for col in range(len(grid))],             
                 [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
                 [[999 for row in range(len(grid[0]))] for col in range(len(grid))]]
    
    policy = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
                [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
                [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
                  [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]
    change = True
    while change==True:
        change=False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(len(forward)):
                    if x == goal[0] and y == goal[1]:
                        if value[orientation][x][y] > 0:
                            value[orientation][x][y] = 0
                            policy[orientation][x][y] = '*'
                            change = True
                            
                    elif grid[x][y] == 0:
                        for i in range(len(action)):
                            o2 = (orientation + action[i]) % 4
                            x2 = x + forward[o2][0]
                            y2 = y + forward[o2][1]
                            if x2 >=0 and x2 < len(grid) and y2>=0 and y2<len(grid[0]) and grid[x2][y2]==0:
                                v2 = value[o2][x2][y2] + cost[i]
                                if v2 < value[orientation][x][y]:
                                    change = True #Si quitamos esto toma otro camino
                                    value[orientation][x][y] = v2
                                    policy[orientation][x][y] = action_name[i]
    x = init[0]
    y = init[1]
    orientation = init[2]
    policy2D[x][y] = policy[orientation][x][y]
    while policy[orientation][x][y] != '*':
        if policy[orientation][x][y] == '#':
            o2 = orientation
        elif policy[orientation][x][y] == 'R':
            o2 = (orientation -1) % 4
        elif policy[orientation][x][y] == 'L':
            o2 = (orientation +1) % 4
        x = x + forward[o2][0]
        y = y + forward[o2][1]
        orientation = o2
        policy2D[x][y] = policy[orientation][x][y]
    #for i in range(len(policy)):
    #    for way in range(len(policy[0])):
    #        print("%s"%(policy[i][way]))
    #    print(' ')
    #print(" ")
    for i in range(len(policy2D)):
        print("%s"%(policy2D[i]))
    return policy2D

optimum_policy2D(grid,init,goal,cost)
input()