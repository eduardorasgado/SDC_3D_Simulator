#Selfdriving_simulador3Dv2
#Integrated robot 3D
from math import *
import random
from copy import deepcopy
import sys
import time
import vpython

# don't change the noise paameters

steering_noise    = 0.1
distance_noise    = 0.03
measurement_noise = 0.3

class plan:

    # --------
    # init: 
    #    creates an empty plan
    #

    def __init__(self, grid, init, goal, cost = 1):
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.spath = []

    # --------
    #
    # make heuristic function for a grid
        
    def make_heuristic(self, grid, goal, cost):
        self.heuristic = [[0 for row in range(len(grid[0]))] 
                          for col in range(len(grid))]
        for i in range(len(self.grid)):    
            for j in range(len(self.grid[0])):
                self.heuristic[i][j] = abs(i - self.goal[0]) + abs(j - self.goal[1])



    # ------------------------------------------------
    # 
    # A* for searching a path to the goal
    #
    #

    def astar(self):


        if self.heuristic == []:
            print("Heuristic must be defined to run A*")
            raise ValueError

        # internal motion parameters
        delta = [[-1,  0], # go up
                 [ 0,  -1], # go left
                 [ 1,  0], # go down
                 [ 0,  1]] # do right


        # open list elements are of the type: [f, g, h, x, y]

        closed = [[0 for row in range(len(self.grid[0]))] 
                  for col in range(len(self.grid))]
        action = [[0 for row in range(len(self.grid[0]))] 
                  for col in range(len(self.grid))]

        closed[self.init[0]][self.init[1]] = 1


        x = self.init[0]
        y = self.init[1]
        h = self.heuristic[x][y]
        g = 0
        f = g + h
        

        open = [[f, g, h, x, y]]

        found  = False # flag that is set when search complete
        resign = False # flag set if we can't find expand
        count  = 0
        #came_from = {}

        while not found and not resign:

            # check if we still have elements on the open list
            if len(open) == 0:
                resign = True
                print('###### Search terminated without success')
                
            else:
                # remove node from list
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]

            # check if we are done

            if x == goal[0] and y == goal[1]:
                found = True
                # print '###### A* search successful'

            else:
                # expand winning element and add to new open list
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]):
                        if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                            g2 = g + self.cost
                            h2 = self.heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i
                            #came_from[(x2,y2)] = (x,y) 
        
            count += 1
        #print(came_from)
        try:
            # extract the path
            x = self.goal[0]
            y = self.goal[1]
            current=[x,y]
            invpath = [current]
            while x != self.init[0] or y != self.init[1]:
                #current = came_from[current]
                #invpath.append(current)
                x2 = x - delta[action[x][y]][0]
                y2 = y - delta[action[x][y]][1]
                x = x2
                y = y2
                invpath.append([x, y])
        except Exception as e:
            print(str(e),"Error!!!!!!!!!!!!!!!!")

        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])
        
        return self.path
    
    # ------------------------------------------------
    # 
    # this is the smoothing function
    #this will smooth the path found by A*
    def smooth(self, weight_data = 0.1, weight_smooth = 0.1, 
               tolerance = 0.000001):

        if self.path == []:
            print("Run A* first before smoothing path")
            raise ValueError
            
        #making a copy of path original
        self.spath = [[0 for row in range(len(self.path[0]))] for col in range(len(self.path))]
        for i in range(len(self.path)):
            for j in range(len(self.path[0])):
                self.spath[i][j] = self.path[i][j]

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(self.path)-1):
                for j in range(len(self.path[0])):
                    aux = self.spath[i][j]
                    
                    self.spath[i][j] += weight_data *(self.path[i][j] - self.spath[i][j])                   
                    self.spath[i][j] += weight_smooth *(self.spath[i-1][j] + self.spath[i+1][j] - (2.0 * self.spath[i][j]))
                    if i >= 2:
                        self.spath[i][j] += 0.5 * weight_smooth *(2.0 * self.spath[i-1][j] - self.spath[i-2][j]-self.spath[i][j])
                    if i <= len(self.path) - 3:
                        self.spath[i][j] += 0.5 * weight_smooth *(2.0 * self.spath[i+1][j] - self.spath[i+2][j]-self.spath[i][j])
                    change += abs(aux - self.spath[i][j])
        return self.spath
                
# ------------------------------------------------
# 
# this is the robot class
#

class robot:

    # --------
    # init: 
    #	creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length = 0.5):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0
        self.num_collisions    = 0
        self.num_steps         = 0

    # --------
    # set: 
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)

    # --------
    # set_noise: 
    #	sets the noise parameters
    #

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise     = float(new_s_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    # --------
    # check: 
    #    checks of the robot pose collides with an obstacle, or
    # is too far outside the plane

    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = sqrt((self.x - float(i)) ** 2 + 
                                (self.y - float(j)) ** 2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return False
        return True
        
    def check_goal(self, goal, threshold = 0.2):
        dist =  sqrt((float(goal[0]) - self.x) ** 2 + (float(goal[1]) - self.y) ** 2)
        return dist < threshold
        
    # --------
    # move: 
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, grid, steering, distance,tolerance = 0.001, max_steering_angle = pi / 4.0):

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0


        # make a new copy
        res = robot()
        res.length            = self.length
        res.steering_noise    = self.steering_noise
        res.distance_noise    = self.distance_noise
        res.measurement_noise = self.measurement_noise
        res.num_collisions    = self.num_collisions
        res.num_steps         = self.num_steps + 1

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)


        # Execute motion
        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:

            # approximate by straight line motion

            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * pi)

        else:

            # approximate bicycle model for motion

            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

        # check for collision
        # res.check_collision(grid)

        return res

    # --------
    # sense: 
    #    

    def sense(self):

        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]
    # --------
    # measurement_prob
    #    computes the probability of a measurement
    # 
    def measurement_prob(self, measurement):
        # compute errors
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0)/ sqrt(2.0 * pi * (self.measurement_noise ** 2))
        error *= exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0)/ sqrt(2.0 * pi * (self.measurement_noise** 2))

        return error

    def __repr__(self):
        # return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)
        return '[%.5f, %.5f]'  % (self.x, self.y)

# ------------------------------------------------
# 
# this is the particle filter class
#

class particles:

    # --------
    # init: 
    #	creates particle set with given initial position
    #

    def __init__(self, theta,steering_noise, distance_noise, measurement_noise, N = 1000, x=0, y=0):
        self.N = N
        self.steering_noise    = steering_noise
        self.distance_noise    = distance_noise
        self.measurement_noise = measurement_noise        
        self.data = []
        for i in range(self.N):
            x1=random.random()*len(grid)
            y1=random.random()*len(grid[0])
            r = robot()
            r.set(x1, y1, theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)

    # --------
    #
    # extract position from a particle set
    # 
    
    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((self.data[i].orientation-self.data[0].orientation + pi) % (2.0 * pi))+ self.data[0].orientation - pi)
        return [x / self.N, y / self.N, orientation / self.N]

    # --------
    #
    # motion of the particles
    # 

    def move(self, grid, steer, speed):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move(grid, steer, speed)
            newdata.append(r)
        self.data = newdata

    # --------
    #
    # sensing and resampling
    # 
    def sense(self, Z):
        #IMPORTANCE WEIGHT
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3

# --------
#
# run:  runs control program for the robot
#

def run(grid, goal, spath, params, printflag = False, speed = 0.1, timeout = 1000):
    x_robot_positions = []
    y_robot_positions = []
    tetha_rot = []
    
    #creating a ROBOT
    myrobot = robot()
    myrobot.set(0., 0., 0.)
    myrobot.set_noise(steering_noise, distance_noise, measurement_noise)
    x_robot_positions.append((myrobot.y*10)+5)
    y_robot_positions.append((myrobot.x*10)+5)
    tetha_rot.append(myrobot.orientation)    
    
    #CREATING PARTICLES
    filter = particles(myrobot.orientation,steering_noise, distance_noise, measurement_noise)
 
    cte  = 0.0
    err  = 0.0
    N    = 0
    int_err = 0
    index = 0 # index into the path
    while not myrobot.check_goal(goal) and N < timeout:
        diff_cte = - cte


        # ----------------------------------------
        # compute the CTE

        # start with the present robot estimate
        #PROMEDIO}7AVERAGE OF ALL THE PARTICLES LOCALIZATION
        estimate = filter.get_position()
        
        #taking care the robot doesnt get out the smoothed lines
        dx = spath[index+1][0]-spath[index][0]
        dy = spath[index+1][1]-spath[index][1]
        rx= estimate[0] - spath[index][0]
        ry= estimate[1] - spath[index][1]
        u = (rx*dx+ry*dy)/sqrt(dx**2+dy**2)
        cte=(ry*dx-rx*dy)/sqrt(dx**2+dy**2)
        
        if u >1.0 and index < len(spath) - 1:
            index+=1
            #int_err=0
        # ----------------------------------------

        #difference= diff = -cte(past)+cte(now)
        diff_cte += cte

        #PD CONTROL
        steer = - params[0] * cte - params[1] * diff_cte - 0.004*int_err 
        int_err+=diff_cte

        myrobot = myrobot.move(grid, steer, speed)
        x_robot_positions.append((myrobot.y*10)+5)
        y_robot_positions.append((myrobot.x*10)+5)
        tetha_rot.append(myrobot.orientation)
        
        #MOTION OF PARTICLES
        filter.move(grid, steer, speed)    
        
        #MEASUREMENT OF ROBOT AND PARTICLES
        Z = myrobot.sense()
        filter.sense(Z)

        if not myrobot.check_collision(grid):
            print('##### Collision ####')

        err += (cte ** 2)
        N += 1

        if printflag:
            print(myrobot, cte, index, u)
        
        #vpython.rate(50)
    
    return [myrobot.check_goal(goal), myrobot.num_collisions, myrobot.num_steps],x_robot_positions,y_robot_positions,tetha_rot


# ------------------------------------------------
# 
# this is our main routine
#

    
def main(grid, init, goal, steering_noise, distance_noise, measurement_noise,weight_data, weight_smooth, p_gain, d_gain):
    landmarkss = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j]==1:
                landmarkss.append((i,j))

    z = (len(grid)*10)+5
    x = (len(grid[0])*10)+5
    
    escena = vpython.canvas(width=1200,height=1000)
    escena.title = "Simulador de Autonomía Vehicular"
    escena.background = vpython.color.cyan
    escena.center = vpython.vector(x/2,-1,z/2)
    escena.forward= vpython.vector(0,-2,-1)
    #escena.range= 80
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
    
    suelo = vpython.box(pos=vpython.vector(x/2,-0.5,z/2),color=vpython.color.white,size=vpython.vector(x,1.0,z),texture={'file':vpython.textures.metal, 'place':'all'})
    high_wall = 30
    for i in range(len(landmarkss)):
        x_wall = (landmarkss[i][1]*10)+5
        z_wall = (landmarkss[i][0]*10)+5
        if i%2==0:
            wall = vpython.box(pos=vpython.vector(x_wall,15,z_wall),color=vpython.vector(1,0.7,0.2),size=vpython.vector(10,high_wall,10),texture={'file':vpython.textures.metal, 'place':'all'})
        else:
            wall = vpython.box(pos=vpython.vector(x_wall,25,z_wall),color=vpython.vector(1,0.7,0.2),size=vpython.vector(10,high_wall+20,10),texture={'file':vpython.textures.metal, 'place':'all'})
    
    #IMPLEMENTING PLANING: A* ALGORITHM
    path = plan(grid, init, goal)
    path.astar()
    path_way = path.path
    
    #IMPLEMENTING SMOOTHNESS PATH
    path.smooth(weight_data, weight_smooth)
    path_smoothed = path.spath
    
    inicio =vpython.text(text="Inicio",align='center',color=vpython.color.green,pos=vpython.vector((path_smoothed[0][1]*10)+5,22,(path_smoothed[0][0]*10)+5),height=5,width=12)
    meta = vpython.text(text="Meta",align='center',color=vpython.color.green,pos=vpython.vector((path_smoothed[-1][1]*10)+5,22,(path_smoothed[-1][0]*10)+5),height=5,width=12)
    
    #IMPLEMENTING PARTICLE FILTERS AND PID
    result, x_robot_position,y_robot_position,tetha_rot = run(grid, goal, path.spath, [p_gain, d_gain])
    
    #Representing on a 3D simulation
    pos = vpython.vector(x_robot_position[0],2,y_robot_position[0])
    vehicle = vpython.box(pos=pos,color=vpython.color.blue,size=vpython.vector(6,4,8))
    trayectoria = vpython.curve(color=vpython.color.white)
    trayectoria_car = vpython.curve(color=vpython.color.black)
    sensor_get = vpython.sphere(pos=pos,color=vpython.color.yellow,opacity=0.3,radius=50)
    #time.sleep(4)
    
    escena.forward = vpython.vector(0,-2,1)
    
    for k in range(len(path_smoothed)):
        #smoothed trajectory from path finder
        poss = vpython.vector((path_smoothed[k][1]*10)+5,2,(path_smoothed[k][0]*10)+5)
        trayectoria.append(poss)
        vpython.rate(20)
    time.sleep(3)
    escena.range= 70                       
    last_orient = 0
    for i in range(len(x_robot_position)):
        npos = vpython.vector(x_robot_position[i],2,y_robot_position[i])
        vehicle.pos = npos
        sensor_get.pos = npos
        vehicle.rotate(axis=vpython.vector(0,1,0),angle=-last_orient)
        vehicle.rotate(axis=vpython.vector(0,-1,0),angle=-tetha_rot[i])
        last_orient = tetha_rot[i]
        trayectoria_car.append(npos)
        escena.center = npos
        vpython.rate(20)
        
    return result
# ------------------------------------------------
# 
# input data and parameters
#


# grid format:
#   0 = navigable space
#   1 = occupied space

grid3 = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 1],
        [0, 1, 0, 1, 0, 0]]

grid = [[0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0]]

grid2 = [[0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0],
        [1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0],
        [0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0]]


init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]


steering_noise    = 0.1
distance_noise    = 0.03
measurement_noise = 0.3

weight_data       = 0.1
#weight_smooth =0.05
weight_smooth     = 0.2
p_gain            = 2.0
d_gain            = 6.0

#p_gain = 3.3205086077857686
#d_gain = 5.652317975118982

    
print(main(grid, init, goal, steering_noise, distance_noise, measurement_noise,weight_data, weight_smooth, p_gain, d_gain))




def twiddle(init_params):
    #este twiddle no sirve
    n_params   = len(init_params)
    dparams    = [1.0 for row in range(n_params)]
    params     = [0.0 for row in range(n_params)]
    K = 10

    for i in range(n_params):
        params[i] = init_params[i]


    best_error = 0.0;
    for k in range(K):
        ret = main(grid, init, goal, 
                   steering_noise, distance_noise, measurement_noise, 
                   params[0], params[1], params[2], params[3])
        if ret[0]:
            best_error += ret[1] * 100 + ret[2]
        else:
            best_error += 99999
    best_error = float(best_error) / float(k+1)
    print(best_error)

    n = 0
    while sum(dparams) > 0.0000001:
        for i in range(len(params)):
            params[i] += dparams[i]
            err = 0
            for k in range(K):
                ret = main(grid, init, goal, 
                           steering_noise, distance_noise, measurement_noise, 
                           params[0], params[1], params[2], params[3], best_error)
                if ret[0]:
                    err += ret[1] * 100 + ret[2]
                else:
                    err += 99999
            print(float(err) / float(k+1))
            if err < best_error:
                best_error = float(err) / float(k+1)
                dparams[i] *= 1.1
            else:
                params[i] -= 2.0 * dparams[i]            
                err = 0
                for k in range(K):
                    ret = main(grid, init, goal, 
                               steering_noise, distance_noise, measurement_noise, 
                               params[0], params[1], params[2], params[3], best_error)
                    if ret[0]:
                        err += ret[1] * 100 + ret[2]
                    else:
                        err += 99999
                print(float(err) / float(k+1))
                if err < best_error:
                    best_error = float(err) / float(k+1)
                    dparams[i] *= 1.1
                else:
                    params[i] += dparams[i]
                    dparams[i] *= 0.5
        n += 1
        print('Twiddle #', n, params, ' -> ', best_error)
    print(' ')
    return params


#print(twiddle([weight_data, weight_smooth, p_gain, d_gain]))
