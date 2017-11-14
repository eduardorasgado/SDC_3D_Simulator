#3D PID

"""
Created on Sun Jul 16 10:57:18 2017
PID control
@author: Eduardo Rasgado
"""


#the desire trajectory for the robot is the x axis
#stering angle = -tau * crosstrack_error
import vpython
import time
import random
import numpy as np

class robot(object):
    def __init__(self,length=20.0):
        #xcreates robot and initializes location/orientaton to 0,0,0.
        self.x = 0.0
        self.y = 0.0
        self.orientation= 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0
        
    def setter(self,x,y,orientation):
        #sets a robot coordinate
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)
        
    def set_noise(self, steering_noise,distance_noise):
        #set the noise parameters
        #this is ften useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        
    def set_steeringdrift(self,drift):
        #sets the systematical steering drift parameter
        self.steering_drift = drift
        
    def move(self, steering, distance,tolerance=0.001,max_steering_angle=np.pi/4.0):
        #steering = front wheel steering angle, limited by max_steering_angle
        #distance = total distance driven, most be non-negative
        if steering > max_steering_angle:
            steering= max_steering_angle
        if steering < -max_steering_angle:
            steering=-max_steering_angle
        if distance < 0.0:
            distance = 0.0
        #make a new copy
        res = robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.steering_drift = self.steering_drift
        
        #appying noise
        steering2 = random.gauss(steering,self.steering_noise)
        distance2 = random.gauss(distance,self.distance_noise)
        
        #apply steering_drift
        steering2 += self.steering_drift
        
        #Execute motion
        #beta = tan(alpha)x d/L
        turn = np.tan(steering2) * distance2 / self.length
        
        if abs(turn) <tolerance:  #linea recta
            res.x = self.x + distance2 * np.cos(self.orientation)
            res.y = self.y + distance2 * np.sin(self.orientation)
            res.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            #turn > 0
            #approximate bycicle model for motion
            #globales:                       
            #R = d/beta
            radius = distance2 / turn
            #cx = x-Rsin(tetha)
            cx = self.x - (radius*np.sin(self.orientation))
            #cy = y+Rcos(tetha)
            cy = self.y + (radius*np.cos(self.orientation))
            #tetha = (tetha+betha)mod 2pi
            res.orientation = (self.orientation + turn) % (2.0*np.pi)
            #x = cx+Rsin(tetha+beta)
            res.x = cx + (np.sin(res.orientation)*radius)
            #y = cy - Rcos(tetha+betha)
            res.y = cy - (np.cos(res.orientation)*radius)
            
        return res,steering2
    
    def __repr__(self):
        return '[x=%s, y=%s, orient=%s]'%(self.x,self.y,self.orientation)
    
def run_proportional(param):
    myrobot = robot()
    myrobot.setter(0.0,5.0,0.0)
    #myrobot.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
    speed = 1.0 #motion distance equalt to speed (we assume time = 1)
    N=500
    tau =-param
    xs = []
    ys = []
    orient= []
    for i in range(N):
        xs.append(myrobot.x)
        ys.append(myrobot.y)
        a = myrobot.y*tau
        steer = a
        myrobot,errr = myrobot.move(steer,speed)  
        orient.append(myrobot.orientation)
        if i==150:
            myrobot.set_steeringdrift(40.0/180.0*np.pi) #volantazo
            
        if i ==400:
            myrobot.set_steeringdrift(-20.0/180.0*np.pi) #volantazo
        
        #print(myrobot,steer)
    return xs,ys,orient

def run_proportional_derivative(param1,param2):
    myrobot2 = robot()
    myrobot2.setter(0.0,5.0,0.0)
    #myrobot2.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
    speed = 1.0 #motion distance equalt to speed (we assume time = 1)
    N=500
    dt =1.0
    tau_p =param1
    tau_d =param2
    xs2 = []
    ys2 = []
    orient = []
    last_error = myrobot2.y
    for i in range(N):
        xs2.append(myrobot2.x)
        ys2.append(myrobot2.y)
        diff_cte = myrobot2.y - last_error
        steering_angle = -tau_p * myrobot2.y - tau_d * (diff_cte/dt)
        last_error = myrobot2.y
        myrobot2,errr = myrobot2.move(steering_angle,speed)
        orient.append(myrobot2.orientation)
        if i==150:
            myrobot2.set_steeringdrift(40.0/180.0*np.pi) #volantazo
        if i ==400:
            myrobot2.set_steeringdrift(-20.0/180.0*np.pi) #volantazo
        
        #print(myrobot2,steering_angle)
    return xs2,ys2,orient

def run_proportional_integrative_derivative(param1,param2,param3):
    myrobot = robot()
    myrobot.setter(0.0,5.0,0.0)
    #myrobot.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
    speed = 1.0 #motion distance equalt to speed (we assume time = 1)
    N=500
    dt =1.0
    tau_p =param1
    tau_d =param2
    tau_i = param3
    xs = []
    ys = []
    orient = []
    last_error = myrobot.y
    integrated_error =0.0
    for i in range(N):
        xs.append(myrobot.x)
        ys.append(myrobot.y)
        diff_cte = myrobot.y - last_error
        integrated_error +=myrobot.y
        last_error = myrobot.y
        steering_angle = -tau_p * myrobot.y - tau_d * (diff_cte/dt) -tau_i * integrated_error
        if i==150:
            myrobot.set_steeringdrift(40.0/180.0*np.pi) #volantazo
        if i ==400:
            myrobot.set_steeringdrift(-20.0/180.0*np.pi) #volantazo
            
        myrobot,errr = myrobot.move(steering_angle,speed)
        orient.append(myrobot.orientation)
        
        #print(myrobot,steering_angle)
    return xs,ys,orient

def make_robot():
    #reset the robot back
    robot0 = robot()
    robot0.setter(0,5.0,0)
    robot0.set_steeringdrift(40.0/180.0*np.pi)
    return robot0

def run_test(roboto, params,n=100,speed=1.0):
    x_trajectory = []
    y_trajectory = []
    err = 0
    prev_cte = roboto.y
    int_cte = 0.0
    for i in range(2*n):
        cte = roboto.y
        diff_cte = (cte-prev_cte)/speed
        int_cte+= cte
        prev_cte = cte
        steer = -params[0]*cte -params[1]*diff_cte - params[2]*int_cte
        roboto,errr = roboto.move(steer,speed)
        x_trajectory.append(roboto.x)
        y_trajectory.append(roboto.y)
        if i >=n:
            #suma de los errores
            err += cte**2
    return x_trajectory, y_trajectory, err/n #promedio de errores
                  
def twiddle(tol=0.2):
    p = [2.0,6.0,0.004]
    dp = [1,1,1]
    robot11 = make_robot()
    x_trajectory,y_trajectory,best_err = run_test(robot11,p)
    it = 0
    while sum(dp)>tol:
        print("Iteration {}, best error = {}".format(it,best_err))
        for i in range(len(p)):
            p[i] +=dp[i]
            robot11 = make_robot()
            x_trajectory,y_trajectory, err = run_test(robot11,p)
            
            if err<best_err:
                best_err = err
                dp[i]*=1.1
            else:
                p[i] -= 2.0 * dp[i]
                robot11 = make_robot()
                x_trajectory,y_trajectory,err = run_test(robot11,p)
                
                if err<best_err:
                    best_err = err
                    dp[i] += 1.1
                else:
                    p[i]+=dp[i]
                    dp[i]*=0.9
        it+=1
    return p

def representer(xs,ys,name,clock,orienter):
    escena = vpython.canvas(width=1200,height=600)
    escena.title = name
    escena.background = vpython.color.cyan
    
    pos = vpython.vector(xs[0],2,-ys[0])
    escena.range = 100
    escena.center = vpython.vector(0,0,0)
    escena.forward=vpython.vector(1,-2,1)
    #suelo = vpython.box(pos = vpython.vector(0,-1,0),size=vpython.vector(10,0.01,200),color=vpython.color.white)
    suelo = vpython.box(pos = vpython.vector(250,-1,0),size=vpython.vector(500,0.01,200),color=vpython.color.white,texture=vpython.textures.wood)
    vehicle = vpython.box(pos=pos,color=vpython.color.red,size=vpython.vector(8,4,6))
    trayectoria = vpython.curve(color=vpython.color.yellow)
    road = vpython.curve(color=vpython.color.black)
    muro = vpython.box(pos=vpython.vector(0,20,10),color=vpython.color.blue,size=vpython.vector(20,40.99,2),texture=vpython.textures.stucco)
    piedra1 = vpython.sphere(pos=vpython.vector(150,-1,-ys[150]),color=vpython.color.black,radius=1)
    piedra2 = vpython.sphere(pos=vpython.vector(xs[400],-1,-ys[400]),color=vpython.color.black,radius=1)
    time.sleep(clock)
    #escena.range = 45
    escena.range = 20
    for i in range(len(xs)):
        if i>0:
            vehicle.rotate(axis=vpython.vector(0,1,0),angle=-orienter[i-1])
        pos = vpython.vector(xs[i],2,-ys[i])
        #suelo.pos=vpython.vector(i/2.0,-1,0)
        #suelo.size = vpython.vector(i,0.01,200)
        
        muro.pos =vpython.vector(i/2,20,10)
        muro.size= vpython.vector(i,40.99,2)
        
        vehicle.pos = pos
        escena.center = vpython.vector(xs[i],5,-ys[i])
        trayectoria.append(pos)
        vehicle.rotate(axis=vpython.vector(0,-1,0),angle=-orienter[i])
        road.append(vpython.vector(i,2,0))
        vpython.rate(40)
    #suelo = vpython.box(pos=vpython.vector(250,-1,0),size=vpython.vector(500,0.01,200),color=vpython.color.white)
        
        
xs,ys,orient1 = run_proportional(0.1) #try with 0.3
name = "Proportional control"
clock1=4
representer(xs,ys,name,clock1,orient1)
input("siguiente")

xs2,ys2,orient2= run_proportional_derivative(0.2,3.0) #try with 0.3
name2 ="Control proporcional derivativo"
clock2=8
representer(xs2,ys2,name2,clock2,orient2)
input("siguiente")

xs3,ys3,orient3= run_proportional_integrative_derivative(0.2,3.0,0.004) #try with 0.3
name3 = "Control PID"
representer(xs3,ys3,name3,clock2,orient3)
input("siguiente")

#ajustando parametros de kp, kd, ki
params = twiddle()
kp,kd,ki = params
robot1 = make_robot()

print(kp,kd,ki)
#testing de las constantes obtenidas
#x_trajectory,y_trajectory,err = run_test(robot1,params)
xs4,ys4,orient4= run_proportional_integrative_derivative(kp,kd,ki)
name4 = "Control PID con constantes optimas"
representer(xs4,ys4,name4,clock2,orient4)
input("Fin")
"""
Las pruebas se pueden hacer jugando con el tiddle, con los puntos de inicio de cada robot y metiendo + o - piedras
recordar al variar los valores de twiddle cmbiar los de make_robot
"""