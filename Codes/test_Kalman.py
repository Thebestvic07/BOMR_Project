from utils.data import *
import numpy as np
from kalman_filter import*
import matplotlib.pyplot as plt

iter=100

pos = Robot(position=Point(x=0, y=0), direction=0) 
kalman = Kalman(pos)

speedvar = 6.25
posvar = 0.25
thetavar = 0.01
dt = 0.1

memory = []

# GO Straight 
inputspeed=np.array([50,50])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)
    #pos_next, speed = kalman.kalman_filter(input, mot_mes, None)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext)
    
    memory.append([pos_next.position.x, pos_next.position.y])


# Make a curve 
inputspeed=np.array([20,50])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)
    #pos_next, speed = kalman.kalman_filter(input, mot_mes, None)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext)
    
    memory.append([pos_next.position.x, pos_next.position.y])


# and the other way around
inputspeed=np.array([50,20])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)
    #pos_next, speed = kalman.kalman_filter(input, mot_mes, None)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext)
    
    memory.append([pos_next.position.x, pos_next.position.y])

# GO Straight 
inputspeed=np.array([50,50])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)
    #pos_next, speed = kalman.kalman_filter(input, mot_mes, None)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext)
    
    memory.append([pos_next.position.x, pos_next.position.y])


memory = np.array(memory)
plt.plot(memory[:,0],memory[:,1])
plt.ylim([-50, 50])
plt.xlim([-10,100])
plt.show()