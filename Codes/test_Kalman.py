from utils.data import *
import numpy as np
from kalman_filter import*
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

iter=100

pos = Robot(position=Point(x=10, y=10), direction= 0, found = True)
kalman = Kalman(pos)


speedvar = 5
posvar = 0.125
thetavar = 0.01
dt = 0.1

memory = []
Cam = False

##Testing Motion Model
# inputspeed=np.array([50,50])
# prev_state = np.array([0,0,10,10,0])
# pos_next, G = motion_model(prev_state, inputspeed, dt)

# for i in range(iter):
#     pos_next, G = motion_model(pos_next, np.array([0,0]), dt)
#     memory.append([pos_next[2], pos_next[3], pos_next[4]])


# inputspeed=np.array([-50,-50])
# pos_next, G = motion_model(pos_next, inputspeed, dt)

# inputspeed=np.array([0,6.3*12.5])
# pos_next, G = motion_model(pos_next, inputspeed, dt)

# for i in range(iter):
#     pos_next, G = motion_model(pos_next, np.array([0,0]), dt)
#     memory.append([pos_next[2], pos_next[3], pos_next[4]])

# GO Straight 
inputspeed=np.array([50,50])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext, found = Cam)
    
    memory.append([pos_next.position.x, pos_next.position.y, pos_next.direction])


# Make a curve 
inputspeed=np.array([0,50])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext, found = Cam)
    
    memory.append([pos_next.position.x, pos_next.position.y, pos_next.direction])


# and the other way around
inputspeed=np.array([50,0])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext, found = Cam)
    
    memory.append([pos_next.position.x, pos_next.position.y, pos_next.direction])

# GO Straight 
inputspeed=np.array([50,50])
for i in range(iter):
    input = Motors(inputspeed[0], inputspeed[1])
    mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))

    pos_next, speed = kalman.kalman_filter(input, mot_mes, pos)

    xnext = np.random.normal(pos_next.position.x + speed[0]*np.cos(pos_next.direction)*dt, posvar)
    ynext = np.random.normal(pos_next.position.y + speed[0]*np.sin(pos_next.direction)*dt, posvar)
    thetanext = np.random.normal(pos_next.direction + speed[1]*dt, thetavar)
    pos = Robot(position=Point(x=xnext, y=ynext), direction=thetanext, found = Cam)
    
    memory.append([pos_next.position.x, pos_next.position.y, pos_next.direction])

memory = np.array(memory)

plt.subplots()
plt.title("Position")
plt.plot(memory[:,0],memory[:,1])
plt.ylim([20, 0])
plt.xlim([0,20])

plt.subplots()
plt.title("Direction")
plt.plot(memory[:,2])
plt.ylim([-np.pi, np.pi])

plt.show()

# memory = []
# filtered = []
# b, a = butter(2, 0.99, 'low')

# inputspeed=np.array([50,50])
# for i in range(iter):
#     mot_mes = Motors(np.random.normal(inputspeed[0],speedvar), np.random.normal(inputspeed[1],speedvar))
#     memory.append([mot_mes.left, mot_mes.right])
#     filtered.append(lfilter(b, a, np.array(memory)[-min(5,len(memory)):,:])[-1])

# left = np.array(memory)[:,0]
# right = np.array(memory)[:,1]

# leftf = np.array(filtered)[:,0]
# rightf = np.array(filtered)[:,1]

# plt.plot(left)
# plt.ylim([0, 100])
# plt.xlim([-10,100])
# plt.plot(leftf)
# plt.legend(["left", "leftf"])
# plt.show()