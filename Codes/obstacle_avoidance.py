#local avoidance with obstacle identification via capteurs
#we used the potential field Navigation
#return the velocity of the robot in the local frame

import numpy as np


def obstacle_avoidance(prox_horizontal, speed0=200):

    speedGain = 2     # gain used with ground gradient
             # gains used with front proximity sensors 0..4
    obstGain = 0.05    # speed gain used with obstacle avoidance
    obstThr=1000 #detection of box

    obstacle_detected=False

    if any(value > obstThr for value in prox_horizontal):
        obstacle_detected=True

    if not(obstacle_detected):
        return obstacle_detected,0,0

    elif obstacle_detected:
        diffDelta=0.5*prox_horizontal[0]+0.75*prox_horizontal[1]-1*prox_horizontal[2]-0.75*prox_horizontal[3]-0.5*prox_horizontal[4]
        motor_L= obstGain*diffDelta
        motor_R= -obstGain*diffDelta
        return obstacle_detected, motor_L , motor_R
    
    else:
       print("error in obstacle avoidance") 


       """     obstSpeedGain = [6, 4, -2, -6, -8]
    obstacle_detected=False
    obstThr=1500 #detection of box

    obstSpeedGain_np = np.array(obstSpeedGain)

    if any(value > obstThr for value in prox_horizontal):
        obstacle_detected=True

        
    if not(obstacle_detected):
        return obstacle_detected,0,0

    elif obstacle_detected:
        spLeft = np.sum(prox_horizontal * obstSpeedGain_np // 100)
        spRight= np.sum(prox_horizontal[::-1] * obstSpeedGain_np // 100)

    # motor control
        motor_L = speed0 + spLeft
        motor_R = speed0 + spRight
        return obstacle_detected, motor_L , motor_R
    
    else:
       print("error in obstacle avoidance")

 """
