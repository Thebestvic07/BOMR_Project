#local avoidance with obstacle identification via capteurs
#we used the potential field Navigation
#return the velocity of the robot in the local frame

import numpy as np


def obstacle_avoidance(prox_horizontal):

    obstThr=1500 #detection of box
    
    addLeft = 0
    addRight = 0
    weight=[50,75,-100,-75,-50]

    obstacle_detected=False

    if any(value > obstThr for value in prox_horizontal):
        obstacle_detected=True

    if not(obstacle_detected):
        return obstacle_detected,0,0

    #this part is inspired from the potential field navigation excercise in session 4
    elif obstacle_detected:
        for i in range(5):
            addLeft += prox_horizontal[i] * Delta[i]//100
            addRight += prox_horizontal[i] * Delta[4 - i]//100
        
        return obstacle_detected, addLeft , addRight
    
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
