#local avoidance with obstacle identification via capteurs
#we used the potential field Navigation
#return the velocity of the robot in the local frame

import numpy as np


def obstacle_avoidance(prox_horizontal):
    """
    This function implements obstacle avoidance using the potential field navigation approach.
    It returns the velocity of the robot in the local frame based on the horizontal proximity sensor readings.

    :param prox_horizontal: List of horizontal proximity sensor readings.
    :return: Tuple (obstacle_detected, left_velocity, right_velocity).
             - obstacle_detected: Boolean indicating whether an obstacle is detected.
             - left_velocity: Velocity adjustment for the left motor in the local frame.
             - right_velocity: Velocity adjustment for the right motor in the local frame.
    """
    obstThr=1500 #detection of the white box
    
    addLeft = 0
    addRight = 0
    weight=[50,75,-100,-75,-50] #weight of associated to each sensor

    obstacle_detected=False


    # Check if any sensor reading is above the obstacle detection threshold
    if any(value > obstThr for value in prox_horizontal):
        obstacle_detected=True

    if not(obstacle_detected):
                # No obstacle detected, return zero velocity adjustments
        return obstacle_detected,0,0

    #this part is inspired from the potential field navigation excercise in session 4
    elif obstacle_detected:
        for i in range(5):
<<<<<<< HEAD
            addLeft += prox_horizontal[i] * weight[i]//200
            addRight += prox_horizontal[i] * weight[4 - i]//200
=======
            addLeft += prox_horizontal[i] * weight[i] // 200
            addRight += prox_horizontal[i] * weight[4 - i] // 200
>>>>>>> 5310da73dea54644310a0a7377365d8445fb9133
        
        return obstacle_detected, addLeft , addRight
    
    else:
       print("error in obstacle avoidance")
