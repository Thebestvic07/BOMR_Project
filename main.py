from timeit import default_timer as timer
from tdmclient import ClientAsync, aw
import time

from Codes.utils.data import *
from Codes.utils.etalonnage import *
from Codes.utils.communication import *
from Codes.utils.import_all import *
from obstacle_avoidance.obstacle_avoidance import *
from obstacle_avoidance.motion_control import *



from Kalman import *
#redéfinir les noms des fichiers
#from global_path_planning import *
#from vision import *


#VISION
cam=cv.VideoCapture(0, cv.CAP_DSHOW)



#For navigation


#For display


#For kalman


############################### MAIN ##########################################

print("\nStart Main")

distance = 0
point_to_go = [0, 0]
prev_point_to_go = [0,0]
actual_point = 0
is_finished = False

start_time = None
timer = 0.1

thymio = Thymio()

#init tdm client
client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock())

print("\nThymio connected")
goal_reached = False

while not goal_reached:





    ######################################## VISION ################################
    

    ######################################## Kalman Filter ################################

    #pos_est, mot_est, cov_est = kalman_filter(pos_prev, mot_prev, cov_prev, pos_mes, mot_mes, mot_input, dt = TIMESTEP)

    ######################################## Global Navigation ################################    

    #Compute where to go





    ######################################## Local Navigation/Motion ################################
    
    distance_tot = Point.dist(prev_point_to_go, point_to_go)

    #need to define distance and distance_total
    Kp = controller(distance, distance_tot, slowing_distance = 100)


    motor_L,motor_R= compute_velocity(thymio.position, point_to_go, Kp, thymio.angle, goal_reached)

    # Compute and set motors speed
    obstacle_detected, motor_L_obstacle, motor_R_obstacle = obstacle_avoidance(thymio.sensors[:4])
    if obstacle_detected : 
        motor_L+=motor_L_obstacle
        motor_R+=motor_R_obstacle

    thymio.set_variable(Motors(motor_L,motor_R))


########################################STOP############################################
    key = cv.waitKey(1)
    if key == 27: 
        break  # Press esc to quit

#stop the robot
thymio.stop()
#close all windows
cv.destroyAllWindows()
