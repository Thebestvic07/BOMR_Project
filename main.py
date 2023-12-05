## Lib import
from timeit import default_timer as timer
from tdmclient import ClientAsync, aw
import time
import cv2 as cv
import keyboard

from Codes.utils.data import *
from Codes.utils.communication import *
from Codes.kalman_filter import *
from Codes.motion_control import *
from Codes.obstacle_avoidance import *
from Codes.camera import *
 

## Constants
GRID_RES = 25
THYMIO_WIDTH = 2.0
TIMESTEP = 0.1


## Functions

def run_camera(mes_pos : Robot, mes_goal: Point, map : Map):
    '''
    Function that updates the global Mes_Robot variable with camera data every 0.1 seconds

    '''
    cap = cv2.VideoCapture(0)

    while True:
        if cap.isOpened() == False:
            print("Error : video stream closed")
        else:
            frame = cap.read()[1]
    
            frame, arucos, robot_pos, angle = show_robot(frame, grid_resolution)
            goal_pos = get_goal_pos(arucos, grid_resolution)

            if goal_pos != (0, 0):
                mes_goal = Point(goal_pos[0], goal_pos[1])

            mes_pos

            cv2.imshow("Video Stream", frame)

            print(f'Robot position: {robot_pos} and angle: {angle}')
            print(f'Goal position: {goal_pos}')
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        time.sleep(0.1)


def update_thymio(thymio : Thymio):
    '''
    Function that updates the variables of the thymio object (sensor data) every 0.1 seconds

    '''
    while True:
        thymio.read_variables()

        time.sleep(0.1)


def update_control():



## Main