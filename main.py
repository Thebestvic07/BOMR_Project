## Lib import
from timeit import default_timer as timer
from tdmclient import ClientAsync, aw
import time
import sys
import cv2 as cv
import threading
import keyboard

from Codes.utils.data import *
from Codes.utils.communication import *
from Codes.kalman_filter import *
from Codes.motion_control import *
from Codes.obstacle_avoidance import *
from Codes.camera import *
from Codes.map import *
 

## Constants
DEFAULT_GRID_RES = 20
TIMESTEP = 0.1
SIZE_THYM = 2.5  #size of thymio in number of grid
SPEEDCONV = 0.05
LOST_TRESH = 10 #treshold to be considered lost
REACH_TRESH = 3 #treshhold to reach current checkpoint
GLOBAL_PLANNING = True
GOAL_REACHED = False


## Functions

def run_camera(mes_pos : Robot, mes_goal: Point, camera : Camera, grid_res=DEFAULT_GRID_RES):
    '''
    Function that updates the global Mes_Robot variable with camera data every 0.1 seconds on average

    '''
    cap = cv2.VideoCapture(0)

    ret, frame = cap.read()
    camera.update(ret)
    
    width, height = 707, 10007
    # width, height = set_param_frame(arucos)

    last_known_goal_pos = (0, 0)
    robot_pos = (0, 0)
    while True:
        if cap.isOpened() == False:
            print("Error : video stream closed")
        else:
            frame = cap.read()[1]

            frame, arucos = get_arucos(frame)

            frame, arucos, robot_pos, angle = show_robot(frame, arucos, grid_res)
            goal_pos = center_in_grid(arucos, 99, grid_res)

            if robot_pos != (0,0) :
                robot_pos = Point(robot_pos[0], robot_pos[1])
                mes_pos.update(Robot(robot_pos, angle, True))
            else:
                mes_pos.update(Robot(mes_pos.position, mes_pos.direction, False))
                    
            if goal_pos != (0, 0):
                mes_goal.update(Point(goal_pos[0], goal_pos[1]))
            draw_goal(frame, arucos, grid_res)

            # frame = projected_image(frame, arucos, width, height)
            cv2.imshow("Video Stream", frame)

            # print(f'Robot position: {robot_pos} and angle: {angle}')
            # print(f'Goal position: {goal_pos}')
            
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        time.sleep(0.095)


def update_thymio(thymio : Thymio):
    '''
    Function that updates the variables of the thymio object (sensor data) every 0.1 seconds on average

    '''
    while True:
        thymio.read_variables()
        time.sleep(0.095)


def display(env : Environment, path : list, visitedNodes : list, grid_res=DEFAULT_GRID_RES):
    '''
    Function that displays the current state of the robot and the goal on the screen
    '''
    print("Starting display thread...")

    # Update display
    while True:
        display = cv.imread(".\captured_frame.png")
        if env.map.frame is not None:     #check if frame is not empty
                #show goal position with red point
                display = draw_circle(display, (env.goal.x, env.goal.y), grid_res, radius=10, color=(0, 0, 255), thickness=-1)
            
                #show robot position with green point + arrow
                display = draw_circle(display, (env.robot.position.x, env.robot.position.y), grid_res, radius=5, color=(0, 255, 0), thickness=-1)
                display = draw_arrow_from_robot(display, env.robot, grid_res)

                if path != []:
                    #show path with blue points 
                    for point in path:
                        display = draw_circle(display, (point.x, point.y), grid_res, radius=3, color=(255, 0, 0), thickness=-1) 
                else:
                    #show visited nodes with blue points 
                    for point in visitedNodes:
                        display = draw_circle(display, (point.x, point.y), grid_res, radius=2, color=(100, 0, 0), thickness=-1)
                    
        cv.imshow("Positions", display) 

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        time.sleep(0.1)


## Main

if __name__ == "__main__":
    # Init Thymio
    thymio = Thymio()

    # Init map
    map = Map([], [], None)

    builtmap, grid_res = apply_grid_to_camera(DEFAULT_GRID_RES)
    frame = cv2.imread("captured_frame.png")
    
    map.update(builtmap, frame)

    # Init variables
    Mes_car = Robot(Point(0,0), 0)  # Mes_car = measured car
    Mes_goal = Point(0,0)           # Mes_goal = measured goal
    env = Environment(Mes_car, map, Mes_goal)       # env = estimated environment (updated with Kalman)
    input = Motors(0,0)             # input = motors command
    path = list()                   # path = list of checkpoints to reach
    visitedNodes = list()           # visitedNodes = list of visited nodes during A* algorithm
    camera = Camera()

    # Launch Threads
    camera_thread = threading.Thread(target=run_camera, args=(Mes_car, Mes_goal, camera, grid_res), daemon=True)
    camera_thread.start()

    thymio_thread = threading.Thread(target=update_thymio, args=(thymio,), daemon=True)
    thymio_thread.start()

    display_thread = threading.Thread(target=display, args=(env, path, visitedNodes, grid_res), daemon=True)
    display_thread.start()

    # Wait for camera to find Thymio and goal

    while True:
        if camera.Status == False:
            print("Waiting for camera to start...")
        else: 
            if Mes_car.found:
                print("Thymio found !")
            if Mes_goal.x != 0 or Mes_goal.y != 0:
                print("Goal found !")
            if Mes_car.found and (Mes_goal.x != 0 or Mes_goal.y != 0):
                print("Let's go !")
                break

            print("Waiting for camera to find Thymio and goal...")

        # Check if escape key pressed
        if keyboard.is_pressed('esc'):
            break

        time.sleep(0.1)

    # Init Kalman filter
    time.sleep(3)
    kalman = Kalman(Mes_car)

    # Init timer
    start = time.time()
    current = start

    print("Starting main loop...")

    print(f'Robot position: {(Mes_car.position.x, Mes_car.position.y)} and angle: {Mes_car.direction}')
    print(f'Goal position: {(Mes_goal.x, Mes_goal.y)}')

    # Init loop
    while True:
        # Update motion every 0.1 seconds
        deltaT = current - start
        if deltaT < TIMESTEP:
            time.sleep(0.01)
            current = time.time()
            continue

        # Update env with Kalman
        if not GOAL_REACHED:
            print(thymio.motors.left, "  ", thymio.motors.right)
            newcar, newspeed = kalman.kalman_filter(input, thymio.motors, Mes_car, deltaT)
            env.update(Environment(newcar, map, Mes_goal))
        
        # Compute path if needed
        if GLOBAL_PLANNING:
            print("Planning...")

            visitedNodes.clear()
            path.clear()

            calculate_path(env, path, visitedNodes, 0, False)
            GLOBAL_PLANNING = False
            print("Planning finished !")

            # Update timer
            start = time.time()
            current = start
            continue
        
        # Compute distance to next checkpoint and update accordingly 
        if not GOAL_REACHED and len(path) > 0:
            dist_to_checkpoint = env.robot.position.dist(path[0])
        else:
            dist_to_checkpoint = 0

        if dist_to_checkpoint >= LOST_TRESH:
            # If lost, recompute path on next iteration
            print("Lost !")
            thymio.set_variable(Motors(0,0))
            GLOBAL_PLANNING = True
            continue

        if dist_to_checkpoint <= REACH_TRESH:
            # If sufficiently close to checkpoint, remove it from path and go to next one 
            print("Checkpoint reached !")
            path.pop(0) if len(path) > 1 else print("Path finished !")

        # Check if goal reached
        if env.robot.position.dist(env.goal) < 0.1:
            print("Goal reached !")
            thymio.set_variable(Lights([0,255,0]))   # Light up the Thymio !
            time.sleep(0.2)
            GOAL_REACHED = True

        # Update motion
        Kp_rot, Kp_fwd = controller(dist_to_checkpoint, speedConv = SPEEDCONV, thymio_width = SIZE_THYM)

        motor_L, motor_R = compute_velocity(env.robot, path[0], Kp_rot, Kp_fwd, GOAL_REACHED)
        print(f'motion motor: {motor_L}, {motor_R}')

        # Compute and set motors speed
        prox_array = thymio.sensors.prox
        obstacle_detected, addLeft , addRight = obstacle_avoidance(prox_array)
        if obstacle_detected : 
            motor_L += addLeft
            motor_R -= addRight

        print(f'obstacle contribution: {addLeft}, {addRight}')

        input = Motors(int(motor_L), int(motor_R))
        thymio.set_variable(input)

        # Update timer
        start = time.time()
        current = start

        # Check if escape key pressed
        if keyboard.is_pressed('esc'):
            break

    # Stop Thymio
    thymio.stop()
    exit()

