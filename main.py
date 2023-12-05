## Lib import
from timeit import default_timer as timer
from tdmclient import ClientAsync, aw
import time
import cv2 as cv
import threading

from Codes.utils.data import *
from Codes.utils.communication import *
from Codes.kalman_filter import *
from Codes.motion_control import *
from Codes.obstacle_avoidance import *
from Codes.camera import *
from Codes.map import *
 

## Constants
GRID_RES = 25
TIMESTEP = 0.1
SIZE_THYM = 2.0  #size of thymio in number of grid
LOST_TRESH = 10 #treshold to be considered lost
REACH_TRESH = 3 #treshhold to reach current checkpoint
GLOBAL_PLANNING = True
GOAL_REACHED = False


## Functions

def run_camera(mes_pos : Robot, mes_goal: Point):
    '''
    Function that updates the global Mes_Robot variable with camera data every 0.1 seconds

    '''
    cap = cv2.VideoCapture(1)

    arucos = get_arucos(cap.read()[1])[1]
    grid_resolution = get_dist_grid(arucos)
    grid_res = grid_resolution if grid_resolution != None else grid_res

    while True:
        if cap.isOpened() == False:
            print("Error : video stream closed")
        else:
            frame = cap.read()[1]
            '''
            arucos = get_arucos(frame)
            frame = projected_image(frame, arucos)
            grid_resolution = get_dist_grid(arucos)
            '''
            frame, arucos, robot_pos, angle = show_robot(frame, grid_resolution)
            goal_pos = get_goal_pos(arucos, grid_resolution)

            frame, arucos, robot_pos, angle = show_robot(frame, grid_res)
            goal_pos = get_goal_pos(arucos, grid_res)


            if robot_pos != None :
                robot_pos = Point(round(robot_pos[0]/grid_res), round(robot_pos[1]/grid_res))
                mes_pos.update(Robot(robot_pos, angle, True))
            else:
                mes_pos.update(Robot(mes_pos.position, mes_pos.direction, False))
            
            
            goal_pos = tuple(round(pos/grid_resolution) for pos in goal_pos)    
            if goal_pos != (0, 0):
                mes_goal.update(Point(goal_pos[0], goal_pos[1]))
            

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


## Main

if __name__ == "__main__":
    # Init Thymio
    thymio = Thymio()
    thymio.start()

    # Init map
    map = Map([], [], None)
    '''
    arucos = get_arucos(frame)
    frame = projected_image(frame, arucos)
    GRID_RES = get_dist_grid(arucos)
    '''
    frame, builtmap = apply_grid_to_camera(GRID_RES)
    display = frame.copy()
    map.update(builtmap, display)
        

    # Init variables
    Mes_car = Robot(Point(0,0), 0)  # Mes_car = measured car
    Mes_goal = Point(0,0)           # Mes_goal = measured goal
    env = Environment(Mes_car, map, Mes_goal)       # env = estimated environment (updated with Kalman)
    input = Motors(0,0)             # input = motors command
    path = []                       # path = list of checkpoints to reach

    
    # Launch Threads
    camera_thread = threading.Thread(target=run_camera, args=(Mes_car, Mes_goal,), daemon=True)
    camera_thread.start()

    thymio_thread = threading.Thread(target=update_thymio, args=(thymio,), daemon=True)
    thymio_thread.start()

    # Wait for camera to find Thymio
    time.sleep(3)
    while True:
        if Mes_car.found:
            break
        print("Waiting for camera to find Thymio...")
        time.sleep(0.1)

    # Init Kalman filter
    time.sleep(3)
    kalman = Kalman(Mes_car)

    if env.map.frame:     #check if frame is not empty
       cv2.circle(display, (GRID_RES*env.goal.x+GRID_RES//2, GRID_RES*env.goal.y+GRID_RES//2), 3, (0,0,255), -1)    #show goal position with red point
       cv2.circle(display, (GRID_RES*env.robot.position.x+GRID_RES//2, GRID_RES*env.robot.position.y+GRID_RES//2), 3, (0,255,0), -1)
       cv2.imshow("Positions", display) 
    
    # Init timer
    start = time.time()
    current = start

    # Init loop
    while True:
        display = env.frame.copy()
        # Update motion every 0.1 seconds
        deltaT = current - start
        if deltaT < TIMESTEP:
            time.sleep(0.01)
            current = time.time()
            continue

        # Update env with Kalman
        if not GOAL_REACHED:
            newcar = kalman.kalman_filter(input, thymio.motors, Mes_car.position, deltaT)
            env.update(Environment(newcar, map, Mes_goal))

        
        
        # Compute path if needed
        if GLOBAL_PLANNING:
            print("Planning...")
            path = calculate_path(env, SIZE_THYM, False)
            GLOBAL_PLANNING = False

            # Update timer
            start = time.time()
            current = start
            continue

        if env.map.frame:     #check if frame is not empty
            cv2.circle(display, (GRID_RES*env.goal.x+GRID_RES//2, GRID_RES*env.goal.y+GRID_RES//2), 5, (0,0,255), -1)    #show goal position with red point
            cv2.circle(display, (GRID_RES*env.robot.position.x+GRID_RES//2, GRID_RES*env.robot.position.y+GRID_RES//2), 5, (0,255,0), -1)
            for point in path:
                cv2.circle(display, (GRID_RES*point.x+GRID_RES//2, GRID_RES*point.y+GRID_RES//2), 2, (255, 0, 0), -1)   
            cv2.imshow("Positions", display) 
        
        # Compute distance to next checkpoint and update accordingly 
        if not GOAL_REACHED:
            dist_to_checkpoint = env.robot.position.dist(path[0])
        else:
            dist_to_checkpoint = 0

        if dist_to_checkpoint >= LOST_TRESH:
            # If lost, recompute path on next iteration
            print("Lost !")
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
        Kp_rot, Kp_fwd = controller(dist_to_checkpoint)

        motor_L, motor_R = compute_velocity(env.robot.position, path[0], Kp_rot, Kp_fwd, env.robot.direction, GOAL_REACHED)

        # Compute and set motors speed
        obstacle_detected, v_obstacle = obstacle_avoidance(thymio.sensors[:4])
        if obstacle_detected : 
            motor_L += v_obstacle
            motor_R -= v_obstacle

        input = Motors(motor_L, motor_R)
        thymio.set_variable(input)

        # Check if escape key pressed
        if keyboard.is_pressed('esc'):
            break

        # Update timer
        start = time.time()
        current = start

    # Stop Thymio
    thymio.stop()

