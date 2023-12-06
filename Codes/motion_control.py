
import math
from .utils.data import *

def compute_velocity(robot, goal, Rot_control, Fwd_control, final_goal_reached=False):

  # thymio stops when arrived at final goal
  if not (final_goal_reached):
    position = robot.position
    thymio_angle = robot.direction
    #convert from radians to degrees
    thymio_angle = 180*thymio_angle/math.pi

    # computes the angle between the robot and the goal
    angle = 180-180*math.atan2(goal.y - position.y, goal.x - position.x)/math.pi
    angle_error = thymio_angle - angle

    # forces the angles to be between -pi and pi
    # to avoid problems with the atan2 function
    angle_error = (angle_error + 180) % 360 - 180

    #computes v_orientation, v_position so it doesn't get calculated twice
    Fwd_mot = Fwd_control * (180-abs(angle_error)) / 180
    Rot_mot = Rot_control * angle_error / 180

    #v_orientation + v_position
    motor_L = -Rot_mot + Fwd_mot
    motor_R =  Rot_mot + Fwd_mot

  else:
    motor_L = 0
    motor_R = 0

  return motor_L, motor_R

# def compute_angle_error(position, goal, thymio_angle):
 # angle = -180*math.atan2(goal[1] - position[1], goal[0] - position[0])/math.pi
 #angle_error = thymio_angle- angle
 #return angle_error


def controller(distance, slowing_distance = 0.7 , speedConv = 0.05, thymio_width = 2.5):
  
  K = 50   #proportional gain

  # if distance > slowing_distance :
  #   fwdspeed_target = 4.0 
  #   rotspeed_target = 1.0

  fwdspeed_target = 2.0
  rotspeed_target = 1.0

  fwd_order = K * fwdspeed_target / (2 * speedConv)
  rot_order = K * rotspeed_target / (thymio_width * speedConv * np.pi)

  return rot_order, fwd_order
