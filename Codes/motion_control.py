
import math
from .utils.data import *

def compute_velocity(robot, goal, Rot_control, Fwd_control, final_goal_reached=False):
  """
  Computes the motor velocities based on the position, goal, and control parameters.

  :param position: Current position of the robot.
  :param goal: Goal position to reach (next point in the global path).
  :param Kp_angle: Proportional gain for angle control.
  :param Kp_dist: Proportional gain for distance control.
  :param thymio_angle: Current orientation of the robot.
  :param final_goal_reached: Flag indicating whether the final goal is reached, not motion is needed.
  :return: Tuple (motor_left, motor_right) representing motor velocities.
  """
  # thymio stops when arrived at final goal
  if not (final_goal_reached):
    position = robot.position
    thymio_angle = robot.direction
    #convert from radians to degrees
    thymio_angle = 180*thymio_angle/math.pi

    # computes the angle between the robot and the goal
    angle = -180*math.atan2(goal.y - position.y, goal.x - position.y)/math.pi
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


def controller(distance, slowing_distance = 1.0, speedConv = 0.05, thymio_width = 2.5):
  """
  Computes the control gains based on the current distance to the goal.
  It inhibits the straight movement (v_direction) of the robot when it starts the new segment of the path,
  and accelerates the rotation (v_orientation)

  :param distance: Current distance to the goal (next point in the global path)
  :param slowing_distance: Distance threshold for reducing control gains.
  :param speed_conv: Conversion factor for speed control.
  :return: Tuple (Kp_angle, Kp_dist) representing control gains.
  """

  #if the thymio is near from the starting point
  K = 20   #proportional gain

  if distance > slowing_distance :
    fwdspeed_target = 3.0 
    rotspeed_target = 0.5

  else:
    Kp_angle = 1.2 / speedConv
    Kp_dist = 1.0 / speedConv

  return rot_order, fwd_order
