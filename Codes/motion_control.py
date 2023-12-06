import math
import numpy as np
from .utils.data import *

# Constants
Kp = 100/np.pi
Kd = Kp * 0.2

def compute_angle_error(position, checkpoint, thymio_angle):
  """
  Computes the angle error between the robot and the goal.

  :param position: Current position of the robot.
  :param checkpoint: position to reach (next point in the global path).
  :param thymio_angle: Current orientation of the robot.
  :return: Angle error between the robot and the goal.
  """
  angle = np.pi - np.arctan2( -(checkpoint.y - position.y), checkpoint.x - position.x)

  angle_error = thymio_angle - angle 
  angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
  return angle_error

def compute_derived_angle_error(angle_error, prev_angle_error, dt):
  """
  Computes the derived angle error between the robot and the goal.

  :param angle_error: Angle error between the robot and the goal.
  :param prev_angle_error: Previous angle error between the robot and the goal.
  :param dt: Time step.
  :return: Derived angle error between the robot and the goal.
  """
  return (angle_error - prev_angle_error) / dt


def controller(robot, checkpoint, base_speed, prev_angle_error, dt, final_goal_reached=False):
  '''
  This function computes the motor velocities based on the position, goal, and control parameters.
  param robot : Current position of the robot
  param checkpoint : Goal position to reach (next point in the global path)
  param base_speed : Base speed of the robot
  param final_goal_reached : Flag indicating whether the final goal is reached, not motion is needed.

  return : Input as a tuple (MotL, MotR) 
  return : angle_error
  '''
  if final_goal_reached:
      return 0, 0, 0
  
  angle_error = compute_angle_error(robot.position, checkpoint, robot.direction)
  derived_angle_error = compute_derived_angle_error(angle_error, prev_angle_error, dt)

  print("angle_error: ", angle_error, "derived_angle_error: ", derived_angle_error)

  rot_speed = Kp * angle_error + Kd * derived_angle_error

  print("rot_speed: ", rot_speed)

  # Clip the rotation speed to avoid saturation
  rot_speed = rot_speed if abs(rot_speed) < 200 - base_speed else 200 - base_speed

  MotL = base_speed + rot_speed
  MotR = base_speed - rot_speed

  print("MotL: ", MotL, "MotR: ", MotR, "angle_error: ", angle_error, "derived_angle_error: ", derived_angle_error)
  return MotL, MotR, angle_error


# def compute_velocity(robot, goal, Rot_control, Fwd_control, final_goal_reached=False):
#   """
#   Computes the motor velocities based on the position, goal, and control parameters.

#   :param position: Current position of the robot.
#   :param goal: Goal position to reach (next point in the global path).
#   :param Kp_angle: Proportional gain for angle control.
#   :param Kp_dist: Proportional gain for distance control.
#   :param thymio_angle: Current orientation of the robot.
#   :param final_goal_reached: Flag indicating whether the final goal is reached, not motion is needed.
#   :return: Tuple (motor_left, motor_right) representing motor velocities.
#   """
#   # thymio stops when arrived at final goal
#   if not (final_goal_reached):
#     position = robot.position
#     thymio_angle = robot.direction
#     #convert from radians to degrees
#     thymio_angle = 180*thymio_angle/math.pi

#     # computes the angle between the robot and the goal
#     angle = 180-180*math.atan2(goal.y - position.y, goal.x - position.x)/math.pi
#     angle_error = thymio_angle - angle

#     # forces the angles to be between -pi and pi
#     # to avoid problems with the atan2 function
#     angle_error = (angle_error + 180) % 360 - 180

#     #computes v_orientation, v_position so it doesn't get calculated twice
#     Fwd_mot = Fwd_control * (180-abs(angle_error)) / 180
#     Rot_mot = Rot_control * angle_error / 180

#     #v_orientation + v_position
#     motor_L = -Rot_mot + Fwd_mot
#     motor_R =  Rot_mot + Fwd_mot

#   else:
#     motor_L = 0
#     motor_R = 0

#   return motor_L, motor_R

# def compute_angle_error(position, goal, thymio_angle):
 # angle = -180*math.atan2(goal[1] - position[1], goal[0] - position[0])/math.pi
 #angle_error = thymio_angle- angle
 #return angle_error


# def controller(distance, slowing_distance = 1.0, speedConv = 0.05, thymio_width = 2.5):
#   """
#   Computes the control gains based on the current distance to the goal.
#   It inhibits the straight movement (v_direction) of the robot when it starts the new segment of the path,
#   and accelerates the rotation (v_orientation)

#   :param distance: Current distance to the goal (next point in the global path)
#   :param slowing_distance: Distance threshold for reducing control gains.
#   :param speed_conv: Conversion factor for speed control.
#   :return: Tuple (Kp_angle, Kp_dist) representing control gains.
#   """

#   #if the thymio is near from the starting point
#   K = 20   #proportional gain

#   # if distance > slowing_distance :
#   #   fwdspeed_target = 4.0 
#   #   rotspeed_target = 1.0

#   fwdspeed_target = 2.0
#   rotspeed_target = 1.0

#   fwd_order = K * fwdspeed_target / (2 * speedConv)
#   rot_order = K * rotspeed_target / (thymio_width * speedConv * np.pi)

#   return rot_order, fwd_order
