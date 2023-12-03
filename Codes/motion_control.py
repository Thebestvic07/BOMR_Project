
import math

def compute_velocity(position, goal, Kp_angle, Kp_dist, thymio_angle, final_goal_reached=False):

  # thymio stops when arrived at final goal
  if not (final_goal_reached):
    # computes the angle between the robot and the goal
    angle = -180*math.atan2(goal.y - position.y, goal.x - position.y)/math.pi
    angle_error = thymio_angle - angle

    # forces the angles to be between -pi and pi
    # to avoid problems with the atan2 function
    angle_error = (angle_error + 180) % 360 - 180

    #v_orientation + v_position
    motor_L = (Kp_dist * (180-abs(angle_error))) + Kp_angle * angle_error
    motor_R = (Kp_dist * (180-abs(angle_error))) - Kp_angle * angle_error

  else:
    motor_L = 0
    motor_R = 0

  return motor_L, motor_R

# def compute_angle_error(position, goal, thymio_angle):
 # angle = -180*math.atan2(goal[1] - position[1], goal[0] - position[0])/math.pi
  #angle_error = thymio_angle- angle
  #return angle_error


def controller(distance, distance_total, slowing_distance = 100):
    
  if distance < slowing_distance or (distance_total - distance) < slowing_distance:
    Kp_angle = 3
    Kp_dist = 0.8

  elif distance > slowing_distance and (distance_total - distance) > slowing_distance:
    Kp_angle = 1
    Kp_dist = 1

  return Kp_angle, Kp_dist

