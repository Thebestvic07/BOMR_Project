from Codes.utils.data import *
import numpy as np

THYMIO_WIDTH = 5
TIMESTEP = 0.1

init_state = np.zeros(5,1)
init_cov = np.zeros(5,5) 

R = np.diag([10,10,1,1,1])    # measurement covariance matrix
Q = np.diag([0,0,0,0,0])    # state covariance matrix

def kalman_filter(pos_prev, mot_prev, cov_prev, pos_mes, mot_mes, mot_input, dt = TIMESTEP):
    """
    This function implement an EKF that estimates the current state 
    For this it uses the camera & motor speed measurement, the motor input and the previous state
    
    param pos_prev         : previous position a posteriori estimation (Robot object)
    param mot_prev         : previous motor speed a posteriori estimation (Motor object)
    param pos_mes          : measured position via camera (Robot object)
    param mot_mes          : measured motor speed (Motor object)
    param mot_input        : input motor speed (Motor object)
    param dt               : time step of the system (float)
    
    return pos_est         : new a posteriori position estimation
    return mot_est         : new a posteriori motor speed estimation 
    return cov_est         : new a posteriori covariance estimation
    """

    ## Prediciton through the a priori estimate
    a_priori_state, G = motion_model(pos_prev, mot_prev, mot_input, dt)    
    a_priori_cov      = np.dot(G, np.dot(cov_prev, G.T)) + R

    ## Gain computation
    # Since our measurement Y = X + noise, the jacobi matrix H is the identity matrix
    K = np.dot(a_priori_cov, np.linalg.inv(a_priori_cov+ Q))
    
    ## Update through the measurement   
    meas_state = np.concatenate((mot_mes, pos_mes), axis=0)
    innovation = meas_state - a_priori_state

    a_posteriori_state = a_priori_state + np.dot(K, innovation)
    a_posteriori_cov = np.dot((np.eye(5) - K), a_priori_cov)
    
    ## Return the new state
    mot_est = a_posteriori_state[0:2]
    pos_est = a_posteriori_state[2:5]
    cov_est = a_posteriori_cov
     
    return pos_est, mot_est, cov_est

## Our motion model is non linear.
# It is ruled by the following laws :
    #   forward speed    : v = (mot_r + mot_l) / 2
    #   rotational speed : w = (mot_r - mot_l) / THYMIO_WIDTH
    #   Position X       : x+ = x + v*cos(theta)*dt
    #   Position Y       : y+ = y + v*sin(theta)*dt
    #   Direction theta  : theta+ = theta + w*dt
# Therefore we'll take our state X = [mot_l, mot_r, x, y, theta]
# Our input is U = [T_mot_l, T_mot_r] (with T for target) 
# Our measurement Y = [S_mot_l, S_mot_r, x_mes, y_mes, theta_mes] (with S for sensor)
# The motors' speed are measured at all time while the position are only measured while the camera is active

def motion_model(prev_state, input, dt):
    """
    This function returns the state at the next time step and its Jacobian 
        param prev_state : previous state as an array(5,1)
        param input      : new input as an array(2,1)
        param dt         : timestep
 
        return estimated state as an array(5,1)
        return Jacobian as a numpy matrix (5,7)
    """
    est_state = np.zeros(5,1)
    Jacobian = np.zeros(5,7)

    v = (prev_state[1] + prev_state[0]) / 2
    w = (prev_state[1] - prev_state[0]) / THYMIO_WIDTH
    x = prev_state[2]
    y = prev_state[3]
    theta = prev_state[4]

    est_state[0,:] = input[0,:]
    est_state[1,:] = input[1,:]
    est_state[2,:] = x + v * np.cos(theta) * dt
    est_state[3,:] = y + v * np.sin(theta) * dt
    est_state[4,:] = theta + w * dt

    # mot l & r part
    Jacobian[0,5] = 1
    Jacobian[1,6] = 1

    # x part 
    Jacobian[2,0] = np.cos(theta) * dt / 2
    Jacobian[2,1] = np.cos(theta) * dt / 2
    Jacobian[2,2] = 1
    Jacobian[2,4] = - np.sin(theta) * v * dt / 2

    # y part
    Jacobian[2,0] = np.sin(theta) * dt / 2
    Jacobian[2,1] = np.sin(theta) * dt / 2
    Jacobian[2,3] = 1
    Jacobian[2,4] = np.cos(theta) * v * dt / 2

    # theta part 
    Jacobian[2,0] = -1 / THYMIO_WIDTH
    Jacobian[2,1] =  1 / THYMIO_WIDTH
    Jacobian[2,4] =  1

    return est_state, Jacobian