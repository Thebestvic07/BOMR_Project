# BOMR_Project

# Course Project - Basics of Mobile Robotics

## Project Overview
The "Course Project - [ Basics of Mobile Robotics ](https://moodle.epfl.ch/course/view.php?id=15293) focuses on creating an environment where a Thymio robot navigates through obstacles using global and local navigation techniques. The project encompasses the following key aspects:

1. **Create an Environment**
   - The environment should include a set of obstacles (black paper sheets).
   - Thymio is expected to navigate and avoid obstacles without relying on sensors to detect them.

2. **Find the Best Path**
   - Thymio's objective is to travel from its position in the map to a target.

3. **Motion Control & Pose Estimation**
   - Implement motion control to guide the robot along the path.
   - Achieve accurate pose estimation through Bayesian filtering methods.

4. **Avoid Obstacles**
   - During navigation, Thymio must employ local navigation techniques to avoid physical obstacles (3D white blocks).
   - Those obstacles are not detected in the vision 

## Installation
To install and set up the project, you can download it from our [GitHub repository](https://github.com/Thebestvic07/BOMR_Project.git)

## Usage
To run the project, execute the main.py script.

## Vision and Aruco Configuration

Captures video frames from the camera, processes ArUco markers, and displays the robot's position, orientation, and goal information. The robot navigates until the goal is reached or the user interrupts the program by pressing 'q'.

## Kalman Filter
The Kalman filter function implement an EKF that estimates the current state.
For this it uses the camera & motor speed measurement, the motor input and the previous state. It returns the new a posteriori motor speed estimation and the new a posteriori position estimation.

## Global Navigation
The visualization includes creating plots to display the environment, A* algorithm progress, and the final path. Obstacles are shown in red, free cells in white, the start position in green, the goal position in purple, and the path in blue.

## Motion Control
The functions get the the angle, position of the Thymio and the next point in the global path. It will compute the velocity of both wheels depending on a Kp controler.

## Local Navigation
The obstacle avoidance algorithm uses a potential field navigation approach. The obstacles are identified based on proximity sensor readings and calculates the velocity adjustments in the local frame to navigate around them.

## Report
Find a complet report in a Jupyter Notebook on our [GitHub repository](https://github.com/Thebestvic07/BOMR_Project.git). The report provides a detailed explanations of our code.