import numpy as np
import cv2
import math 
import time
from .utils.data import *


def is_black_cell(image):
    # Know if the grid cell contains more than 1/8 pixels who are black
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_mask = cv2.threshold(image, 110, 255, cv2.THRESH_BINARY)

    # Calculate the percentage of black pixels in the image
    total_pixels = image.size
    black_pixels = total_pixels - cv2.countNonZero(binary_mask)
    percentage_black = (black_pixels / total_pixels) * 100

    # Set the threshold percentage
    threshold_percentage = 10  # Adjust this threshold as needed

    # Check if the percentage of black pixels exceeds the threshold
    if percentage_black > threshold_percentage:
        return True
    else:
        return False
    
def create_white_image(grid_resolution):
    # Create a white image
    white_image = 255 * np.ones((grid_resolution, grid_resolution, 3), dtype=np.uint8)

    # Save the white image
    cv2.imwrite('white_image.png', white_image)

def create_black_image(grid_resolution):
    # Create a black image
    black_image = np.zeros((grid_resolution, grid_resolution, 3), dtype=np.uint8)

    # Save the black image
    cv2.imwrite('black_image.png', black_image)

def replace_white():
    return cv2.imread('white_image.png')

def replace_black():
    return cv2.imread('black_image.png')


# Change the image according to the percentage of black pixels
def change_cell(image):
    
    if is_black_cell(image):
        # Change the image to a black cell
        return cv2.imread('black_image.png')

    else:
        # Change the image to a white cell
        return cv2.imread('white_image.png')
    

def apply_grid(image, grid_resolution):

    # Get image dimensions
    height, width = image.shape[:2]

    # Create black and white images
    create_black_image(grid_resolution)
    create_white_image(grid_resolution)

    # Get resolution of the grid
    x_cells = int(width / grid_resolution)
    y_cells = int(height / grid_resolution)

    #creat a 2d list of grid cells
    map = [[0 for _ in range(y_cells)] for _ in range(x_cells)]
    obstacles = []
    internal_map = [[0 for _ in range(y_cells)] for _ in range(x_cells)]

    obstacles = []


    new_image = list(range(y_cells))
    final_image = list(range(x_cells))

    for y in range(y_cells):
        for x in range(x_cells):
            # Define the bounding box for the current grid cell
            x_min, y_min = x*grid_resolution, y*grid_resolution
            x_max, y_max = x_min + grid_resolution, y_min + grid_resolution

            # Crop the image to the bounding box
            cell_content = image[y_min:y_max, x_min:x_max]

            if is_black_cell(cell_content):
                map[x][y] = 0
                obstacles.append(Point(x,y))
            else:
                map[x][y] = 1

            # Store the cell content in the grid
            internal_map[x][y] = change_cell(cell_content)

            if x==0:
                new_image[y] = internal_map[x][y]
            else:
                new_image[y] = np.hstack((new_image[y], internal_map[x][y]))
        
        if y==0:
            final_image = new_image[y]

        if y!=0:
            final_image = np.vstack((final_image,new_image[y]))

    map = Map([Point(0,0), Point(width,0), Point(width, height), Point(0,height)], obstacles)
    return final_image, map


def set_aruco():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    return detector

def get_info_arucos(corners, ids, image):

    arucos = []
    if len(corners) > 0:
 
        ids = ids.flatten()
 
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
 
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
 
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            
            arucos.append((cX, cY, markerID, (bottomLeft, topLeft)))
            
 
    return image, arucos

def robot_center_is(arucos):
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 95:
                return (arucos[i][0], arucos[i][1])
            
    else:
        return None
    
def center_in_grid(arucos, pos, grid_resolution):
    if len(arucos) !=0:
        x = int(pos[0]/grid_resolution)
        y = int(pos[1]/grid_resolution)
        return (x, y)
    else:
        return (0, 0)

def get_goal_pos(arucos, grid_resolution):
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 99:
                x = int(arucos[i][0])
                y = int(arucos[i][1])
                return (x, y)
            
    else:
        return (0, 0)
    
def check_if_goal_reached(arucos, robot_pos, goal_pos):
    if len(arucos) !=0:
        if (robot_pos == goal_pos) and (robot_pos != (0, 0)) and (goal_pos != (0, 0)):
            return True
        else:
            return False
    else:
        return False

def get_angle_of_robot(arucos):
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 95:
                x1, y1 = arucos[i][3][0][0], arucos[i][3][0][1]
                x2, y2 = arucos[i][3][1][0], arucos[i][3][1][1]
                angle = math.atan2(-(y2 - y1), x2 - x1)
                return angle
    else:
        return 0

def draw_arrow(image, arucos, angle, length=80, color=(0, 255, 0), thickness=3):

    if len(arucos) != 0:
        for i in range(len(arucos)):
            if arucos[i][2] == 95:
                arrow_start = (arucos[i][0], arucos[i][1])
                x =  int(round(arrow_start[0] + length * math.cos(angle)))
                y =  int(round(arrow_start[1] - length * math.sin(angle)))
                arrow_end = (x, y)
                 # Use cv2.arrowedLine to draw the arrow on the image
                image = cv2.arrowedLine(image, arrow_start, arrow_end, color, thickness)
                break
            
    return image

def activate_camera():
    # Open the camera
    cap = cv2.VideoCapture(0)

    while cap.isOpened():

        frame = cap.read()[1]

        cv2.imshow("Videa Stream", frame)
    
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    cap.release()

def get_arucos(frame):
    detector = set_aruco()
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
    frame, arucos = get_info_arucos(markerCorners, markerIds, frame)
    return frame, arucos

def show_robot(frame, arucos, grid_resolution):
    real_center = robot_center_is(arucos)
    robot_pos = center_in_grid(arucos, real_center, grid_resolution)
    angle = get_angle_of_robot(arucos)
    frame = draw_arrow(frame, arucos, angle)

    return frame, arucos, robot_pos, angle

def get_corners(arucos):
    pos1, pos2, pos3, pos4 = (0, 0), (0, 0), (0, 0), (0, 0)
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 0:
                pos1 = (arucos[i][0], arucos[i][1])
            if arucos[i][2] == 1:
                pos2 = (arucos[i][0], arucos[i][1])
            if arucos[i][2] == 2:
                pos3 = (arucos[i][0], arucos[i][1])
            if arucos[i][2] == 3:
                pos4 = (arucos[i][0], arucos[i][1])
        return pos1, pos2, pos3, pos4       
    else:
        return (0, 0), (0, 0), (0, 0), (0, 0)
    
def get_dist_grid(arucos):
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 0:
                x1, y1 = arucos[i][3][0][0], arucos[i][3][0][1]
                x2, y2 = arucos[i][3][1][0], arucos[i][3][1][1]
                dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                return dist
    else:
        return 0
    
def get_dist_height_circuit(arucos):
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 0:
                for j in range(len(arucos)):
                    if arucos[j][2] == 3:
                        x1, y1 = arucos[i][0], arucos[i][1]
                        x2, y2 = arucos[j][0], arucos[j][1]
                        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                        return dist
    else:
        return 0
    
def get_dist_width_circuit(arucos):
    if len(arucos) !=0:
        for i in range(len(arucos)):
            if arucos[i][2] == 0:
                for j in range(len(arucos)):
                    if arucos[j][2] == 1:
                        x1, y1 = arucos[i][0], arucos[i][1]
                        x2, y2 = arucos[j][0], arucos[j][1]
                        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                        return dist
    else:
        return 0
    
def set_param_frame(arucos):
    dist_aruco = int(get_dist_grid(arucos))
    width, height = 0, 0
    if dist_aruco != 0:
        width = int(get_dist_width_circuit(arucos)/dist_aruco) + 1
        height = int(get_dist_height_circuit(arucos)/dist_aruco) + 1
        width, height = width*dist_aruco, height*dist_aruco
    return width, height
    
def projected_image(frame, arucos, width, height):
    pos1, pos2, pos3, pos4 = get_corners(arucos)
    condition = width != 0 and height != 0 and pos1 != (0,0) and pos2 != (0,0) and pos3 != (0,0) and pos4 != (0,0)
    if condition:
        pts1 = np.float32([pos1, pos2, pos3, pos4])
        pts2 = np.float32([[0,0],[width,0],[width,height],[0,height]])
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        result = cv2.warpPerspective(frame,matrix,(width,height))
        return result
    else:
        return frame

def apply_grid_to_camera(grid_resolution):

    # Open the camera
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera

    # Wait for 3 seconds
    time.sleep(3)

    # Capture a frame
    ret, frame = cap.read()

    # Check if the frame was successfully captured
    if ret:
        frame = apply_grid(frame, grid_resolution)[0]
        map = apply_grid(frame, grid_resolution)[1]
        # Save the captured frame as an image
        cv2.imwrite("captured_frame.jpg", frame)
        print("Frame registered successfully.")

    # Release the camera
    cap.release()

    return map


cap = cv2.VideoCapture(0)

grid_resolution = get_dist_grid(get_arucos(cap.read()[1])[1])

last_known_goal_pos = (0, 0)

arucos = get_arucos(cap.read()[1])[1]

width, height = set_param_frame(arucos)

while cap.isOpened():

    robot_pos = (0, 0)

    frame = cap.read()[1]

    frame, arucos = get_arucos(frame)

    frame, arucos, robot_pos, angle = show_robot(frame, arucos, grid_resolution)

    projected_frame = projected_image(frame, arucos, width, height)
    
    goal_pos = get_goal_pos(arucos, grid_resolution)

    if goal_pos != (0, 0):
        last_known_goal_pos = goal_pos

    if check_if_goal_reached(arucos, robot_pos, last_known_goal_pos):
        print('Goal reached')
        break

    cv2.imshow("Video Stream", projected_frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
cap.release()

'''
image = cv2.imread('perspect.png')

frame, arucos = get_arucos(image)

frame = projected_image(image, arucos)

cv2.imshow("Video Stream", frame)

cv2.waitKey(0)
cv2.destroyAllWindows()
'''