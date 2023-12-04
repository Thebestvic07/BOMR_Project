#%%
from A_star_alg import *

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

from data import *
from PIL import Image
import time


start_time = time.time()

PLOT = True         #set to true to show map with matplot

#seulement pour visuel de la map
def create_empty_plot(max_x, max_y):
    """
    Helper function to create a figure of the desired dimensions & grid
    
    :param max_val: dimension of the map along the x and y dimensions
    :return: the fig and ax objects.
    """
    fig, ax = plt.subplots(figsize=(7,7))
    

    major_x_ticks = np.arange(0, max_x+1, 5)
    minor_x_ticks = np.arange(0, max_x+1, 1)

    major_y_ticks = np.arange(0, max_y+1, 5)
    minor_y_ticks = np.arange(0, max_y+1, 1)

    ax.set_xticks(major_x_ticks)
    ax.set_xticks(minor_x_ticks, minor=True)
    ax.set_yticks(major_y_ticks)
    ax.set_yticks(minor_y_ticks, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    ax.set_ylim([-1,max_y])
    ax.set_xlim([-1,max_x])
    ax.grid(True)
    
    return fig, ax





#import map from image and set start/goal
im = Image.open('b.png', 'r')
im = im.convert('L')
grid = np.array(im)
grid = (grid !=255)*1
max_x, max_y = grid.shape[0], grid.shape[1] # Size of the map

goal = (1,7)
start = (19,7)



def create_grid(world):
    "Create the grid with obstacles from the environment"
    max_x = 0
    max_y = 0
    for corners in world.map.corners:
        if corners.x > max_x:
            max_x = corners.x
        if corners.y > max_y:
            max_y = corners.y
    
    goal = (world.goal.x, world.goal.y) #set goal and start as tupples
    start = (world.start.x, world.start.y)

    grid = np.zeros((max_x, max_y)) 
    for obs in world.map.obstacles:
        for pos in obs.summits:
            grid[pos.x][pos.y] = 1       #set obstacles as 1 in array
    
    return grid, start, goal
    

def map_without_collision(grid, size_thym):
    "add half size of thymio to the obstacles to avoid collisions"
    map = grid.copy()
    max_x, max_y = grid.shape[0], grid.shape[1] # Size of the map

    for i in range(max_x):          
        for j in range(max_y):
            if  grid[i,j]:                   
                for x in range(int(2*np.ceil(size_thym/2)+1)):      #set positions too close to obstacle as obstacles 
                    x -= int(np.ceil(size_thym/2))
                    for y in range(int(2*np.ceil(size_thym/2)+1)):
                        y -= int(np.ceil(size_thym/2))
                        if (0 <= i+x < max_x) and (0 <= j+y < max_y): #check if position is in grid
                            map[i+x,j+y] = 1

    return map

def convert_path(path):
    "convert the path given in as an array of size(n,2) as a list of Points"
    list_points = list()
    
    for i in range(np.size(path,1)):
        pt = Point(path[0][i], path[1][i])
        list_points.append(pt)

    return list_points

occupancy_grid = map_without_collision(grid, size_thym=5)




# -----------------------------------------
# DO NOT EDIT THIS PORTION OF CODE - 
# EXECUTION AND PLOTTING OF THE ALGORITHM
# -----------------------------------------
    
    
# List of all coordinates in the grid
x,y = np.mgrid[0:max_x:1, 0:max_y:1]
pos = np.empty(x.shape + (2,))
pos[:, :, 0] = x; pos[:, :, 1] = y
pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
coords = list([(int(x[0]), int(x[1])) for x in pos])

# Define the heuristic, here = distance to goal ignoring obstacles
h = np.linalg.norm(pos - goal, axis=-1)
h = dict(zip(coords, h))

# Run the A* algorithm
path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid)
path = np.array(path).reshape(-1, 2).transpose()


visitedNodes = np.array(visitedNodes).reshape(-1, 2).transpose()

#Creating the grid
if PLOT:
    cmap = colors.ListedColormap(['white', 'red']) # Select the colors with which to display obstacles and free cells
    fig, ax = create_empty_plot(max_x, max_y)
    # Displaying the map
    ax.imshow(grid.transpose(), cmap=cmap)
    plt.title("Map : free cells in white, occupied cells in red")
    # # Displaying the map
    fig_astar, ax_astar = create_empty_plot(max_x, max_y)
    ax_astar.imshow(occupancy_grid.transpose(), cmap=cmap)

    # Plot the best path found and the list of visited nodes
    ax_astar.scatter(visitedNodes[0], visitedNodes[1], marker="o", color = 'orange')
    ax_astar.plot(path[0], path[1], marker="o", color = 'blue')
    ax_astar.scatter(start[0], start[1], marker="o", color = 'green', s=200)
    ax_astar.scatter(goal[0], goal[1], marker="o", color = 'purple', s=200)

print(path)

# for i in range(path.shape[1]-1):
#     print(path[0][i+1]-path[0][i], path[1][i+1]-path[1][i])
    
def calculate_path(world, size_thym):
    """calls all functions to calculate path
       out: path given in a list of Points 
    """

    grid, start, goal = create_grid(world)
    grid = map_without_collision(grid, size_thym)

    # List of all coordinates in the grid
    x,y = np.mgrid[0:max_x:1, 0:max_y:1]
    pos = np.empty(x.shape + (2,))
    pos[:, :, 0] = x; pos[:, :, 1] = y
    pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
    coords = list([(int(x[0]), int(x[1])) for x in pos])

    # Define the heuristic, here = distance to goal ignoring obstacles
    h = np.linalg.norm(pos - goal, axis=-1)
    h = dict(zip(coords, h))

    # Run the A* algorithm
    path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid)
    path = np.array(path).reshape(-1, 2).transpose()

    visitedNodes = np.array(visitedNodes).reshape(-1, 2).transpose()

    path = convert_path(path)
    return path


print("--- %s seconds ---" % (time.time() - start_time))



# %%
