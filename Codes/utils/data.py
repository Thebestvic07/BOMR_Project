import numpy as np
from dataclasses import dataclass, field
# Source : https://docs.python.org/fr/3/library/dataclasses.html#module-dataclasses

@dataclass
class Point:      
    """ Object to caracterize a position """
    x : float
    y : float
    
    def pos(self) -> np.ndarray:
        return np.array([self.x, self.y]) 

    def dist(self, other) -> float:
        return np.linalg.norm(self.pos() - other.pos())
    
    def copy(self):
        return Point(self.x, self.y)


@dataclass
class Map:
    """ Object to caracterize a map """
    corners : list[Point]
    obstacles : list[Point]

@dataclass
class Robot:
    """ Object to caracterize the Thymio in space """
    position : Point
    direction : float       #angle from x direction (->) 

    def copy(self):
        return Robot(self.position.copy(), self.direction, self.covariance.copy())

@dataclass
class Environment:
    """ Object to caracterize the full environment """
    robot : Robot
    map   : Map
    goal  : Point

@dataclass
class Motors:
    """ Object to describe Thymio's motor speeds """
    left  : int = 0
    right : int = 0

@dataclass
class Target:
    """ Object to describe Thymio's target motor speeds """
    left  : int = 0
    right : int = 0

@dataclass
class Sensors:
    """ Object to regroup Thymio's prox & ground sensors """
    prox   : list[int]   # 7 int list
    ground : list[int]   # 2 int list

    def left(self):
        return self.prox[0]
    
    def center(self):
        return self.prox[2]

    def right(self):
        return self.prox[4]
    
    def behind_l(self):
        return self.prox[5]
    
    def behind_r(self):
        return self.prox[6]
    

@dataclass 
class Lights:
    """ Object to caracterize Thymio's top lights """
    color : list[int]  # 3 int list