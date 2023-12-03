from tdmclient import ClientAsync, aw
from data import *
import numpy as np
import time 

class Thymio:
    client = None
        
    def __init__(self):
        self.node = None

        self.sensors = Sensors([0,0,0,0,0,0,0],[0,0])
        self.motors = Motors(0,0)
        self.leds = Lights([0,0,0])

        self.situation = Robot(Point(0,0), 0)
    
        self.start()

    def start(self):
        """
        Init asynchronous connection 
        """
        client = ClientAsync()
        try: 
            self.node = aw(client.wait_for_node())
            aw(self.node.lock())
            print("Connected to Thymio !")
         
            self.read_variables()
            self.get_position()
            self.set_variable(Lights([0,32,0]))
    
        except:
            print("Connection to Thymio failed!")

    def stop(self):
        """
        Stop the robot + break connection
        """
        if self.node == None:
            raise "Thymio not connected"
    
        self.set_variable(Motors(0,0))
        self.set_variable(Lights([0,0,0]))
        aw(self.node.unlock())

    def set_variable(self, variable):
        """
        Set specified variable to its value
        Variable can either be Motors or Lights
        """
        if self.node == None:
            raise "Thymio not connected"
        
        if variable.__class__ is Motors:
            v = {
                "motor.left.target": [variable.left],
                "motor.right.target": [variable.right],
            }    
    
        elif variable.__class__ is Lights:
            v = {"leds.top": variable.color}
        
        else:
            raise "unknown command"
        
        aw(self.node.set_variables(v))
        self.read_variables()

        
    def read_variables(self, data=None):
    def read_variables(self, data = None):
        """
        Update sensor variables
        """
        if self.node == None:
            raise "Thymio not connected"

        aw(self.node.wait_for_variables({"prox.horizontal", "prox.ground.delta", "motor.left.speed", "motor.right.speed", "leds.top"}))

        self.sensors = Sensors(
            self.node.var["prox.horizontal"],
            self.node.var["prox.ground.delta"]
        )
        
        self.motors = Motors(
            self.node.var["motor.left.speed"],
            self.node.var["motor.right.speed"]
        )
    
        self.leds = Lights(self.node.var["leds.top"])

        if data != None :
            data.append([self.sensors, self.motors, self.leds])


    def get_position(self):
        """
        Return estimated position
        """
        if self.node == None:
            raise "Thymio not connected"



