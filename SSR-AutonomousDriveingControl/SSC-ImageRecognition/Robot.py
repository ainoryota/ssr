from abc import ABC
from Motor import Motor
import time
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder
from Order import MotorModeOrder
from OutputController import OutputController
from OutputController import MotorMode


import math

class Robot(ABC):
    """description of class"""
    def __init__(self):
        OutputController().msgPrint("robot set")

class RealRobot(Robot):
    """description of class"""
    def __init__(self):
        OutputController().msgPrint("realrobot set")
       
        self.motors = []
        self.motors.append(Motor(0,MotorMode.Pos))
        self.motors.append(Motor(1,MotorMode.Pos))
        self.motors.append(Motor(2,MotorMode.Pos))
        self.motors.append(Motor(3,MotorMode.Pos))
        self.motors.append(Motor(4,MotorMode.Velocity))
        self.motors.append(Motor(5,MotorMode.Velocity))
        self.motors.append(Motor(6,MotorMode.Pos))
        self.motors.append(Motor(7,MotorMode.Pos))
        self.motors.append(Motor(8,MotorMode.Pos))
        self.motors.append(Motor(9,MotorMode.Velocity))
        self.motors.append(Motor(10,MotorMode.Velocity))
        self.motors.append(Motor(11,MotorMode.Velocity))


        
