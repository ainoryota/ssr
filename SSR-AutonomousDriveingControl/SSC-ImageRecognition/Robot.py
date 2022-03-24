from abc import ABC
from Motor import Motor
import time
from Order import Mode
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder
from OutputController import OutputController
import numpy as np
import math

class Robot(ABC):
    """description of class"""
    def __init__(self):
        print("robot set")

class RealRobot(Robot):
    """description of class"""
    def __init__(self):
        print("realrobot set")
       
        self.motors = []

        self.motors.append(Motor(0,Mode.Pos))
        self.motors.append(Motor(1,Mode.Pos))
        self.motors.append(Motor(2,Mode.Pos))
        self.motors.append(Motor(3,Mode.Pos))
        self.motors.append(Motor(4,Mode.Velocity))
        self.motors.append(Motor(5,Mode.Velocity))
        self.motors.append(Motor(6,Mode.Pos))
        self.motors.append(Motor(7,Mode.Pos))
        self.motors.append(Motor(8,Mode.Pos))
        self.motors.append(Motor(9,Mode.Velocity))
        self.motors.append(Motor(10,Mode.Velocity))
        self.motors.append(Motor(11,Mode.Velocity))

        #初期姿勢
        data = np.loadtxt("Data/normal_switching.csv",delimiter=",")
        self.motors[0].insertOrder(PosOrder(0,0))
        self.motors[1].insertOrder(PosOrder(round(data[0,0] / math.pi * 180,2),0))
        self.motors[2].insertOrder(PosOrder(round(data[0,1] / math.pi * 180,2),0))
        self.motors[3].insertOrder(PosOrder(round(data[0,2] / math.pi * 180,2),0))
        self.motors[4].insertOrder(VelocityOrder(0,0))
        self.motors[5].insertOrder(VelocityOrder(0,0))
        self.motors[6].insertOrder(PosOrder(round(data[0,3] / math.pi * 180,2),0))
        self.motors[7].insertOrder(PosOrder(round(data[0,4] / math.pi * 180,2),0))
        self.motors[8].insertOrder(PosOrder(round(data[0,5] / math.pi * 180,2),0))
        self.motors[9].insertOrder(VelocityOrder(0,0))
        self.motors[10].insertOrder(VelocityOrder(0,0))
        self.motors[11].insertOrder(VelocityOrder(0,0))
        OutputController().pushStep()
        
