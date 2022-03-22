from abc import ABC
from enum import Enum

class Order(ABC):
    def __init__(self,delay,sleepTime):
        self.id = -1
        self.delay = delay
        self.sleepTime = sleepTime

    def setID(self,id):
        self.id=id;


class InitOrder(Order):
    def __init__(self,mode,delay):
        super().__init__(delay,0.3)
        self.mode = mode
        self.gain = 0X00


class PosOrder(Order):#角度はdegree
    def __init__(self,pos,delay,sleepTime=0.01):
        super().__init__(delay,sleepTime)
        self.pos = pos

class VelocityOrder(Order):
    def __init__(self,velocity,delay,sleepTime=0.01):
        super().__init__(delay,sleepTime)
        self.velocity = velocity

class Mode(Enum):
    Pos = 0
    Velocity = 1