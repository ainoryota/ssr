from abc import ABC
from Motor import Motor
import time
from Order import Mode
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder
from OutputController import OutputController

class Robot(ABC):
    """description of class"""
    def __init__(self):
        print("robot set")

class RealRobot(Robot):
    """description of class"""
    def __init__(self):
        print("realrobot set")
       
        self.motor0=Motor(0,Mode.Pos)
        self.motor2=Motor(2,Mode.Pos)
        OutputController().pushStep()

        t=0
        self.motor0.insertOrder(PosOrder(180,t));
        self.motor2.insertOrder(PosOrder(180,t));
        t+=2
        self.motor0.insertOrder(PosOrder(-180,t));
        self.motor2.insertOrder(PosOrder(-180,t));
        t+=2
        OutputController().pushStep()


        #self.motor3=Motor(3,0)
        #self.motor4=Motor(4,1)
        #self.motor5=Motor(5,1)
        #self.motor6=Motor(6,0)
        #self.motor7=Motor(7,0)
        #self.motor8=Motor(8,0)
        #self.motor9=Motor(9,1)
        #self.motor00=Motor(10,1)
        #self.motor01=Motor(11,1)
        
        #初期姿勢テスト
        

    
        #初期姿勢
        #data = np.loadtxt("Data/normal_switching.csv",delimiter=",")
        #Control.Motor_pos3(ser,1,2,3,round(data[0,0]/pi*180,2),round(data[0,1]/pi*180,2),round(data[0,2]/pi*180,2))
        #Control.Motor_pos3(ser,6,7,8,round(data[0,3]/pi*180,2),round(data[0,4]/pi*180,2),round(data[0,5]/pi*180,2))
