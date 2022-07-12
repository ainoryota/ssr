import time
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder
from OutputController import OutputController

class Motor(object):
    def __init__(self,id,status):
        self.id=id;     
        self.status=status;
        self.insertOrder(InitOrder(status,0))
        
    def insertOrder(self,order):
        order.setID(self.id);
        OutputController().insertOrder(order)