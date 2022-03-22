from OutputController import OutputController
from OutputController import Mode
import time
from Order import Mode
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder

class Motor(object):
    def __init__(self,id,status):
        self.id=id;        
        self.insertOrder(InitOrder(status,0))
        
    def insertOrder(self,order):
        order.setID(self.id);
        OutputController.get_instance().insertOrder(order)