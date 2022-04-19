
from functools import partial

import sys
import cv2
import time


import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit


from threading import Thread
import threading

from System import System
from Robot import RealRobot
from Form import Form
from OutputController import OutputDone
from OutputController import OutputController

from multiprocessing import Process, Manager

def startSystem(stepQueue):
    #OutputController().setStepQueue(stepQueue)
    #system = System()
    robot = RealRobot()
    form = Form(robot)

if __name__ == '__main__':
    print("Start") 
    with Manager() as manager:
        stepQueue = manager.Queue()
        #p1 = Process(target=OutputDone, args=(stepQueue,))
        p2 = Process(target=startSystem,args=(stepQueue,))
        #p1.start()
        p2.start()

        #p1.join()
        p2.join()
    quit()
            
    cc = 0
    il = 0
    timerlabel = 0
    branchdata = []

    realsense_init()
