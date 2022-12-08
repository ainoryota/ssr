from functools import partial
import sys
import cv2
import time
import math
import pyrealsense2 as rs
import numpy as np
from numba import jit
from threading import Thread
import threading
from System import System
from Robot import RealRobot
from Form import Form
from OutputController import OutputDone
from OutputController import OutputController
from multiprocessing import Process, Manager
import tkinter as tk
import platform
from PIL import Image,ImageTk #udo pip install pillow
from ctypes import alignment, windll
import os
import queue

#root = tk.Tk()
#caps = []
#il = ""
#def StartWindow():
#    #ウィンドウの生成
#    global caps
#    global root
#    global il
    
#    wf = 0
#    wc = 1
#    wb = 0

#    root.title('開始画面')
#    root.geometry('1280x720+75+0')
#    label = tk.Label(root,text = "webF,webC,webB")
#    label.grid(row = 0, column =0, padx = 5, pady = 5)

#    caps = [(i,cv2.VideoCapture(i,cv2.CAP_DSHOW)) for i in range(12)]
#    idxList = []
#    for cap in caps:
#        idxList.append(cap[0])

#    img =
#    Image.open(os.path.abspath('C:/Users/MSD/Documents/GitHub/SSR/SSR-AutonomousDriveingControl/SSC-ImageRecognition/test.png'))
#    testImg = ImageTk.PhotoImage(img)

#    il = tk.Label(root,image=testImg)
#    il.grid(row=1, column=1,ipadx=5)


#    loop()
#    root.mainloop()

#    return(wf,wc,wb)

#def loop():
#    global caps
#    global root
#    global il

#    imgs = []

#    for (i,cap) in caps:
#        ret, frame = cap.read()
#        if(ret) :
#            img = cv2.cvtColor(cv2.resize(frame,(160,90)),cv2.COLOR_BGR2RGB)
#            img[0:50,0:50,:] = 0
#            imgs.append(cv2.putText(img,str(i),(10,40),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255)))

#    for idx in range(6 - len(imgs)):
#        imgs.append(np.zeros((90,160,3),np.uint8))

#    allImg0 = np.hstack((imgs[0],imgs[1],imgs[2]))
#    allImg1 = np.hstack((imgs[3],imgs[4],imgs[5]))



#    allImg = np.vstack((allImg0,allImg1))
#    allImg = Image.fromarray(allImg)
#    allImg = ImageTk.PhotoImage(allImg)
#    il.configure(image=allImg)
#    il.image = allImg

#    root.after(500,loop)
def startSystem(stepQueue):
    #OutputController().setStepQueue(stepQueue)
    system = System()
    robot = RealRobot()
    form = Form(robot)

if __name__ == '__main__':
    OutputController().msgPrint("SelectStart") 

    #(wf,wc,wb) = StartWindow()
    #OutputController().msgPrint("RobotStart")
    if False:
        with Manager() as manager:
            stepQueue = manager.Queue()
            p1 = Process(target=OutputDone, args=(stepQueue,))
            p2 = Process(target=startSystem,args=(stepQueue,))
            p1.start()
            p2.start()

            p1.join()
            p2.join()
        quit()
    else:
        stepQueue = queue.Queue()
        #thread1 = threading.Thread(target=OutputDone, args=(stepQueue,))
        #thread1.start()
        startSystem(stepQueue)
            
    cc = 0
    il = 0
    timerlabel = 0

