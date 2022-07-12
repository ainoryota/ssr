import time
from VideoStream import VideoStream
from PIL import Image,ImageTk #udo pip install pillow
import tkinter as tk
import cv2
import time

###
import platform
from functools import partial

import sys
import tkinter as tk
from PIL import Image,ImageTk #udo pip install pillow
from SSC_ImageRecognition12 import IR
from SSC_ImageRecognition12 import ResetLog
import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit
from ctypes import windll




class RealSense(object):
    def __init__(self,root,branch,data):
        self.root = root
        self.br = branch
        self.data = data
        self.vs = VideoStream()
        time.sleep(1)
        self.vs.start();

        
        time.sleep(1)
    
        img = Image.open('testResult/test.png')
        testImgA = ImageTk.PhotoImage(img)
        self.il = tk.Label(self.root,image=testImgA)
        self.il.grid(row=3, column=4,columnspan=30,rowspan=10)

        self.timerlabel = tk.Label(self.root,text="")
        self.timerlabel.grid(row=33, column=7)

        self.branchdata = []

        self.getRealsense()
       

    #@jit("f8[:,:]()")
    def getRealsense(self):
        start = time.time()
    
    
        accel = self.vs.acc
        gyro = self.vs.gyro
        color_image = self.vs.color_image
        depth_image=self.vs.depth_image

        #depth_image = cv2.resize(depth_image, (640, 360))
        #depth_image = depth_image[0:320,320:640]

        #depth_image =cv2.cvtColor((cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08,beta =0), cv2.COLORMAP_JET)),cv2.COLOR_RGB2GRAY)
        #a=cv2.convertScaleAbs(depth_image, alpha=0.08)
        #b=cv2.applyColorMap(a, cv2.COLORMAP_JET)
        #c = cv2.cvtColor(b,cv2.COLOR_BGR2RGB)
        #depth_image=b

        depth_image=np.asanyarray(self.vs.depth_image)
        depth_colormap = cv2.cvtColor(cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET),cv2.COLOR_BGR2RGB)

        ir_image1 = cv2.cvtColor(self.vs.ir_image1,cv2.COLOR_GRAY2RGB)
        ir_image2 = cv2.cvtColor(self.vs.ir_image2,cv2.COLOR_GRAY2RGB)

        

        #testImg=ir_image1
        testImg = np.vstack((np.hstack((ir_image1,ir_image2)), np.hstack((color_image,depth_colormap))))
        testImg = cv2.resize(testImg,dsize=(640,320))
        testImg = Image.fromarray(testImg)
        testImg = ImageTk.PhotoImage(testImg)

 
        self.il.configure(image=testImg)
        self.il.image = testImg
    
        self.root.after(10,self.getRealsense)