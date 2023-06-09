
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
import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit
from ctypes import alignment, windll
from RealSense import RealSense 
from WebCamera import WebCameraMgr 
import os
from Utilty import getImageFromFile
from OutputController import OutputController


class Camera(object):
    def __init__(self,imgArea,infArea,branch,data):
        self.imgArea = imgArea
        self.infArea = infArea
        self.br = branch
        self.data = data
    
        devices = rs.context().query_devices()
        max_n_device = 2
        serial_numbers = list(map(lambda device: device.get_info(rs.camera_info.serial_number), devices))[:max_n_device]
        #serial_numbers.sort()

        for rscam in serial_numbers:
            OutputController().msgPrint("realsenseList:",rscam)
        serial_numbers.sort()
        

        img = getImageFromFile("test.png")
        testImg = ImageTk.PhotoImage(img)

        self.il = tk.Label(self.imgArea,image=testImg)
        self.il.grid(row=1, column=1,columnspan=30,rowspan=20,ipadx=5)


        self.timerlabel = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.timerlabel.grid(row=5, column=1)

        self.AccelLabelX = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.AccelLabelX.grid(row=6, column=1)
        self.AccelLabelY = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.AccelLabelY.grid(row=7, column=1)
        self.AccelLabelZ = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.AccelLabelZ.grid(row=8, column=1)
        self.GyroLabelX = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.GyroLabelX.grid(row=9, column=1)
        self.GyroLabelY = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.GyroLabelY.grid(row=10, column=1)
        self.GyroLabelZ = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.GyroLabelZ.grid(row=11, column=1)
        self.RobotTheta = tk.Label(self.infArea,text="",anchor="w",width = 20)
        self.RobotTheta.grid(row=12, column=1)



        try:
            self.rs1 = RealSense(serial_numbers[0],self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)

        except Exception as e:
            OutputController().msgPrint("Realsense1 Start Error",e)
            self.rs1=None

        OutputController().msgPrint("Open realsense2")
        
        try:
            self.rs2 = RealSense(serial_numbers[1],self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)
        except Exception as e:
            OutputController().msgPrint("Realsense2 Start Error",e)
            self.rs2=None

        OutputController().msgPrint("Open webcam")
        try:
            raise ValueError("no cam mode")
            self.webcam = WebCameraMgr(self.imgArea,self.infArea,self.br,self.data,self.il)
        except Exception as e:
            OutputController().msgPrint("WebCamera Start Error",e)
            self.webcam=None

        OutputController().msgPrint("Open realsense1")
        #self.rs2=RealSense.RealSense(self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)
        #self.webcam=RealSense.RealSense(self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)



    def startRealsense1(self):
        self.stopRealsense2()
        self.stopWebCam()
        if(self.rs1!=None):self.rs1.start()
        else:OutputController().msgPrint("no rs1")

    def stopRealsense1(self):
        if(self.rs1!=None):self.rs1.stop()
        else:OutputController().msgPrint("no rs1")

    def startRealsense2(self):
        self.stopRealsense1()
        self.stopWebCam()
        if(self.rs2!=None):self.rs2.start()
        else:OutputController().msgPrint("no rs2")

    def stopRealsense2(self):
        if(self.rs2!=None):self.rs2.stop()
        else:OutputController().msgPrint("no rs2")

    def startWebCam(self):
        self.stopRealsense1()
        self.stopRealsense2()
        if(self.webcam!=None):self.webcam.start()
        else:OutputController().msgPrint("no webcam")

    def stopWebCam(self):
        if(self.webcam!=None):self.webcam.stop()
        else:OutputController().msgPrint("no webcam")