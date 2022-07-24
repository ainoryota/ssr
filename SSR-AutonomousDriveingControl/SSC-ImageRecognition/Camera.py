
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




class Camera(object):
    def __init__(self,imgArea,infArea,branch,data):
        self.imgArea = imgArea
        self.infArea = infArea
        self.br = branch
        self.data = data
    
        devices = rs.context().query_devices()
        max_n_device = 2
        serial_numbers = list(map(lambda device: device.get_info(rs.camera_info.serial_number), devices))[:max_n_device]

        for rscam in serial_numbers:
            print("realsenseList:",rscam)

        img = Image.open('test.png')
        testImg = ImageTk.PhotoImage(img)

        self.il = tk.Label(self.imgArea,image=testImg)
        self.il.grid(row=1, column=1,columnspan=30,rowspan=20,ipadx=5)

        self.timerlabel = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.timerlabel.grid(row=0, column=34,columnspan=5)

        self.AccelLabelX = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.AccelLabelX.grid(row=1, column=34,columnspan=5)
        self.AccelLabelY = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.AccelLabelY.grid(row=2, column=34,columnspan=5)
        self.AccelLabelZ = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.AccelLabelZ.grid(row=3, column=34,columnspan=5)


        self.GyroLabelX = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.GyroLabelX.grid(row=4, column=34,columnspan=5)
        self.GyroLabelY = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.GyroLabelY.grid(row=5, column=34,columnspan=5)
        self.GyroLabelZ = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.GyroLabelZ.grid(row=6, column=34,columnspan=5)
        self.RobotTheta = tk.Label(self.infArea,text="",anchor="w",width = 50)
        self.RobotTheta.grid(row=7, column=34,columnspan=5)

        print("Open webcam")
        self.webcam = WebCameraMgr(self.imgArea,self.infArea,self.br,self.data,self.il)

        print("Open realsense1")
        self.rs1 = RealSense(serial_numbers[0],self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)

        print("Open realsense2")
        self.rs2 = RealSense(serial_numbers[1],self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)


        #self.rs2=RealSense.RealSense(self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)
        #self.webcam=RealSense.RealSense(self.imgArea,self.infArea,self.br,self.data,self.il,self.timerlabel,self.AccelLabelX,self.AccelLabelY,self.AccelLabelZ,self.GyroLabelX,self.GyroLabelY,self.GyroLabelZ,self.RobotTheta)



    def startRealsense1(self):
        self.stopRealsense2()
        self.stopWebCam()
        self.rs1.start()

    def stopRealsense1(self):
        self.rs1.stop()

    def startRealsense2(self):
        self.stopRealsense1()
        self.stopWebCam()
        self.rs2.start()

    def stopRealsense2(self):
        self.rs2.stop()

    def startWebCam(self):
        self.stopRealsense1()
        self.stopRealsense2()
        self.webcam.start()

    def stopWebCam(self):
        self.webcam.stop()