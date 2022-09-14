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

from SSC_ImageRecognition14 import ImageReconition
from SSC_ImageRecognition14 import IR
from SSC_ImageRecognition14 import ResetLog
import math
import pyrealsense2 as rs
import numpy as np
import os
import sys
from numba import jit
from ctypes import alignment, windll
from BranchSystem import BranchSystem 


def getLikeAngle(a1,b1,c1,d1):
    a = int(a1 / 5) * 5
    b = int(b1 / 5) * 5
    c = int(c1 / 5) * 5
    d = int(d1 / 5) * 5

    path = str(a) + "_" + str(b) + "_" + str(c) + "_" + str(d) + ".csv"
    if(os.path.exists(path)):return (a,b,c,d)

    a = int(a1 / 10) * 10
    b = int(b1 / 10) * 10
    c = int(c1 / 10) * 10
    d = int(d1 / 10) * 10


    path = str(a) + "_" + str(b) + "_" + str(c) + "_" + str(d) + ".csv"
    if(os.path.exists(path)):return (a,b,c,d)


    a = int(a1 / 10) * 10 + 10
    b = int(b1 / 10) * 10
    c = int(c1 / 10) * 10
    d = int(d1 / 10) * 10


    path = str(a) + "_" + str(b) + "_" + str(c) + "_" + str(d) + ".csv"
    if(os.path.exists(path)):return (a,b,c,d)


    a = int(a1 / 10) * 10
    b = int(b1 / 10) * 10 + 10
    c = int(c1 / 10) * 10
    d = int(d1 / 10) * 10


    path = str(a) + "_" + str(b) + "_" + str(c) + "_" + str(d) + ".csv"
    if(os.path.exists(path)):return (a,b,c,d)


    a = int(a1 / 10) * 10
    b = int(b1 / 10) * 10
    c = int(c1 / 10) * 10 + 10
    d = int(d1 / 10) * 10


    path = str(a) + "_" + str(b) + "_" + str(c) + "_" + str(d) + ".csv"
    if(os.path.exists(path)):return (a,b,c,d)


    a = int(a1 / 10) * 10
    b = int(b1 / 10) * 10
    c = int(c1 / 10) * 10
    d = int(d1 / 10) * 10 + 10


    path = str(a) + "_" + str(b) + "_" + str(c) + "_" + str(d) + ".csv"
    if(os.path.exists(path)):return (a,b,c,d)

    print("out ofj range angle")
    return (a,b,c,d)

class RealSense(object):
    def __init__(self,serialNo,imgArea,infArea,branch,data,il,timerlabel,AccelLabelX,AccelLabelY,AccelLabelZ,GyroLabelX,GyroLabelY,GyroLabelZ,RobotTheta):
        self.serialNo = serialNo
        self.imgArea = imgArea
        self.infArea = infArea
        self.br = branch
        self.data = data
        self.vs = VideoStream(serialNo,(640,360))
        self.il = il
        self.timerlabel = timerlabel
        self.AccelLabelX = AccelLabelX
        self.AccelLabelY = AccelLabelY
        self.AccelLabelZ = AccelLabelZ
        self.GyroLabelX = GyroLabelX
        self.GyroLabelY = GyroLabelY
        self.GyroLabelZ = GyroLabelZ
        self.RobotTheta = RobotTheta
        self.StopFlag = True
        self.branchSystem=BranchSystem()
        print("Open realsense",self.serialNo)
       
    def start(self):
        self.StopFlag = False
        time.sleep(1)
        self.vs.start()
        time.sleep(1)
        self.getRealsense()

    def stop(self):
        self.StopFlag = True
        self.vs.stop()

    #@jit("f8[:,:]()")
    def getRealsense(self):
        if(self.StopFlag == True):return
        start = time.time()
    
        accel = self.vs.acc
        gyro = self.vs.gyro
        color_image = self.vs.color_image
        depth_image = self.vs.depth_image
        ir_image1 = self.vs.ir_image1
        ir_image2 = self.vs.ir_image2

        self.AccelLabelX.configure(text="加速度X:{0:.2f}".format(accel.x))
        self.AccelLabelY.configure(text="加速度Y:{0:.2f}".format(accel.y))
        self.AccelLabelZ.configure(text="加速度Z:{0:.2f}".format(accel.z))
        self.GyroLabelX.configure(text="ジャイロX:{0:.2f}".format(gyro.x))
        self.GyroLabelY.configure(text="ジャイロY:{0:.2f}".format(gyro.y))
        self.GyroLabelZ.configure(text="ジャイロY:{0:.2f}".format(gyro.z))
        self.RobotTheta.configure(text="ロボットの角度(deg):{0:.2f} ".format(math.degrees(-math.atan2(accel.y,accel.z))))

        (IsBranch,SleepTime,InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle)=self.branchSystem.getBranch()
        self.branchSystem.setImage(color_image,depth_image,ir_image1,ir_image2)
        self.branchSystem.calcCablewayInf(accel,True)
        testImg=self.branchSystem.getOutputImage()
        self.il.configure(image=testImg)
        self.il.image = testImg
        self.timerlabel.configure(text="処理時間:{0} ms".format(math.floor((time.time()-start)*1000)))

        if(IsBranch):
            print("■■■■■分岐",LElevationAngle,RElevationAngle,Langle,Rangle)
            if(self.data["v_auto"].get()):
                self.branchSystem.ResetLog()
                time.sleep(SleepTime)
                InclinationAngle =int(max(-20,InclinationAngle)//5*5)
                InclinationAngle =int(min(20,InclinationAngle)//5*5)
                ElevationAngle=int(max(-20,ElevationAngle)//5*5)
                ElevationAngle=int(min(20,ElevationAngle)//5*5)
                RTurningAngle =int(max(-80,RTurningAngle)//5*5)
                RTurningAngle =int(min(80,RTurningAngle)//5*5)
                LTurningAngle =int(max(-80,LTurningAngle)//5*5)
                LTurningAngle =int(min(80,LTurningAngle)//5*5)
                print("Branch Angle:",InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle,self.data["v1"].get(),self.data["v3"].get(),self.data["v7"].get(),self.data["v8"].get())
                self.br.branchAngle(InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle,self.data["v1"].get(),self.data["v3"].get(),self.data["v7"].get(),self.data["v8"].get())

        self.imgArea.after(10,self.getRealsense)