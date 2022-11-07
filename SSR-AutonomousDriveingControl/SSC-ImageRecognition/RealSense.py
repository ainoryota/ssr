import time
from VideoStream import VideoStream
from PIL import Image,ImageTk #udo pip install pillow
import tkinter as tk
import cv2
import time

import platform
from functools import partial
import sys
import math
import pyrealsense2 as rs
import numpy as np
import os
import sys
from numba import jit
from ctypes import alignment, windll
from BranchSystem import BranchSystem 
from Utilty import getLikeAngle,rounddown
from OutputController import OutputController
from FormSingleton import FormSingleton


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
        self.branchSystem = BranchSystem()
        self.stop_branch_time = 0

        OutputController().msgPrint("Open realsense",self.serialNo)
       
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

        (IsBranch,SleepTime,InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle) = self.branchSystem.getBranch()

        (h,w) = depth_image.shape

        OutputController().msgPrint("value")
        hoge = np.where((depth_image < 500) & (depth_image > 200))
        OutputController().msgPrint(hoge)
        


        self.branchSystem.setImage(color_image,depth_image,ir_image1,ir_image2)
        rule1,rule2=self.branchSystem.calcCablewayInf(accel,True)
        testImg = self.branchSystem.getOutputImage()
        self.il.configure(image=testImg)
        self.il.image = testImg
        self.timerlabel.configure(text="処理時間:{0} ms".format(math.floor((time.time() - start) * 1000)))
        tangle = self.branchSystem.tangle
        InclinationAngle = self.branchSystem.InclinationAngle
        Rangle = self.branchSystem.Rangle
        Langle = self.branchSystem.Langle


        fit = FormSingleton().updateThreeGraph(hoge[1],hoge[0],depth_image[hoge])
        
        OutputController().msgPrint(tangle,-fit[0],-fit[1],math.degrees(math.atan(-fit[0])),math.degrees(math.atan(-fit[1])))
        OutputController().msgPrint("仰角（ロボットのみ）=",tangle)
        OutputController().msgPrint("仰角（ケーブルのみ）=",math.degrees(math.atan(-fit[1])))
        OutputController().msgPrint("仰角（両方）=",tangle + math.degrees(math.atan(-fit[1])))
        OutputController().msgPrint("回転角Inc=",rounddown(math.degrees(math.atan(-fit[0])),1))
        OutputController().msgPrint("RightAngle=",rounddown(Rangle,0))
        OutputController().msgPrint("LAngle=",rounddown(Langle,0))
        OutputController().msgPrint("rule1=",rounddown(rule1,2))
        OutputController().msgPrint("rule2=",rounddown(rule2,2))
        tangle = tangle + math.degrees(math.atan(-fit[1]))
        InclinationAngle = math.degrees(math.atan(-fit[0]))
        

        OutputController().msgPrint("time:",self.stop_branch_time)
        if(self.stop_branch_time > 0):
            self.stop_branch_time-=1
        elif(IsBranch):
            OutputController().msgPrint("■■■■■分岐",tangle,InclinationAngle,Rangle,Langle)
            (tangle,InclinationAngle,Rangle,Langle) = getLikeAngle(tangle,InclinationAngle,Rangle,Langle)
            if((InclinationAngle,tangle,Rangle,Langle) != (0,0,0,0)):
                OutputController().msgPrint("■■■■■■Like:",tangle,InclinationAngle,Rangle,Langle)

                if(self.data["v_auto"].get()):
                    self.branchSystem.ResetLog()
                    time.sleep(SleepTime)
                    OutputController().msgPrint("Branch Angle:",ElevationAngle,InclinationAngle,Rangle,Langle,self.data["v1"].get(),self.data["v3"].get(),self.data["v_tention"].get(),self.data["v8"].get())
                    self.br.branchAngle(ElevationAngle,InclinationAngle,Rangle,Langle,self.data["v1"].get(),self.data["v3"].get(),self.data["v_tention"].get(),self.data["v8"].get())
                    self.stop_branch_time = 70
            else:
                OutputController().msgPrint("No like data")

        self.imgArea.after(10,self.getRealsense)