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


        
        #print("color:",color_image.shape)
        #print("depth;",depth_image.shape)
        #print("IR1:",ir_image1.shape)
        #print("IR2:",ir_image2.shape)
        result = IR(color_image,depth_image,ir_image1,accel,True)
        if(len(result)>1):#不正ならFalseだけが返る
            # return
            # [CreteViewImage(color_image,depth_image,ir_image,brendImage,cvpaste(imageMap,
            # np.zeros(ElevationImage.shape), 0, 0,
            # 0,1),ElevationImage),y,x,rule1,rule2,rule3,LElevationAngle,RElevationAngle,angleA,angleB]


            testImg = result[0]
            testImg = Image.fromarray(testImg)
            testImg = ImageTk.PhotoImage(testImg)

            y = result[1]
            x = result[2]
            rule1 = result[3]
            rule3 = result[4]
            LElevationAngle = result[5]
            RElevationAngle = result[6]
            LRE = result[7]
            Rangle = result[8]
            Langle = result[9]

            self.il.configure(image=testImg)
            self.il.image = testImg
    
            timer = math.floor((time.time() - start) * 1000)

            self.AccelLabelX.configure(text="加速度X:{0:.2f}".format(accel.x))
            self.AccelLabelY.configure(text="加速度Y:{0:.2f}".format(accel.y))
            self.AccelLabelZ.configure(text="加速度Z:{0:.2f}".format(accel.z))
            self.GyroLabelX.configure(text="ジャイロX:{0:.2f}".format(gyro.x))
            self.GyroLabelY.configure(text="ジャイロY:{0:.2f}".format(gyro.y))
            self.GyroLabelZ.configure(text="ジャイロY:{0:.2f}".format(gyro.z))
            self.timerlabel.configure(text="処理時間:{0} ms".format(timer))
            self.RobotTheta.configure(text="ロボットの角度(deg):{0:.2f} ".format(math.degrees(-math.atan2(accel.y,accel.z))))

            print()

            print("★rule",rule1,rule3,":","Place",(y,x),"GammaAngle",math.degrees(-math.atan2(accel.y,accel.z)),"ElevationAngle",LRE,"TurnAngle",Langle,Rangle)



            if(rule1 > 1 and rule3 > 1):
                tangle=math.degrees(-math.atan2(accel.y,accel.z))
                (tangle,LRE,angleA,angleB) = getLikeAngle(tangle,LRE,Rangle,Langle)
                print("■■■■■分岐",result[4],result[5],result[6],result[7],result[8])
                if(self.data["v_auto"].get()):
                    SleepLength = 0
                    TimeCounter = 1
                    for i in range(4):
                        SleepLength+=max(0,result[8][i] - result[8][i + 1])
                        if(result[8][i] - result[8][i + 1] > 0):TimeCounter+=700
                    #TimeCounterでSleepLengthだけ進んでいる
                    SleepVel = 1000 * SleepLength / TimeCounter

                    SleepTime = result[1] / SleepVel
                    SleepTime = max(0,SleepTime)

                    print("Sleep",SleepTime)
                    ResetLog()


                    time.sleep(SleepTime)
                    result[4] = max(-20,result[4])
                    result[4] = min(20,result[4])
                    result[5] = max(-20,result[5])
                    result[5] = min(20,result[5])
                    result[6] = max(-80,result[6])
                    result[6] = min(80,result[6])
                    result[7] = max(-80,result[7])
                    result[7] = min(80,result[7])
                    print("Branch Angle:",result[4],result[5],result[6],result[7],self.data["v1"].get(),self.data["v3"].get(),self.data["v7"].get(),self.data["v8"].get())
                    #Branch Angle:Exception in Tkinter callback 4:-5 5:-10 6:75
                    #7:70 v1:True v3:True v7:True v8:False
                    self.br.branchAngle(result[4],result[5],result[6],result[7],self.data["v1"].get(),self.data["v3"].get(),self.data["v7"].get(),self.data["v8"].get())


        self.imgArea.after(10,self.getRealsense)