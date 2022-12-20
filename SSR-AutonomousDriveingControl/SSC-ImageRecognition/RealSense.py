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
from Utilty import getLikeAngle,rounddown,ConvertDepthCoordinate,DebugImage
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
        self.web_image = np.zeros((480,640,3))
        self.webcam = None
        self.tangleLog = [0 for n in range(10)]
        self.InclinationLog = [0 for n in range(10)]
        self.RAngleLog = [0 for n in range(10)]
        self.LAngleLog = [0 for n in range(10)]
        self.MaxValue1Log = [0 for n in range(10)]
        self.MaxValue2Log = [0 for n in range(10)]
        self.MaxValue3Log = [0 for n in range(10)]
        self.valueLog = [0 for n in range(10)]
        self.timerLog = [0 for n in range(100)]
        self.startTime = time.time()

        if(False):
            try:
                okcamera = []
                OutputController().msgPrint("Camera List")
                for i in range(10):
                    try:
                        self.capture = cv2.VideoCapture(i,cv2.CAP_DSHOW)
                        ret, frame = self.capture.read()
                        self.capture.release()
                        OutputController().msgPrint(i,ret,frame.shape)
                        okcamera.append(i)
                    except:
                        pass

                self.webcam = cv2.VideoCapture(okcamera[1],cv2.CAP_DSHOW)
                OutputController().msgPrint("Web camera start",okcamera[1])
            except:
                OutputController().msgPrint("Web camera open error")

        OutputController().msgPrint("Open realsense",self.serialNo)
       
    def ResetLog(self):
        for data in self.valueLog:
            data = 0
        return 0

    def start(self):
        self.StopFlag = False
        time.sleep(1)
        self.vs.start()
        time.sleep(1)
        self.getRealsense()

    def stop(self):
        self.StopFlag = True
        self.vs.stop()

    def getAverageLog(self):
        hoge = np.where(np.array(self.valueLog) > 750)
        if(len(hoge[0]) > 1):
            InclinationAngle = 0
            Rangle = 0
            Langle = 0
            tangle = 0
            for i in range(len(hoge[0])):
                n = len(hoge[0])
                gain = float(i + 1) ** 2 / ((n * (n + 1) * (2 * n + 1)) / 6)
                InclinationAngle +=np.array(self.InclinationLog)[hoge][i] * gain
                Rangle += np.array(self.RAngleLog)[hoge][i] * gain
                Langle += np.array(self.LAngleLog)[hoge][i] * gain
                tangle += np.array(self.tangleLog)[hoge][i] * gain

            return (round(tangle),round(InclinationAngle),round(Rangle),round(Langle))
        else:     return (0,0,0,0)


    #@jit("f8[:,:]()")
    def getRealsense(self):
        if(self.StopFlag == True):return
        start = time.time()

        accel = self.vs.acc
        gyro = self.vs.gyro
        color_image = self.vs.color_image
        depth_image_original = self.vs.depth_image.astype(np.int16)

        self.timerLog.pop(0)
        self.timerLog.append(time.time() - self.startTime)

        
        DebugImage(depth_image_original,0,True)
        depth_image = np.zeros((360,640),dtype=np.int16)
        Y_all, X_all = np.mgrid[:360, :640]
        Y_all = Y_all * 1.333
        X_all = X_all * 1.325
        Y_all = Y_all.flatten().astype(int)
        X_all = X_all.flatten().astype(int)

        depth_image = depth_image_original[(Y_all,X_all)]
        depth_image = np.reshape(depth_image,((360,640)))

        hoge = np.where((depth_image < 600) & (depth_image > 0))

        print("A",time.time() - start)
        result,x,y,z,fit = FormSingleton().updateThreeGraph(hoge[1],hoge[0],depth_image[hoge])
        if(result ==False):
            print("Graph Error")
            self.imgArea.after(100,self.getRealsense)
            return
        
        print("L",time.time() - start)
        depth_image.fill(0)
        mask = (x < 640) & (y < 360) & (x > 0) & (y > 0)

        depth_image[y.astype(int)[mask],x.astype(int)[mask]] = z[mask]

        DebugImage(depth_image,1,True)
        

        
        ir_image1 = self.vs.ir_image1
        ir_image2 = self.vs.ir_image2
        if(self.webcam != None):
            ret, frame = self.webcam.read()
            if(ret): self.web_image = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)


        self.branchSystem.setImage(color_image,depth_image,ir_image1,ir_image2,self.web_image)


        value,rule0,rule1,rule2,rule5,rule6,rule7,a = self.branchSystem.calcCablewayInf(accel,self.timerLog,True)


        rule3_L = np.var(self.LAngleLog)
        rule3_R = np.var(self.RAngleLog)
        nonzero1 = np.where(np.array(self.MaxValue1Log) != 0)
        if(len(nonzero1[0]) == 0):
            rule4 = 0
        else:
            nonzero2 = np.where(np.array(self.MaxValue2Log)[nonzero1] != 0)
            if(len(nonzero2[0]) == 0):
                rule4 = 0
            else:
                rule4 = np.var(np.array(self.MaxValue1Log)[nonzero1][nonzero2] / np.array(self.MaxValue2Log)[nonzero1][nonzero2]) * 1000

        

        testImg = self.branchSystem.getOutputImage()
        self.il.configure(image=testImg)
        self.il.image = testImg

        Rangle = self.branchSystem.Rangle - (90 - self.branchSystem.ProceedAngle)
        Langle = self.branchSystem.Langle + (90 - self.branchSystem.ProceedAngle)
        

        robot_tangle = math.degrees(-math.atan2(accel.y,accel.z))
        cable_angle = math.degrees(math.atan(-fit[1]))
        tangle = robot_tangle + cable_angle
        InclinationAngle = math.degrees(math.atan(-fit[0]))

        self.valueLog.pop(0)
        self.valueLog.append(value)
        self.tangleLog.pop(0)
        self.tangleLog.append(tangle)
        self.InclinationLog.pop(0)
        self.InclinationLog.append(InclinationAngle)
        self.RAngleLog.pop(0)
        self.RAngleLog.append(Rangle)
        self.LAngleLog.pop(0)
        self.LAngleLog.append(Langle)
        self.MaxValue1Log.pop(0)
        self.MaxValue1Log.append(self.branchSystem.maxValue1)
        self.MaxValue2Log.pop(0)
        self.MaxValue2Log.append(self.branchSystem.maxValue2)
        self.MaxValue3Log.pop(0)
        self.MaxValue3Log.append(self.branchSystem.maxValue3)
        

        self.timerlabel.configure(text="処理時間:{0} ms".format(math.floor((time.time() - start) * 1000)))
        self.AccelLabelX.configure(text="加速度X:{0:.2f}".format(accel.x))
        self.AccelLabelY.configure(text="加速度Y:{0:.2f}".format(accel.y))
        self.AccelLabelZ.configure(text="加速度Z:{0:.2f}".format(accel.z))
        self.GyroLabelX.configure(text="ジャイロX:{0:.2f}".format(gyro.x))
        self.GyroLabelY.configure(text="ジャイロY:{0:.2f}".format(gyro.y))
        self.GyroLabelZ.configure(text="ジャイロZ:{0:.2f}".format(gyro.z))
        self.RobotTheta.configure(text="ロボットの角度(deg):{0:.2f} ".format(robot_tangle))
        
        #OutputController().msgPrint("---------")
        #OutputController().msgPrint("{:<10} {:<10}
        #{:<10}".format('ロボット仰角','ケーブル仰角','仰角'))
        #OutputController().msgPrint("{:<16.2f} {:<16.2f}
        #{:<12.2f}".format(robot_tangle,cable_angle,tangle))
        #OutputController().msgPrint("{:<10} {:<10}
        #{:<10}".format('回転角','RAngle','LAngle'))
        #OutputController().msgPrint("{:<13.2f} {:<10.2f}
        #{:<10.2f}".format(InclinationAngle,Rangle,Langle))
        #OutputController().msgPrint("{:<10}
        #{:<10}".format('rule1','rule2',"IsBranch"))
        #OutputController().msgPrint("{:<10.2f}
        #{:<10.2f}".format(rule1,rule2,IsBranch))
        #OutputController().msgPrint("time:",self.stop_branch_time)
        FormSingleton().updateForm()

        OutputController().msgPrint("■CurrentBranch",round(tangle,2),round(InclinationAngle,2),round(Rangle,2),round(Langle,2))
        (tangle,InclinationAngle,Rangle,Langle) = self.getAverageLog()
        OutputController().msgPrint("■AverageBranch",tangle,InclinationAngle,Rangle,Langle)
        OutputController().msgPrint("○rule","value=",round(value),"rule0=",round(rule0,2),"rule1=",round(rule1,2),"rule2=",round(rule2,2),"rule3L=",round(rule3_L),"rule3R=",round(rule3_R),"rule4=",round(rule4),"a=",round(a,2),"rule5=",round(rule5,2),"rule6=",round(rule6,2),"rule7=",round(rule7,2),"stop_time=",self.stop_branch_time)
        OutputController().msgPrint("○rule","v1=",round(self.branchSystem.maxValue1),"v2=",round(self.branchSystem.maxValue2,2),"v3=",round(self.branchSystem.maxValue3,2))

        if(self.stop_branch_time > 0):
            self.stop_branch_time-=1
        elif(rule0 > 0.9 and rule1 > 1 and rule2 > 1 and rule5 > 0.5 and rule6 > 0.5 and rule7 > 0):#and rule3_L < 10000 and rule3_R < 10000
            OutputController().msgPrint("■■■■■分岐",tangle,InclinationAngle,Rangle,Langle)
            (tangle,InclinationAngle,Rangle,Langle) = getLikeAngle(tangle,InclinationAngle,Rangle,Langle)
            if((InclinationAngle,tangle,Rangle,Langle) != (0,0,0,0)):
                OutputController().msgPrint("■■■■■■Like:",tangle,InclinationAngle,Rangle,Langle)

                if(self.data["v_auto"].get()):
                    self.branchSystem.ResetLog()
                    self.ResetLog()
                    time.sleep(1)
                    #OutputController().msgPrint("Branch
                    #Angle:",ElevationAngle,InclinationAngle,Rangle,Langle,self.data["v1"].get(),self.data["v3"].get(),self.data["v_tention"].get(),self.data["v8"].get())
                    self.br.branchAngle(a,tangle,InclinationAngle,Rangle,Langle,self.data["v1"].get(),self.data["v3"].get(),self.data["v_tention"].get(),self.data["v8"].get())
                    self.stop_branch_time = 15
            else:
                OutputController().msgPrint("No like data")


        self.imgArea.after(100,self.getRealsense)