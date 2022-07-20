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
from SSC_ImageRecognition13 import ImageReconition
from SSC_ImageRecognition13 import IR
from SSC_ImageRecognition13 import ResetLog
import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit
from ctypes import alignment, windll




class RealSense(object):
    def __init__(self,root,branch,data):
        self.root = root
        self.br = branch
        self.data = data
        self.vs = VideoStream((640,360))
        time.sleep(1)
        self.vs.start();

        
        time.sleep(1)
    
        img = Image.open('testResult/test.png')
        testImgA = ImageTk.PhotoImage(img)
        self.il = tk.Label(self.root,image=testImgA)
        self.il.grid(row=3, column=4,columnspan=30,rowspan=10)



        self.timerlabel = tk.Label(self.root,text="",anchor="w",width = 50)
        self.timerlabel.grid(row=33, column=6,columnspan=5)

        self.AccelLabelX = tk.Label(self.root,text="",anchor="w",width = 50)
        self.AccelLabelX.grid(row=34, column=6,columnspan=5)
        self.AccelLabelY = tk.Label(self.root,text="",anchor="w",width = 50)
        self.AccelLabelY.grid(row=35, column=6,columnspan=5)
        self.AccelLabelZ = tk.Label(self.root,text="",anchor="w",width = 50)
        self.AccelLabelZ.grid(row=36, column=6,columnspan=5)


        self.GyroLableX = tk.Label(self.root,text="",anchor="w",width = 50)
        self.GyroLableX.grid(row=37, column=6,columnspan=5)
        self.GyroLableY = tk.Label(self.root,text="",anchor="w",width = 50)
        self.GyroLableY.grid(row=38, column=6,columnspan=5)
        self.GyroLableZ = tk.Label(self.root,text="",anchor="w",width = 50)
        self.GyroLableZ.grid(row=39, column=6,columnspan=5)



        self.branchdata = []

        self.getRealsense()
       

    #@jit("f8[:,:]()")
    def getRealsense(self):
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
        result=IR(color_image,depth_image,ir_image1,ir_image2,accel,False,self.data["scale"].get())

        testImg = result[0]
        testImg = Image.fromarray(testImg)
        testImg = ImageTk.PhotoImage(testImg)

    

        self.il.configure(image=testImg)
        self.il.image = testImg
    
        timer = math.floor((time.time() - start) * 1000)

        self.AccelLabelX.configure(text="加速度X:{0:.2f}".format(accel.x))
        self.AccelLabelY.configure(text="加速度Y:{0:.2f}".format(accel.y))
        self.AccelLabelZ.configure(text="加速度Z:{0:.2f}".format(accel.z))
        self.GyroLableX.configure(text="ジャイロX:{0:.2f}".format(gyro.x))
        self.GyroLableY.configure(text="ジャイロY:{0:.2f}".format(gyro.y))
        self.GyroLableZ.configure(text="ジャイロY:{0:.2f}".format(gyro.z))
        self.timerlabel.configure(text="処理時間:{0} ms".format(timer))
        self.branchdata.append([result[1],result[2],timer])

        print("★",'{:.2f}'.format(result[3]),result[1],result[8])
        if(result[3] > 0.13 and result[1] < 150):
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
                #Branch Angle:Exception in Tkinter callback 4:-5 5:-10 6:75 7:70 v1:True v3:True v7:True v8:False
                self.br.branchAngle(result[4],result[5],result[6],result[7],self.data["v1"].get(),self.data["v3"].get(),self.data["v7"].get(),self.data["v8"].get())
            



        self.root.after(10,self.getRealsense)