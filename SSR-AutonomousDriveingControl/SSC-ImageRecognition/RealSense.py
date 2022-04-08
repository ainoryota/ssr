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
from SSC_ImageRecognition7 import ImageReconition
from SSC_ImageRecognition7 import ResetLog
import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit
from ctypes import windll




class RealSense(object):
    def __init__(self,root,branch):
        self.root=root
        self.br=branch
        self.vs = VideoStream()
        time.sleep(1)
        self.vs.start_imu()
        self.vs.start_camera()
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
        minDistance = 0
        maxDistance = 1500
    
    
        accel = self.vs.acc
        gyro = self.vs.gyro
        color_image = self.vs.color_image
        depth_image = self.vs.depth_image


        depth_image = np.where(depth_image > 5000,0,depth_image)#視認性の観点から5000以上なら0に
        original_depth_colormap = cv2.resize(cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET),(640,360)).copy()
        for p in list(zip(*np.where(depth_image == 0))):
            original_depth_colormap[p] = [0,0,0]

        depth_image = np.where(depth_image > maxDistance,0,depth_image)#索道らしくない距離を除去

        #これがないと型が合わない。例えばdepth_colormap=depth_image-minDistance;はだめ
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0), cv2.COLORMAP_JET)

        for x in range(640):
            for y in range(360):
                depth_colormap[y][x] = max(0,depth_image[y][x] - minDistance) * (255 / (maxDistance - minDistance))

        for p in list(zip(*np.where(depth_image == 0))):
            depth_colormap[p] = [0,0,0]

        #画像表示
        color_image_s = cv2.resize(color_image, (640, 360))
        depth_colormap_s = cv2.resize(depth_colormap, (640, 360))

        images = np.hstack((depth_colormap_s,original_depth_colormap))
    
        img = images
    

        image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # imreadはBGRなのでRGBに変換
        image_pil = Image.fromarray(image_rgb) # RGBからPILフォーマットへ変換
        testImg = ImageTk.PhotoImage(image_pil)

        result = ImageReconition(image_rgb,accel)
        testImg = image_rgb
        testImg = result[0]
        testImg = cv2.resize(testImg,dsize=(640,360))


        testImg = Image.fromarray(testImg)
        testImg = ImageTk.PhotoImage(testImg)

    

        self.il.configure(image=testImg)
        self.il.image = testImg
    
        timer = math.floor((time.time() - start) * 1000)
        self.timerlabel.configure(text="{0} ms".format(timer))
        self.branchdata.append([result[1],result[2],timer])

        print("★",'{:.2f}'.format(result[3]),result[1],result[8])
        if(result[3] > 0.13 and result[1] < 150 and IsMovingNow == False):
            print("■■■■■分岐",result[4],result[5],result[6],result[7],result[8])
            if(v_auto.get()):
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
                self.br.branchAngle(result[4],result[5],result[6],result[7],self.root.v1.get(),self.root.v3.get(),self.root.v7.get(),self.root.v8.get())
            



        self.root.after(10,self.getRealsense)