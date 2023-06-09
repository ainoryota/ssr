import time
import cv2

import platform
from functools import partial

import sys

import math
import pyrealsense2 as rs
import numpy as np
import tkinter as tk
from PIL import Image,ImageTk #udo pip install pillow
from ctypes import alignment, windll
from OutputController import OutputController

def cvpaste(img, imgback, x, y, angle, scale):  
    # x and y are the distance from the center of the background image

    r = img.shape[0]
    c = img.shape[1]
    rb = imgback.shape[0]
    cb = imgback.shape[1]
    hrb = round(rb / 2)
    hcb = round(cb / 2)
    hr = round(r / 2)
    hc = round(c / 2)

    # Copy the forward image and move to the center of the background image
    imgrot = np.zeros((rb,cb,3),np.uint8)
    imgrot[hrb - hr:hrb + hr,hcb - hc:hcb + hc,:] = img[:hr * 2,:hc * 2,:]

    # Rotation and scaling
    M = cv2.getRotationMatrix2D((hcb,hrb),angle,scale)
    imgrot = cv2.warpAffine(imgrot,M,(cb,rb))
    # Translation
    M = np.float32([[1,0,x],[0,1,y]])
    imgrot = cv2.warpAffine(imgrot,M,(cb,rb))

    # Makeing mask
    imggray = cv2.cvtColor(imgrot,cv2.COLOR_BGR2GRAY)
    ret, mask = cv2.threshold(imggray, 10, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)

    # Now black-out the area of the forward image in the background image
    img1_bg = cv2.bitwise_and(imgback,imgback,mask = mask_inv)

    # Take only region of the forward image.
    img2_fg = cv2.bitwise_and(imgrot,imgrot,mask = mask)

    # Paste the forward image on the background image
    #OutputController().msgPrint(img1_bg.shape)
    #OutputController().msgPrint(img2_fg.shape)
    imgpaste = cv2.add(img1_bg,img2_fg, dtype = cv2.CV_8U)

    return imgpaste


class WebCameraMgr(object):
    def __init__(self,imgArea,infArea,branch,data,il):
        time.sleep(3)
        self.imgArea = imgArea
        self.infArea = infArea
        self.br = branch
        self.data = data
        self.w = 320
        self.h = 180
        self.windowsize = (self.w,self.h)#最大1920*1080
        self.il = il
        self.StopFlag = True
        okcamera=[];

        OutputController().msgPrint("Camera List")
        for i in range(10):
            try:
                self.capture = cv2.VideoCapture(i,cv2.CAP_DSHOW)
                ret, frame = self.capture.read()
                self.capture.release()
                OutputController().msgPrint(i,ret,frame.shape)
                okcamera.append(i);
            except:
                pass

        OutputController.msgPrint(okcamera)
        self.captureF = cv2.VideoCapture(okcamera[0],cv2.CAP_DSHOW)
        OutputController().msgPrint("Open camera front")

        #self.captureC = cv2.VideoCapture(2,cv2.CAP_DSHOW)
        #OutputController().msgPrint("Open camera center")
        
        #self.captureB = cv2.VideoCapture(3,cv2.CAP_DSHOW)
        #OutputController().msgPrint("Open camera back")

        #0 True (480, 640, 3)*front realsense
        #2 webカメラ
        #3 True (480, 640, 3)
        #4 True (480, 640, 3)*back realsense

        #7 True (480, 640, 3)*bug


       
    def __del__(self):
        self.captureF.release()
        #self.captureC.release()
        #self.captureB.release()

    def start(self):
        self.StopFlag = False
        self.getWebCamera()

    def stop(self):
        self.StopFlag = True

    #@jit("f8[:,:]()")
    def getWebCamera(self):
        if(self.StopFlag == True):return
        imgF = np.zeros((self.h,self.w,3),dtype=np.uint8)
        try:
            imgF = self.getImage(self.captureF,imgF)
        except Exception as e:
            OutputController().msgPrint("error CameraF",e)

        img = imgF
        try:
            output = ImageTk.PhotoImage(Image.fromarray(img))
            self.il.configure(image=output)
            self.il.image = output
        except Exception as e:
            OutputController().msgPrint("error Camera output",e)

        self.imgArea.after(100,self.getWebCamera)

    def getImage(self,cap,img):
        ret, frame = cap.read()
        if(ret): img = cv2.cvtColor(cv2.resize(frame,(self.w,self.h)),cv2.COLOR_BGR2RGB)
        else:OutputController().msgPrint('cant read camera')
        return img