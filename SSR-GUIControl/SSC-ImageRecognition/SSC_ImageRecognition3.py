
import cv2 #pip install opencv-python
import numpy as np #pip install numpy
from matplotlib import pyplot as plt
import os
import math
import glob
import time
from PIL import Image,ImageTk
#import numba

#@numba.jit()
def Calc(original_img):
    cable_size=22
    circle_diameter=82

   
    height, width, channels = original_img.shape[:3]

    maxValue=0;
    maxX=-1;
    maxY=-1;
    step=50;
    checkSize=100;
    W=math.floor(checkSize/2)
    img=original_img

    aveB=np.sum(np.sum(img,axis=0),axis=0)[2]/(width*height)


    for x in range(W,width-W,step):
        for y in range(W,height-W,step):
            a=np.sum(np.sum(img[y-W:y+W,x-W:x+W,:],axis=0),axis=0)
            R=a[0]
            G=a[1]
            B=a[2]
            value=B
            if(value>maxValue):
                #if(B<aveB*(checkSize*checkSize)*0.5):continue;#平均値の半分は青要素を含んでいるものに限定する
                maxValue=value;
                maxX=x;
                maxY=y
    print([maxX,maxY,W])
    return [maxX,maxY,W]



def ImageReconition(original_img):
    img=original_img
    img=original_img[0:360, 280:640]
    original_img=original_img[0:360, (640+280):1280]
    
    #img=cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    data=Calc(img);
    x=data[0]
    y=data[1]
    W=data[2]
    height, width, channels = img.shape[:3]
    if(0<x and x<width):
        #print(x,y);
        img = cv2.rectangle(img,( x-W,y-W),( x+W,y+W),(255,0,0), 3)
    else:
        #print("none")
        x=-1
        y=-1

    img = np.hstack((original_img,img))
    return [img,x,y]
