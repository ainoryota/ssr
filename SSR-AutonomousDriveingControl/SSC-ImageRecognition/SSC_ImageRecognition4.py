
import cv2 #pip install opencv-python
import numpy as np #pip install numpy
from matplotlib import pyplot as plt
import os
import math
import glob
import time
from PIL import Image,ImageTk
import numba

def reg1dim(x, y):
    n = len(x)
    a = ((np.dot(x, y) - y.sum() * x.sum() / n) / ((x ** 2).sum() - x.sum() ** 2 / n))
    b = (y.sum() - a * x.sum()) / n
    return a, b

@numba.jit()
def Calc(original_img):
    
    circle_diameter = 82

   
    height, width, channels = original_img.shape[:3]
    length = int(1 + math.sqrt(height ** 2 + width ** 2))

    maxValue = 0
    maxValue1 = 0
    maxValue2 = 0
    maxValue3 = 0
    maxX = -1
    maxY = -1
    maxAngle1 = -1
    maxAngle2 = -1
    maxAngle3 = -1
    step = 12
    cable_size = 6
    checkSize = 100
    W = math.floor(checkSize / 2)
    img = original_img
    anglestep = 1
    doubel_max = 0
    topAngle1 = 0
    topAngle2 = 0
    topAngle3 = 0

    for x in range(0,width,step):
        for y in range(0,height,step):
            num = int(360 / anglestep)
            angleData = np.zeros((num,2))
            count = 0
            for i in range(num):
                theta = anglestep * i
                angleData[i,0] = theta
                for r in range(0,length):
                    for thick in range(-cable_size,cable_size):
                        X = int(x + r * math.cos(math.radians(theta)) + thick * math.cos(math.radians(theta + 90)))
                        Y = int(y + r * math.sin(math.radians(theta)) + thick * math.sin(math.radians(theta + 90)))
                        if(X<=60):continue
                        if(X >= width or X < 0 or Y >= height or Y < 0 or (img[X,Y,0] + img[X,Y,1] + img[X,Y,2] == 0)):continue
                        angleData[i,1]+=1
                if(angleData[i,1] > 0):
                    count+=1

            angleData = angleData[np.argsort(angleData[:,1])][::-1]
            


            for i in range(count):
                angle1 = int(angleData[i,0])
                for j in range(i,count):
                    angle2 = int(angleData[j,0])
                    if(angle1+60>angle2):continue
                    if(angleData[i,1] + angleData[j,1] > doubel_max):
                        doubel_max = angleData[i,1] + angleData[j,1]
                    for k in range(j,count):
                        angle3 = int(angleData[k,0])
                        if( angle2+60>angle3 or angle3-angle1>300):continue

                        value = angleData[i,1] + angleData[j,1] + angleData[k,1]

                        if(value > maxValue):
                            maxValue = value
                            maxX = x
                            maxY = y
                            maxAngle1 = angle1
                            maxAngle2 = angle2
                            maxAngle3 = angle3
                            maxValue1 = angleData[i,1]
                            maxValue2 = angleData[j,1]
                            maxValue3 = angleData[k,1]
                            #OutputController().msgPrint(maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue)



    num = int(360 / anglestep)

    
    

    #OutputController().msgPrint(maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue)
    #OutputController().msgPrint(maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,doubel_max)
    return [maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,doubel_max]

fortuneLog = [0,0,0,0,0]

def getTopAngle(original_img,x,y,angle1,angle2,angle3):
    
    #maxAngle1,2,3,x,yを採用した際に各方向を実現する最小二乗法近似の結果
    minDistance = 100
    maxDistance = 300
    gridDistance = 0.36#縦方向が130mmであり360ピクセルであるので1ピクセルあたり0.36mmである

    circle_diameter = 82

   
    height, width, channels = original_img.shape[:3]

    length = int(1 + math.sqrt(height ** 2 + width ** 2))

    maxValue = 0
    maxValue1 = 0
    maxValue2 = 0
    maxValue3 = 0
    maxX = x
    maxY = y

    cable_size = 12
    checkSize = 100
    W = math.floor(checkSize / 2)
    img = original_img
    doubel_max = 0
    topAngle1 = 0
    topAngle2 = 0
    topAngle3 = 0
    
    
    angle1x = []
    angle1y = []
    angle2x = []
    angle2y = []
    angle3x = []
    angle3y = []
    



    for i in range(3):
        if(i == 0):theta = angle1
        elif(i == 1):theta = angle2
        elif(i == 2):theta = angle3

        for r in range(0,length):
            count = 0
            for thick in range(-cable_size,cable_size):
                X = int(maxX + r * math.cos(math.radians(theta)) + thick * math.cos(math.radians(theta + 90)))
                Y = int(maxY + r * math.sin(math.radians(theta)) + thick * math.sin(math.radians(theta + 90)))
                if(X >= width or X < 0 or Y >= height or Y < 0 or img[X,Y,0]==0):continue
                value = ((maxDistance-minDistance)*img[X,Y,0]+minDistance)/255
                
                if(i == 0):
                    angle1x = np.append(angle1x,r*gridDistance)
                    angle1y = np.append(angle1y,value)
                elif(i == 1):
                    angle2x = np.append(angle2x,r*gridDistance)
                    angle2y = np.append(angle2y,value)
                elif(i == 2):
                    angle3x = np.append(angle3x,r*gridDistance)
                    angle3y = np.append(angle3y,value)



    a1,b1 = reg1dim(angle1x,angle1y)
    a2,b2 = reg1dim(angle2x,angle2y)
    a3,b3 = reg1dim(angle3x,angle3y)

    topAngle1 = math.atan(a1  ) * 180 / 3.14
    topAngle2 = math.atan(a2  ) * 180 / 3.14
    topAngle3 = math.atan(a3  ) * 180 / 3.14
    return [topAngle1,topAngle2,topAngle3]


def ImageReconition(original_img):
    img = original_img
    img = original_img[0:360, 280:640]
    original_img = original_img[0:360, (640 + 280):1280]
    
    #img=cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    data = Calc(img)

    y = int(data[0])
    x = int(data[1])
    angle1 = data[2]
    angle2 = data[3]
    angle3 = data[4]
    value1 = data[5]
    value2 = data[6]
    value3 = data[7]
    doubel_max = data[8]

    data2=getTopAngle(original_img,y,x,angle1,angle2,angle3)

    topAngle1 = data2[0]
    topAngle2 = data2[1]
    topAngle3 = data2[2]
    OutputController().msgPrint(topAngle1,topAngle2,topAngle3)

    fortunity = min((value1,value2,value3)) / max(1,value1 + value2 + value3)
    fortunity = 1 - doubel_max / max(1,value1 + value2 + value3)
    fortuneLog.pop(0)
    fortuneLog.append(fortunity)
    fortunity = sum(fortuneLog) / 5
    if(fortunity > 0.1 and x < 100):
        for i in range(5):
            fortuneLog[i] = 0

    #OutputController().msgPrint(data,fortunity)
    thickness=25
    thickness=1
    if(fortunity > 0.1*0):
        theta=angle1
        img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(255,0,0,50),thickness=thickness)
        theta=angle2
        img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(0,255,0,50),thickness=thickness)#緑
        theta=angle3
        img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(0,0,255,50),thickness=thickness)
    else:
        for theta in [angle1]:
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(255,0,0,50),thickness=thickness)

    img = np.hstack((original_img,img))
    return [img,x,y,fortunity]
