
import cv2 #pip install opencv-python
import numpy as np #pip install numpy
from matplotlib import pyplot as plt
import os
import math
import glob
import time
from PIL import Image,ImageTk
import numba
import csv

#4164
#3906(nopython)
#4168(ループ順の入れ替え)
#4110(List→numpy)
#4186(条件分岐簡単化)
#3968(ijkループを高速化（解を見つけた時点でbreak）)
#3759（continue条件を早めに持ってくることでループを短縮）
#2450(短形探索をする前に中心線を考えて範囲外なら即breakするように変更)
#2446(不要なキャストを削除)
#2490(初期値を設定するように修正し可読性を向上)
#1224(角度方向は条件を満たさなくなった時点でそれ以降計算しないように変更)
#1304(cosとsinの変換公式)
#1133(角度の満たすべき条件を追加)
#1150(angle1の満たすべき条件を追加)
#935(angle1,2の段階でどんなangle3を取っても最大にならない場合はbreakするように変更)
#915(sinやcosの計算を1箇所に集約)
#862(分岐点の初期位置を限定)
def reg1dim(x, y):
    n = len(x)
    a = ((np.dot(x, y) - y.sum() * x.sum() / n) / ((x ** 2).sum() - x.sum() ** 2 / n))
    b = (y.sum() - a * x.sum()) / n
    return a, b

@numba.jit(nopython=True)
def CalcDiffAngle(angle1,angle2):
    ret = 1000
    ret = min(ret,abs(angle1 - angle2))
    ret = min(ret,abs(angle1 - angle2 - 360))
    ret = min(ret,abs(angle1 - angle2 + 360))
    return ret

@numba.jit(nopython=True)
def CalcScore(field,x,y):
    circle_diameter = 82
    height, width = field.shape
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
    cable_size = 12
    checkSize = 100
    W = math.floor(checkSize / 2)
    anglestep = 5
    doubel_max = 0
    topAngle1 = 0
    topAngle2 = 0
    topAngle3 = 0

    num = int(360 / anglestep)

    angleData = np.zeros((num,2))
    for i in range(num):
        angleData[i,0] = anglestep * i
                
        cos = math.cos(math.radians(anglestep * i))
        sin = math.sin(math.radians(anglestep * i))
        for r in range(0,length):
            originalX = x + r * cos
            originalY = y + r * sin
            if(originalX >= width or originalX < 0 or originalY >= height or originalY < 0):
                break

            for thick in range(0,-cable_size,-1):
                X = int(originalX - thick * sin)#cos(theta+90)
                if(X >= width or X < 0):break
                Y = int(originalY + thick * cos)#sin(theta+90)
                if(Y >= height or Y < 0):break
                if(field[X,Y] == 0):continue
                angleData[i,1]+=1 - abs(float(thick) / cable_size) ** 2

            for thick in range(0,cable_size):
                X = int(originalX - thick * sin)#cos(theta+90)
                if(X >= width or X < 0):break
                Y = int(originalY + thick * cos)#sin(theta+90)
                if(Y >= height or Y < 0):break
                if(field[X,Y] == 0):continue
                angleData[i,1]+=1 - abs(float(thick) / cable_size) ** 2

    count = np.count_nonzero(angleData > 0,0)[1]
    angleData = angleData[np.argsort(angleData[:,1])][::-1]

    for i in range(count):
        angle1 = int(angleData[i,0])
                
        for j in range(i,count):
            angle2 = int(angleData[j,0])
            if(CalcDiffAngle(angle1,angle2) < 60):continue
            if(angleData[i,1] + angleData[j,1] > doubel_max):
                doubel_max = angleData[i,1] + angleData[j,1]
            if(angleData[i,1] + angleData[j,1] + angleData[0,1] < maxValue):break
                    
            for k in range(j,count):
                angle3 = int(angleData[k,0])
                if(CalcDiffAngle(angle2,angle3) < 60 or CalcDiffAngle(angle1,angle3) < 60):continue

                value = angleData[i,1] + angleData[j,1] + angleData[k,1]

                if(value > maxValue):
                    maxValue = value
                    maxAngle1 = angle1
                    maxAngle2 = angle2
                    maxAngle3 = angle3
                    maxValue1 = angleData[i,1]
                    maxValue2 = angleData[j,1]
                    maxValue3 = angleData[k,1]
                            
                break#ソートされているのでbreakしてOK
    return [maxAngle1,maxAngle2,maxAngle3,maxValue,maxValue1,maxValue2,maxValue3,doubel_max]


def Calc(img):
    height, width, channels = img.shape[:3]
    maxValue = 0
    step = 12
    field=np.where(np.sum(img,2)==0,0,1)

    #ざっくり解を調べる
    for x in range(0,width,step):
        for y in range(0,height,step):
            (angle1,angle2,angle3,value,value1,value2,value3,doubel) = CalcScore(field,x,y)
            if(maxValue < value):
                maxValue = value
                maxAngle1 = angle1
                maxAngle2 = angle2
                maxAngle3 = angle3
                maxValue1 = value1
                maxValue2 = value2
                maxValue3 = value3
                maxX = x
                maxY = y
                maxDoubel = doubel

    #解の周辺をさらに調べる
    detailStep = 2
    detailanglestep = 1
    for x in range(maxX - step,maxX + step,detailStep):
        for y in range(maxY - step,maxY + step,detailStep):
            (angle1,angle2,angle3,value,value1,value2,value3,doubel) = CalcScore(field,x,y)
            if(maxValue < value):
                maxValue = value
                maxAngle1 = angle1
                maxAngle2 = angle2
                maxAngle3 = angle3
                maxValue1 = value1
                maxValue2 = value2
                maxValue3 = value3
                maxX = x
                maxY = y
                maxDoubel = doubel


    return [maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,maxDoubel]

fortuneLog = [0,0,0,0,0]

def getTopAngle(img,x,y,angle1,angle2,angle3):
    
    #maxAngle1,2,3,x,yを採用した際に各方向を実現する最小二乗法近似の結果
    minDistance = 100
    maxDistance = 300
    gridDistance = 0.36#縦方向が130mmであり360ピクセルであるので1ピクセルあたり0.36mmである
    height, width, channels = img.shape[:3]
    length = int(1 + math.sqrt(height ** 2 + width ** 2))

    cable_size = 12
    checkSize = 100
    topAngle1 = 0
    topAngle2 = 0
    topAngle3 = 0
    
    
    angle1x = np.zeros(1)
    angle1y = np.zeros(1)
    angle2x = np.zeros(1)
    angle2y = np.zeros(1)
    angle3x = np.zeros(1)
    angle3y = np.zeros(1)

    
    for i in range(3):
        if(i == 0):theta = angle1
        elif(i == 1):theta = angle2
        elif(i == 2):theta = angle3
        cos = math.cos(math.radians(theta))
        sin = math.sin(math.radians(theta))

        for r in range(0,length):
            for thick in range(-cable_size,cable_size):
                X = int(x + r * cos - thick * sin)
                Y = int(y + r * sin + thick * cos)
                if(X >= width or X < 0 or Y >= height or Y < 0 or img[X,Y,0] == 0):continue
                value = ((maxDistance - minDistance) * img[X,Y,0] + minDistance) / 255
                if(i == 0):
                    angle1x = np.append(angle1x,r * gridDistance)
                    angle1y = np.append(angle1y,value)
                elif(i == 1):
                    angle2x = np.append(angle2x,r * gridDistance)
                    angle2y = np.append(angle2y,value)
                elif(i == 2):
                    angle3x = np.append(angle3x,r * gridDistance)
                    angle3y = np.append(angle3y,value)



    a1,b1 = reg1dim(angle1x,angle1y)
    a2,b2 = reg1dim(angle2x,angle2y)
    a3,b3 = reg1dim(angle3x,angle3y)

    topAngle1 = math.degrees(math.atan(a1))
    topAngle2 = math.degrees(math.atan(a2))
    topAngle3 = math.degrees(math.atan(a3))
    return [topAngle1,topAngle2,topAngle3]


def ImageReconition(original_img):
    img = original_img
    
    debugMode = False
    if debugMode:
        return [img,0,0,0]

    img = original_img[0:360, 280:640]
    original_img = original_img[0:360, (640 + 280):1280]
    
    #水平方向角度の取得
    start = time.time()
    (y,x,angle1,angle2,angle3,value1,value2,value3,doubel_max) = Calc(img)
    x=int(x)
    y=int(y)
    print("calcTime",time.time() - start)

    #仰角の取得
    start = time.time()
    (topAngle1,topAngle2,topAngle3) = getTopAngle(img,y,x,angle1,angle2,angle3)
    print("getTopAngleTime",time.time() - start)

    #信頼度の導出
    fortunity = min((value1,value2,value3)) / max(1,value1 + value2 + value3)
    fortunity = 1 - doubel_max / max(1,value1 + value2 + value3)
    fortuneLog.pop(0)
    fortuneLog.append(fortunity)
    fortunity = sum(fortuneLog) / 5
    if(fortunity > 0.1 and x < 100):
        for i in range(5):
            fortuneLog[i] = 0


    #描画など
    print(angle1,angle2,angle3,value1,value2,value3,fortunity)
    thickness = 1
    try:
        if(fortunity >= 0.1 or True):
            theta = angle1
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(255,0,0,50),thickness=thickness)
            img = cv2.putText(img,str(int(topAngle1)),(x + int(100 * math.sin(math.radians(theta))),y + int(100 * math.cos(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(255,0,0))
            theta = angle2
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(0,255,0,50),thickness=thickness)#緑
            img = cv2.putText(img,str(int(topAngle2)),(x + int(100 * math.sin(math.radians(theta))),y + int(100 * math.cos(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0))
            theta = angle3
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(0,0,255,50),thickness=thickness)
            img = cv2.putText(img,str(int(topAngle3)),(x + int(100 * math.sin(math.radians(theta))),y + int(100 * math.cos(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255))
            img = cv2.putText(img,str(int(topAngle1)) + "/" + str(int(topAngle2)) + "/" + str(int(topAngle3)),(0,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
        else:
            for theta in [angle1]:
                img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(255,0,0,50),thickness=thickness)
    except:
        print("error")


    img = np.hstack((original_img,img))
    return [img,x,y,fortunity]
