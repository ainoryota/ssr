
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

#一次式で最小事情近似する
def reg1dim(x, y):
    try:
        n = len(x)
        under = ((x ** 2).sum() - x.sum() ** 2 / n)
        if under == 0:a = 0
        else: a = (np.dot(x, y) - y.sum() * x.sum() / n) / under
        if n == 0:b = 0
        else:       b = (y.sum() - a * x.sum()) / n
    except:
        OutputController().msgPrint("reg1dim Error")
        a = 0
        b = 0

    return a, b

#unitの倍数になるようvalueを調節する
def getUnitValue(value,unit):
    try:
        if(value > 0):    return int((value + unit / 2) / unit) * unit
        else:    return int((value - unit / 2) / unit) * unit
    except:
        OutputController().msgPrint("getUnitValue Error")
        return 0


#2つの角度の差を求める
@numba.jit(nopython=True)
def CalcDiffAngle(angle1,angle2):
    ret = min(360,abs(angle1 - angle2))
    ret = min(ret,abs(angle1 - angle2 - 360))
    ret = min(ret,abs(angle1 - angle2 + 360))
    return ret

#分岐点位置を与えたときにその位置を評価する
@numba.jit(nopython=True)
def CalcScore(field,x,y,anglestep):
    height, width = field.shape
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

    doubel_max = 0
    topAngle1 = 0
    topAngle2 = 0
    topAngle3 = 0

    num = int(360 / anglestep)

    angleData = np.zeros((num,2))

    i = -1
    while i < num:
        i = i + 1
        angleData[i,0] = anglestep * i
                
        cos = math.cos(math.radians(anglestep * i))
        sin = math.sin(math.radians(anglestep * i))

        r = 0
        while True:
            r+=1
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
    step = 12 * 2
    field = np.where(np.any(img > 0,2) == 0,0,1)
    maxX = 0
    maxY = 0
    maxAngle1=-1
    maxAngle2=-1
    maxAngle3=-1
    maxValue1=-1
    maxValue2=-1
    maxValue3=-1
    maxDoubel=-1
    anglestep = 10
    #ざっくり解を調べる
    for x in range(0,width,step):
        for y in range(0,height,step):
            (angle1,angle2,angle3,value,value1,value2,value3,doubel) = CalcScore(field,x,y,anglestep)
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
    detailStep = 5
    detailanglestep = 5
    for x in range(maxX - step,maxX + step,detailStep):
        for y in range(maxY - step,maxY + step,detailStep):
            (angle1,angle2,angle3,value,value1,value2,value3,doubel) = CalcScore(field,x,y,detailanglestep)
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
GammalAngleLog = [0,0,0,0,0]
TurnAngleLog = [0,0,0,0,0]
RangleLog = [0,0,0,0,0]
LangleLog = [0,0,0,0,0]
XLog=[0,0,0,0,0]

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


def getRobotAngle(img,x,y,angle1,angle2,angle3,rotation):
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0
    

    height, width, channels = img.shape[:3]
    field = img.copy()

    newAngles = np.sort(np.abs(np.array([angle1,angle2,angle3])))

    
    
    
    w = width
    h = height


    mat = cv2.getRotationMatrix2D((w / 2, h / 2), newAngles[2] - 270,1)
    spinField = cv2.warpAffine(field, mat, (w, h))



    #角度計算
   #maxAngle1,2,3,x,yを採用した際に各方向を実現する最小二乗法近似の結果
    minDistance = 100
    maxDistance = 300
    gridDistance = 0.36#縦方向が130mmであり360ピクセルであるので1ピクセルあたり0.36mmである
    height, width, channels = img.shape[:3]
    length = int(1 + math.sqrt(height ** 2 + width ** 2))

    cable_size = 12
    checkSize = 100
    
    
    Data1x = np.zeros(1)
    Data1y = np.zeros(1)
    Data2x = np.zeros(1)
    Data2y = np.zeros(1)

    
    for theta in [newAngles[0],90 - newAngles[0] + (newAngles[2] - 270),newAngles[1] - 90 - (newAngles[2] - 270)]:
        cos = math.cos(math.radians(theta))
        sin = math.sin(math.radians(theta))

        for r in range(0,length):
            for thick in range(-cable_size,cable_size):
                X = int(x + r * cos - thick * sin)
                Y = int(y + r * sin + thick * cos)
                if(X >= width or X < 0 or Y >= height or Y < 0 or spinField[X,Y,0] == 0):continue
                value = ((maxDistance - minDistance) * spinField[X,Y,0] + minDistance) / 255
                Data1x = np.append(Data1x,X)
                Data1y = np.append(Data1y,value)
                Data2x = np.append(Data2x,Y)
                Data2y = np.append(Data2y,value)



    a1,b1 = reg1dim(Data1x,Data1y)
    a2,b2 = reg1dim(Data2x,Data2y)

    g = 9.8
    div = min(g,rotation.z) / g
    tieAngle = math.degrees(math.acos(div))
    #OutputController().msgPrint("仰角:",math.degrees(math.atan(a1)) , tieAngle,"/旋回角:",-math.degrees(math.atan(a2)),"/R角:",newAngles[1] - 90 - (newAngles[2] - 270),"/L角:", 90 - newAngles[0] + (newAngles[2] - 270))   
    #OutputController().msgPrint("new角",newAngles,"/a1,a2:",a1,a2)

    GammalAngle = math.degrees(math.atan(a1)) + tieAngle
    TurnAngle = -math.degrees(math.atan(a2))
    Rangle = newAngles[1] - 90 - (newAngles[2] - 270)
    Langle = 90 - newAngles[0] + (newAngles[2] - 270)
    


    try:
        GammalAngle = getUnitValue(GammalAngle,5)
        TurnAngle = getUnitValue(TurnAngle,5)
        Rangle = getUnitValue(Rangle,5)
        Langle = getUnitValue(Langle,5)
        
    except:
        OutputController().msgPrint("errorAngle")
        GammalAngle = 0
        TurnAngle = 0
        Rangle = 0
        Langle = 0
        

    GammalAngle = min(20,GammalAngle)
    GammalAngle = max(-20,GammalAngle)
    TurnAngle = min(20,TurnAngle)
    TurnAngle = max(-20,TurnAngle)
    Rangle = max(30,Rangle)
    Langle = max(30,Langle)

    while abs(Langle + Rangle) < 100:
        Rangle+=5
        Langle+=5


    
    #OutputController().msgPrint("output",GammalAngle,TurnAngle,Rangle,Langle)

    return (GammalAngle,TurnAngle,Rangle,Langle)

def ImageReconition(original_img,rotation):
    img = original_img
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0

    debugMode = False
    if debugMode:
        return [img,0,0,0,GammalAngle,TurnAngle,Rangle,Langle]

    img = original_img[0:360, 280:640]
    original_img = original_img[0:360, (640 + 280):1280]
    
    #水平方向角度の取得
    (y,x,angle1,angle2,angle3,value1,value2,value3,doubel_max) = Calc(img)
    x = int(x)
    y = int(y)

    #仰角の取得
    (topAngle1,topAngle2,topAngle3) = getTopAngle(img,y,x,angle1,angle2,angle3)

    #信頼度の導出
    fortunity = min((value1,value2,value3)) / max(1,value1 + value2 + value3)
    fortunity = 1 - doubel_max / max(1,value1 + value2 + value3)
    fortuneLog.pop(0)
    fortuneLog.append(fortunity)
    fortunity = sum(fortuneLog) / 5
    (GammalAngle,TurnAngle,Rangle,Langle) = getRobotAngle(img,y,x,angle1,angle2,angle3,rotation)


    GammalAngleLog.pop(0)
    GammalAngleLog.append(GammalAngle)
    GammalAngle = sum(GammalAngleLog) / 5

    TurnAngleLog.pop(0)
    TurnAngleLog.append(TurnAngle)
    TurnAngle = sum(TurnAngleLog) / 5

    RangleLog.pop(0)
    RangleLog.append(Rangle)
    Rangle = sum(RangleLog) / 5

    LangleLog.pop(0)
    LangleLog.append(Langle)
    Langle = sum(LangleLog) / 5

    GammalAngle = getUnitValue(GammalAngle,5)
    TurnAngle = getUnitValue(TurnAngle,5)
    Rangle = getUnitValue(Rangle,5)
    Langle = getUnitValue(Langle,5)

    #描画など
    OutputController().msgPrint(x,y,GammalAngle,TurnAngle,Rangle,Langle,'{:.2f}'.format(fortunity))

    thickness = 1
    try:
        if(fortunity >= 0.1 or True):
            theta = angle1
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(255,100,100,50),thickness=thickness)
            img = cv2.putText(img,str(int(topAngle1)),(x + int(100 * math.sin(math.radians(theta))),y + int(100 * math.cos(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(255,100,100))
            theta = angle2
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(100,255,100,50),thickness=thickness)#緑
            img = cv2.putText(img,str(int(topAngle2)),(x + int(100 * math.sin(math.radians(theta))),y + int(100 * math.cos(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(100,255,100))
            theta = angle3
            img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(100,100,255,50),thickness=thickness)
            img = cv2.putText(img,str(int(topAngle3)),(x + int(100 * math.sin(math.radians(theta))),y + int(100 * math.cos(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(100,100,255))
            img = cv2.putText(img,str(int(topAngle1)) + "/" + str(int(topAngle2)) + "/" + str(int(topAngle3)),(0,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
        else:
            for theta in [angle1]:
                img = cv2.line(img,(x,y),(x + int(360 * math.sin(math.radians(theta))),y + int(360 * math.cos(math.radians(theta)))),color=(255,0,0,50),thickness=thickness)
    except:
        OutputController().msgPrint("error")


    img = np.hstack((original_img,img))

    XLog.pop(0)
    XLog.append(x)
    return [img,x,y,fortunity,GammalAngle,TurnAngle,Rangle,Langle,XLog]

def ResetLog():
    for i in range(5):
        fortuneLog[i]=0
        GammalAngleLog[i]=0
        TurnAngleLog[i]=0
        RangleLog[i]=0
        LangleLog[i]=0
        XLog[i]=0
