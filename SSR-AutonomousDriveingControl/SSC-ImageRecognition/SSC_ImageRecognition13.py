

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
from KMeans import KMeans
import matplotlib.pyplot as plt

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
        print("reg1dim Error")
        a = 0
        b = 0

    return a, b

#unitの倍数になるようvalueを調節する
def getUnitValue(value,unit):
    try:
        if(value > 0):    return int((value + unit / 2) / unit) * unit
        else:    return int((value - unit / 2) / unit) * unit
    except:
        print("getUnitValue Error")
        return 0


#2つの角度の差を求める
@numba.jit(nopython=True)
def CalcDiffAngle(angle1,angle2):
    ret = min(360,abs(angle1 - angle2))
    ret = min(ret,abs(angle1 - angle2 - 360))
    ret = min(ret,abs(angle1 - angle2 + 360))
    return ret

#分岐点位置を与えたときにその位置を評価する
def CalcScore(field,x,y,anglestep):
    h,w=field.shape
    p=np.sum(field[max(y-12,0):min(h-1,y+12),max(x-12,0):min(w-1,x+12)])
    return [0,30,60,p,0,0,0,0]


    h, w = field.shape
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
    while i < num-1:
        i = i + 1
        angleData[i][0] = anglestep * i
                
        cos = math.cos(math.radians(anglestep * i))
        sin = math.sin(math.radians(anglestep * i))

        r = 0
        while True:
            r+=1
            originalX = x + r * cos
            originalY = y + r * sin
            if(originalX >= w or originalX < 0 or originalY >= h or originalY < 0):
                break


            for thick in range(0,-cable_size,-1):
                X = int(originalX - thick * sin)#cos(theta+90)
                if(X >= w or X < 0):break
                Y = int(originalY + thick * cos)#sin(theta+90)
                if(Y >= h or Y < 0):break
                if(field[Y][X] == 0):continue
                angleData[i][1]+=1 - abs(float(thick) / cable_size) ** 2

            for thick in range(0,cable_size):
                X = int(originalX - thick * sin)#cos(theta+90)
                if(X >= w or X < 0):break
                Y = int(originalY + thick * cos)#sin(theta+90)
                if(Y >= h or Y < 0):break
                if(field[Y][X] == 0):continue
                angleData[i][1]+=1 - abs(float(thick) / cable_size) ** 2

    count = np.count_nonzero(angleData > 0,0)[1]
    angleData = angleData[np.argsort(angleData[:,1])][::-1]


    angle270Score = 0
    for i in range(count):
        if(angleData[i][0]== 270):
            angle270Score = angleData[i][1]
            break

    for i in range(count):
        angle1 = int(angleData[i][0])            
        for j in range(i,count):
            angle2 = int(angleData[j][1])
            if(CalcDiffAngle(angle1,angle2) < 60):continue
            if(angleData[i][1] + angleData[j][1] > doubel_max):
                doubel_max = angleData[i][1] + angleData[j][1]
            if(angleData[i][1] + angleData[j][1] + angleData[0][1] < maxValue):break
                    
            angle3 = 270
            if(CalcDiffAngle(angle2,angle3) < 60 or CalcDiffAngle(angle1,angle3) < 60):continue

            value = angleData[i][1] + angleData[j][1] + angle270Score

            if(value > maxValue):
                maxValue = value
                maxAngle1 = angle1
                maxAngle2 = angle2
                maxAngle3 = angle3
                maxValue1 = angleData[i][1]
                maxValue2 = angleData[j][1]
                maxValue3 = angle270Score
                            
                break#ソートされているのでbreakしてOK
    return [maxAngle1,maxAngle2,maxAngle3,maxValue,maxValue1,maxValue2,maxValue3,doubel_max]

def calcTurningAngle(binryScale):
    h, w = binryScale.shape

    maxValue = 0
    step = 12 * 2
    field = np.where(binryScale > 0,1,0)
    maxX = 0
    maxY = 0
    maxAngle1 = -1
    maxAngle2 = -1
    maxAngle3 = -1
    maxValue1 = -1
    maxValue2 = -1
    maxValue3 = -1
    maxDoubel = -1
    anglestep = 10

    PointList=np.zeros((int(h/step+1),int(w/step+1)))
    #ざっくり解を調べる
    for x in range(0,w,step):
        for y in range(0,h,step):
            (angle1,angle2,angle3,value,value1,value2,value3,doubel) = CalcScore(field,x,y,anglestep)
            if(PointList[int(y/step)][int(x/step)]==0):PointList[int(y/step)][int(x/step)]=value
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

    ##解の周辺をさらに調べる
    #detailStep = 5
    #detailanglestep = 5
    #for x in range(maxX - step,maxX + step,detailStep):
    #    for y in range(maxY - step,maxY + step,detailStep):
    #        (angle1,angle2,angle3,value,value1,value2,value3,doubel) = CalcScore(field,x,y,detailanglestep)
    #        if(maxValue < value):
    #            maxValue = value
    #            maxAngle1 = angle1
    #            maxAngle2 = angle2
    #            maxAngle3 = angle3
    #            maxValue1 = value1
    #            maxValue2 = value2
    #            maxValue3 = value3
    #            maxX = x
    #            maxY = y
    #            maxDoubel = doubel

    PointList=(PointList*255/maxValue).astype(np.uint8)
    return [maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,maxDoubel,PointList]


def Calc(img):
    height, width, channels = img.shape[:3]
    maxValue = 0
    step = 12 * 2
    field = np.where(np.any(img > 0,2) == 0,0,1)
    maxX = 0
    maxY = 0
    maxAngle1 = -1
    maxAngle2 = -1
    maxAngle3 = -1
    maxValue1 = -1
    maxValue2 = -1
    maxValue3 = -1
    maxDoubel = -1
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
XLog = [0,0,0,0,0]

def getTopAngle(img,x,y,angle1,angle2,angle3):
    #maxAngle1,2,3,x,yを採用した際に各方向を実現する最小二乗法近似の結果
    minDistance = 100
    maxDistance = 1500
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
    maxDistance = 500
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
    #print("仰角:",math.degrees(math.atan(a1)) ,
    #tieAngle,"/旋回角:",-math.degrees(math.atan(a2)),"/R角:",newAngles[1] - 90 -
    #(newAngles[2] - 270),"/L角:", 90 - newAngles[0] + (newAngles[2] - 270))
    #print("new角",newAngles,"/a1,a2:",a1,a2)

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
        print("errorAngle")
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


    
    #print("output",GammalAngle,TurnAngle,Rangle,Langle)

    return (GammalAngle,TurnAngle,Rangle,Langle)

def ImageReconition(binaryScale):
    img = original_img
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0
    
    img = original_img
    #original_img = original_img[0:360, (640 + 280):1280]
    


    debugMode = False
    if debugMode:
        return [img,0,0,0,GammalAngle,TurnAngle,Rangle,Langle,XLog]

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
    print("angle1",angle1,angle2,angle3)
    (GammalAngle,TurnAngle,Rangle,Langle) = getRobotAngle(img,y,x,angle1,angle2,angle3,rotation)
    print("Newangle1",GammalAngle,TurnAngle,Rangle,Langle)

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
    print(x,y,GammalAngle,TurnAngle,Rangle,Langle,'{:.2f}'.format(fortunity))

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
        print("error")


    XLog.pop(0)
    XLog.append(x)
    return [img,x,y,fortunity,GammalAngle,TurnAngle,Rangle,Langle,XLog]

def ShiftTrim(image,Yshift,Xshift):
    image = np.roll(image, (Yshift, Xshift), axis=(0, 1))
    w = image.shape[1]
    h = image.shape[0]
    if Xshift >= 0:
        image[:, :Xshift] = 0
    else:
        image[:, w + Xshift:] = 0

    if Yshift >= 0:
        image[:Yshift] = 0
    else:
        image[h + Yshift:] = 0


    #print(w,h,Xshift,Yshift)
    return image

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
    #print(img1_bg.shape)
    #print(img2_fg.shape)
    imgpaste = cv2.add(img1_bg,img2_fg, dtype = cv2.CV_8U)

    return imgpaste

#2ms
def ScalarImage2RGB(img,ClipMinDistance,ClipMaxDistance):
    rate = 255.0 / (ClipMaxDistance - ClipMinDistance)
    img = np.where((img <= ClipMinDistance) | (img >= ClipMaxDistance),255,rate * (img - ClipMinDistance))
    return np.dstack([img,img,img]).astype(np.uint8)

def IsArea(value,min,max):
    if(min <= value and value <= max):
        return True
    else:
        return False

def RegionGrowing(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale):
    #depth要素のある部分にフラグを付ける
    flag = np.where((minDistance <= depth_scale) & (depth_scale < maxDistance),3,0)
    start_position = [int(h / 2),int(w / 2)]
    threshold = 127#分割領域の閾値
    pix = ir_scale[start_position[1]][start_position[0]]#中心の画素値

    #領域拡張法のための定義
    directs = [(-1,-1), (0,-1), (1,-1), (1,0), (1,1), (0,1),(-1,1),(-1,0)]
    Xdirects=[[-1,0,1],[-1,0,1],[-1,0,1]]
    Ydirects=[[-1,-1,-1],[0,0,0],[1,1,1]]
    #ir_image = cv2.medianBlur(ir_image,9)

    #モルフォロジー変換
    kernel = np.ones((3,3),np.uint8)
    depth_scale = cv2.erode(depth_scale,kernel,iterations = 1)
    #ret,ir_image = cv2.threshold(cv2.cvtColor(ir_image,
    #cv2.COLOR_BGR2GRAY),0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ir_image = cv2.adaptiveThreshold(cv2.cvtColor(ir_image,
    #cv2.COLOR_BGR2GRAY),255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    #cv2.THRESH_BINARY,11,2)
    #ir_image=cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)

    idx = np.where((minDistance <= depth_scale) & (depth_scale <= maxDistance))
    seeds = list(zip(idx[0],idx[1]))
    colorList=ir_scale[idx]
    colorList.sort()
    minColor = colorList[int(len(colorList) * 0.1)] * 0.9
    maxColor = colorList[int(len(colorList) * 0.9)] * 1.1

    counter=0
    while len(seeds)>0:
        (y,x)= seeds.pop(0)
        if x == 0 or y == 0 or x == w-1 or y == h-1: continue

        #depthが指定の範囲内にある隣接箇所で行ったことないところを拡張していく
        flagData=np.where(flag[y-1:y+2,x-1:x+2]==0,1,0)#まだ通ってない場所だけ1
        if(np.sum(flagData)==0):continue
        ir_scale_data=ir_scale[y-1:y+2,x-1:x+2]
        if(ir_scale[y][x]+5<minColor or maxColor<ir_scale[y][x]-5):continue
        growArea=np.where((minColor < ir_scale_data)&(ir_scale_data< maxColor)&(abs(ir_scale[y][x]-ir_scale_data)<=5),1,0)*flagData
        if(np.sum(growArea)==0):continue
        xAll=(Xdirects+x)*growArea
        yAll=(Ydirects+y)*growArea

        #通ったflagを立て隣接箇所をseedsに追加   
        seedPlus=[a for a in list(zip(yAll.ravel(),xAll.ravel())) if a != (0,0)]
        if len(seedPlus)>0:
            a,b=zip(*seedPlus)
            flag[a,b] = 1
            seeds.extend(seedPlus)
    return flag

#赤外線画像とdepth画像を合成した二値画像を生成する
def WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale):
    resultScale=np.zeros((h,w),dtype=np.uint8)
        
    #depth要素のある部分にフラグ3を付ける
    flag = np.where((minDistance <= depth_scale) & (depth_scale < maxDistance),3,0)
        
    #遠くのdepth要素がある部分にフラグ4を付ける
    flag = np.where((depth_scale > overDistance),4,flag)

    #データが不正の位地にフラグ1をつける
    flag = np.where((depth_scale==0) ,1,flag)
            
    #IR要素が小さい部分にフラグ2をつける
    flag = np.where((ir_scale <= 100) ,2,flag)

    binaryScale=np.where(flag==3,255,0).astype(np.uint8)

    return (flag,binaryScale)

def DrawAngleLine(img,x,y,theta,color,thickness):
    return cv2.line(img,(x,y),(x + int(360 * math.cos(math.radians(theta))),y + int(360 * math.sin(math.radians(theta)))),color=color,thickness=thickness)


def IR(color_image,depth_scale,ir_image,robot_rotation,extMode=True,viewScale=50):

    #索道などのパラメータ
    minDistance = 50
    maxDistance = 600
    overDistance = 3000

    #戻り値
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0

    #画像の標準サイズ
    w = 320
    h = 180

    #赤外線カメラとdepthカメラの位地変換用定数
    x = 13
    angle = 0
    scale = 1.4

    #リサイズとファイル形式の変換
    color_image = cv2.resize(color_image, (w,h))
    depth_scale = cv2.resize(depth_scale, (w,h))
    ir_image = cv2.resize(ir_image, (w,h))
    ir_image = cv2.cvtColor(ir_image,cv2.COLOR_GRAY2RGB)

    #画角合わせ
    color_image = np.delete(color_image,slice(0,x),1)
    depth_scale = np.delete(depth_scale,slice(0,x),1)
    ir_image = cvpaste(ir_image, np.zeros((h,w,3)), 0, 0, angle, scale)
    ir_image = np.delete(ir_image,slice(w - x,w),1)

    #画角の結果に合わせた画像サイズ修正
    w = w - x

    #表示用画像と処理用データをそれぞれ生成
    depth_image = ScalarImage2RGB(depth_scale,minDistance,overDistance)
    depth_image[:,int(depth_image.shape[1] / 2),:] = np.array([255,0,0])
    ir_scale = cv2.cvtColor(ir_image,cv2.COLOR_RGB2GRAY)


    #デバッグ用
    if(False):
        result = [depth_image,0,00,0,0,0,0,0,0,0,0,0,0]
        brendImage = cv2.addWeighted(ir_image, viewScale / 100, depth_image, 1 - viewScale / 100, 0)
        return [np.vstack((np.hstack((color_image,depth_image)),np.hstack((ir_image,brendImage)))),result[1],result[2],result[3],result[4],result[5],result[6],result[7],XLog]

    #領域拡張法
    if(False):
        flag=RegionGrowing(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale)
        ir_image2 = ir_image.copy()
        ir_image2[np.where(flag == 1)] = [255,0,0]
        ir_image2[np.where(flag == 2)] = [0,255,0]
        ir_image2[np.where(flag == 3)] = [0,0,255]

    #赤外線画像とdepth画像から2値画像を生成する
    if(True):
        flag,binaryScale=WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale)
        ir_image2 = ir_image.copy()
        ir_image2[np.where(flag == 3)] = [0,0,255]#depth要素のある部分
        ir_image2[np.where(flag == 4)] = [0,255,255]#depth要素が遠い部分
        ir_image2[np.where(flag == 1)] = [255,0,0]#データが不正の位地 
        ir_image2[np.where(flag == 2)] = [0,255,0]#IR要素が小さい部分
        binaryScale = cv2.medianBlur(binaryScale,3)
        ir_image2 = cv2.cvtColor(binaryScale,cv2.COLOR_GRAY2RGB)   

    #得られた二値画像から旋回角を求める
    (maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,maxDoubel,PointList)=calcTurningAngle(binaryScale)
    
    print(maxX,maxY)
    #デバッグ用
    if(True):

        imageMap = ir_image.copy()
        step=12*2
        for x in range(w):
            for y in range(h):
                imageMap[y,x,:] = [PointList[min(int(h/step),int((y+12)/step))][min(int(w/step),int((x+12)/step))],0,0]
        if(5<maxX and maxX<w-5 and 5<maxY and maxY<h-5):
            x=maxX
            y=maxY

            maxAngle1=maxAngle1
            maxAngle2=maxAngle2
            maxAngle3=maxAngle3
            
            thickness=5
            topAngle1=maxAngle1
            topAngle2=maxAngle2
            topAngle3=maxAngle3


            theta = maxAngle1
            ir_image2=DrawAngleLine(ir_image2,x,y,topAngle1,(255,100,100,50),thickness)
            ir_image2 = cv2.putText(ir_image2,str(int(topAngle1)),(x + int(100 * math.cos(math.radians(theta))),y + int(100 * math.sin(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(255,100,100))
            theta = maxAngle2
            ir_image2=DrawAngleLine(ir_image2,x,y,topAngle2,(100,255,100,50),thickness)
            ir_image2 = cv2.putText(ir_image2,str(int(topAngle2)),(x + int(100 * math.cos(math.radians(theta))),y + int(100 * math.sin(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(100,255,100))
            theta = maxAngle3
            ir_image2=DrawAngleLine(ir_image2,x,y,topAngle3,(100,100,255,50),thickness)
            image2 = cv2.putText(ir_image2,str(int(topAngle3)),(x + int(100 * math.cos(math.radians(theta))),y + int(100 * math.sin(math.radians(theta)))),cv2.FONT_HERSHEY_PLAIN,1,(100,100,255))
            ir_image2 = cv2.putText(ir_image2,str(int(topAngle1)) + "/" + str(int(topAngle2)) + "/" + str(int(topAngle3)),(0,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
            ir_image2[y-5:y+5,x-5:x+5,:] = [255,255,0]




        result = [depth_image,0,00,0,0,0,0,0,0,0,0,0,0]
        brendImage = cv2.addWeighted(imageMap, viewScale / 100, ir_image2, 1 - viewScale / 100, 0)
        return [np.vstack((np.hstack((color_image,depth_image)),np.hstack((ir_image,brendImage)))),result[1],result[2],result[3],result[4],result[5],result[6],result[7],XLog]


    result = ImageReconition(binaryScale)
    double_image = np.hstack((result[0],ir_image2))
    return [double_image,result[1],result[2],result[3],result[4],result[5],result[6],result[7],XLog]


def ResetLog():
    for i in range(5):
        fortuneLog[i] = 0
        GammalAngleLog[i] = 0
        TurnAngleLog[i] = 0
        RangleLog[i] = 0
        LangleLog[i] = 0
        XLog[i] = 0

def diff(a,b):
    if(a > b):return a - b
    else: return b - a
