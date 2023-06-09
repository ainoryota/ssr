

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
from skimage.draw import line #pip install scikit-image
from skimage.draw import disk #pip install scikit-image
from skimage import morphology #pip install scikit-image
from functools import lru_cache

#一次式で最小二乗近似する
def reg1dim(x, y):
    try:
        n = len(x)
        under = ((x ** 2).sum() - x.sum() ** 2 / n)
        if under == 0:a = 0
        else: a = (np.dot(x, y) - y.sum() * x.sum() / n) / under
        if n == 0:b = 0
        else:       b = (y.sum() - a * x.sum()) / n
    except:
        OutputController().msgPrint("reg1dim Error",len(x),":",len(y))
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
@lru_cache(maxsize=1000)
def CalcDiffAngle(angle1,angle2):
    ret = min(360,abs(angle1 - angle2))
    ret = min(ret,abs(angle1 - angle2 - 360))
    ret = min(ret,abs(angle1 - angle2 + 360))
    return ret

def CalcDiffAngleNP(angle1,angle2):
    angle1 = np.where(angle1 > 360,angle1 - 360,angle1)
    angle1 = np.where(angle1 > np.abs(angle1 - angle2),np.abs(angle1 - angle2),angle1)
    angle1 = np.where(angle1 > np.abs(angle1 - angle2 - 360),np.abs(angle1 - angle2 - 360),angle1)
    angle1 = np.where(angle1 > np.abs(angle1 - angle2 + 360),np.abs(angle1 - angle2 + 360),angle1)
    return angle1

def trapez(y,y0,w):
    return np.clip(np.minimum(y + 1 + w / 2 - y0, -y + 1 + w / 2 + y0),0,1)

def weighted_line(y1, x1, y2, x2, w, rmin=0, rmax=np.inf):
    # 傾き無限になるときはxとyを入れ替えて計算
    if abs(x2 - x1) < abs(y2 - y1) :
        # xとyを入れ替え
        xx, yy, val = weighted_line(x1, y1, x2, y2, w, rmin=rmin, rmax=rmax)
        return (yy, xx, val)

    # 点1x<点2xにする
    if x1 > x2 :
        return weighted_line(y2, x2, y1, x1, w, rmin=rmin, rmax=rmax)

    # The following is now always < 1 in abs
    slope = (y2 - y1) / (x2 - x1)

    # Adjust weight by the slope
    w *= 1

    # We write y as a function of x, because the slope is always <= 1
    # (in absolute value)
    x = np.arange(x1, x2 + 1, dtype=float)
    y = x * slope + (x2 * y1 - x1 * y2) / (x2 - x1)

    # Now instead of 2 values for y, we have 2*np.ceil(w/2).
    # All values are 1 except the upmost and bottommost.
    thickness = np.ceil(w / 2)
    yy = (np.floor(y).reshape(-1,1) + np.arange(-thickness - 1,thickness + 2).reshape(1,-1))
    xx = np.repeat(x, yy.shape[1])
    vals = trapez(yy, y.reshape(-1,1), w).flatten()
    
    yy = yy.flatten()

    # Exclude useless parts and those outside of the interval
    # to avoid parts outside of the picture
    #mask = np.logical_and.reduce((yy >= rmin, yy < rmax, vals > 0))

    #return (yy[mask].astype(int), xx[mask].astype(int), vals[mask])
    return (yy.astype(int), xx.astype(int), vals)

@lru_cache(maxsize=10000)
def getWeightedLineArray(theta,len,thickness,y,x,h,w):
    cos = math.cos(math.radians(theta))
    sin = math.sin(math.radians(theta))
    rr, cc,_ = weighted_line(0, 0,  int(len * sin),int(len * cos),thickness)
    mask = np.logical_and.reduce((rr + y >= 0, rr + y < h ,cc + x >= 0, cc + x < w))
    return (rr[mask] + y,cc[mask] + x)

@lru_cache(maxsize=5000)
def getDiskArray(y,x,r,h,w):
    rr,cc = disk((y,x), r, shape=(h,w))
    mask = np.logical_and.reduce((rr >= 0, rr < h ,cc >= 0, cc < w))
    return (rr[mask],cc[mask])


#分岐点位置を与えたときにその位置を評価する
def CalcScore(field,x,y,anglestep,thickness):
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


    ruleAngle = 30
    angle3 = 90
    angles = np.array(range(0,360,anglestep))
    angles = angles[np.where((CalcDiffAngleNP(angles,angle3) >= ruleAngle))]
    scores = np.array([np.sum(field[(getWeightedLineArray(angle,300,thickness,y,x,h,w))]) for angle in angles])
    maxValue3 = np.sum(field[(getWeightedLineArray(90,300,thickness,y,x,h,w))])

    idx = np.where((scores > 0))
    angles = angles[idx]
    scores = scores[idx]

    idx = np.argsort(scores)[::-1]
    angles = angles[idx]
    scores = scores[idx]


    for i in range(len(angles)):
        angle1 = angles[i]
        score1 = scores[i]
        idx = np.where(CalcDiffAngleNP(angles[i:],angle1) >= ruleAngle)
        if(len(idx[0]) == 0):continue

        angle2 = angles[i:][idx][0]
        score2 = scores[i:][idx][0]
        maxValue = score1 + score2 + maxValue3
        maxAngle1 = angle1
        maxAngle2 = angle2
        maxAngle3 = angle3
        maxValue1 = score1
        maxValue2 = score2
        break

    return [maxAngle1,maxAngle2,maxAngle3,maxValue,maxValue1,maxValue2,maxValue3]

def CalcTurningAngle(binryScale,step,branchsize,thickness):
    h, w = binryScale.shape

    maxValue = 0
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
    anglestep = 3

    PointList = np.zeros((int(h / step + 2),int(w / step + 2)))

    #解を調べる
    for x in range(int(w * 0.4),int(w * 0.6),step):
        maxDoubel = max(maxDoubel,np.sum(field[(getWeightedLineArray(90,300,thickness,0,x,h,w))]))
        for y in range(0,h,step):
            field = np.where(binryScale > 0,1,0)
            rr,cc = getDiskArray(y,x,branchsize,h,w)
            score = np.sum(field[rr,cc])
            field[rr,cc] = 0
            (angle1,angle2,angle3,value,value1,value2,value3) = CalcScore(field,x,y,anglestep,thickness)
            value+=score
            PointList[int(y / step)][int(x / step)] = value
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


    PointList = (PointList * 255 / maxValue).astype(np.uint8)
    return [maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,maxDoubel,PointList]

def CalcElevationAngle(depthScale,y,x,angle1,angle2,angle3,minDistance,maxDistance,branchsize,thickness,h,w,branchX,branchY):
    elevationAngle = 0
    
    #索道の触れうる部分のidxをすべて持ってくる
    #分岐点周りのデータ+索道3本のデータ
    rr1,cc1 = getDiskArray(y,x,branchsize,h,w)
    rr2,cc2 = getWeightedLineArray(angle1,300,thickness,y,x,h,w)
    rr3,cc3 = getWeightedLineArray(angle2,300,thickness,y,x,h,w)
    rr4,cc4 = getWeightedLineArray(angle3,300,thickness,y,x,h,w)

    #ケーブルが通る部分のdepth情報
    IsCableway = np.zeros((h,w),dtype=np.uint8)
    IsCableway[rr1,cc1] = 1
    IsCableway[rr2,cc2] = 1
    IsCableway[rr3,cc3] = 1
    IsCableway[rr4,cc4] = 1
    EffectiveDepthScale = np.where((depthScale >= minDistance) & (depthScale < maxDistance),depthScale,0) * IsCableway

    #描画用
    elevationImage = np.zeros((int(maxDistance - minDistance),w,3),dtype=np.uint8) + 100

    #最小二乗法で平面の傾きを考える
    idx = np.nonzero(EffectiveDepthScale > 0)
    ydata = idx[0]
    Ymask = np.nonzero(ydata < branchY)
    elevationImage[maxDistance - EffectiveDepthScale[idx] - 1,idx[1]] = [255,255,255]
    
    data1_x = idx[1][Ymask]
    data1_depth = EffectiveDepthScale[idx][Ymask]
    maskL = np.nonzero((data1_x > branchX))#リアル空間で左分岐=画像上で右分岐
    maskR = np.nonzero((data1_x < branchX))

    if(len(data1_x[maskL]) < 2):(aL1,bL1) = (0,0)
    else:aL1,bL1 = reg1dim(data1_x[maskL],data1_depth[maskL])
    if(len(data1_x[maskR]) < 2):(aR1,bR1) = (0,0)
    else:aR1,bR1 = reg1dim(data1_x[maskR],data1_depth[maskR])
    for x in range(w):
        Y1 = int(maxDistance - 1 - int(x * aL1 + bL1))
        Y2 = int(maxDistance - 1 - int(x * aR1 + bR1))
        if(x > branchX and Y1 < int(maxDistance - minDistance) and Y1 >= 0): elevationImage[Y1][x] = [255,0,255]
        if(x < branchX and Y2 < int(maxDistance - minDistance) and Y2 >= 0): elevationImage[Y2][x] = [0,255,255]
    leftY= bR1
    rightY=w * aR1 + bR1
    LRelevationAngle = math.degrees(math.atan((rightY-leftY)/w))
    

    LelevationAngle = math.degrees(math.atan(aL1))
    RelevationAngle = math.degrees(math.atan(aR1))
    return LelevationAngle,RelevationAngle,elevationImage,LRelevationAngle


c = 100
doubelLog = [0 for a in range(c)]
valueLog = [0 for a in range(c)]
GammalAngleLog = [0 for a in range(c)]
TurnAngleLog = [0 for a in range(c)]
RangleLog = [0 for a in range(c)]
LangleLog = [0 for a in range(c)]
XLog = [0 for a in range(c)]

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
    #OutputController().msgPrint("仰角:",math.degrees(math.atan(a1)) ,
    #tieAngle,"/旋回角:",-math.degrees(math.atan(a2)),"/R角:",newAngles[1] - 90 -
    #(newAngles[2] - 270),"/L角:", 90 - newAngles[0] + (newAngles[2] - 270))
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
    doubelLog.pop(0)
    doubelLog.append(fortunity)
    fortunity = sum(doubelLog) / 5
    OutputController().msgPrint("angle1",angle1,angle2,angle3)
    (GammalAngle,TurnAngle,Rangle,Langle) = getRobotAngle(img,y,x,angle1,angle2,angle3,rotation)
    OutputController().msgPrint("Newangle1",GammalAngle,TurnAngle,Rangle,Langle)

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


    #OutputController().msgPrint(w,h,Xshift,Yshift)
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
    #OutputController().msgPrint(img1_bg.shape)
    #OutputController().msgPrint(img2_fg.shape)
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
    Xdirects = [[-1,0,1],[-1,0,1],[-1,0,1]]
    Ydirects = [[-1,-1,-1],[0,0,0],[1,1,1]]
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

    idx = np.where((minDistance <= depth_scale) & (depth_scale < maxDistance))
    seeds = list(zip(idx[0],idx[1]))
    colorList = ir_scale[idx]
    colorList.sort()
    minColor = colorList[int(len(colorList) * 0.1)] * 0.9
    maxColor = colorList[int(len(colorList) * 0.9)] * 1.1

    counter = 0
    while len(seeds) > 0:
        (y,x) = seeds.pop(0)
        if x == 0 or y == 0 or x == w - 1 or y == h - 1: continue

        #depthが指定の範囲内にある隣接箇所で行ったことないところを拡張していく
        flagData = np.where(flag[y - 1:y + 2,x - 1:x + 2] == 0,1,0)#まだ通ってない場所だけ1
        if(np.sum(flagData) == 0):continue
        ir_scale_data = ir_scale[y - 1:y + 2,x - 1:x + 2]
        if(ir_scale[y][x] + 5 < minColor or maxColor < ir_scale[y][x] - 5):continue
        growArea = np.where((minColor < ir_scale_data) & (ir_scale_data < maxColor) & (abs(ir_scale[y][x] - ir_scale_data) <= 5),1,0) * flagData
        if(np.sum(growArea) == 0):continue
        xAll = (Xdirects + x) * growArea
        yAll = (Ydirects + y) * growArea

        #通ったflagを立て隣接箇所をseedsに追加
        seedPlus = [a for a in list(zip(yAll.ravel(),xAll.ravel())) if a != (0,0)]
        if len(seedPlus) > 0:
            a,b = zip(*seedPlus)
            flag[a,b] = 1
            seeds.extend(seedPlus)
    return flag

#赤外線画像とdepth画像を合成した二値画像を生成する
def WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale):
    resultScale = np.zeros((h,w),dtype=np.uint8)
        
    #depth要素のある部分にフラグ3を付ける
    flag = np.where((minDistance <= depth_scale) & (depth_scale < maxDistance),3,0)
        
    #遠くのdepth要素がある部分にフラグ4を付ける
    flag = np.where((depth_scale > overDistance),4,flag)

    #データが不正の位置にフラグ1をつける
    flag = np.where((depth_scale == 0) ,1,flag)
            
    #IR要素が小さい部分にフラグ2をつける
    flag = np.where((ir_scale <= 100) ,2,flag)

    binaryScale = np.where(flag == 3,255,0).astype(np.uint8)
    #binaryScale = np.where(flag == 0,ir_scale,binaryScale).astype(np.uint8)

    return (flag,binaryScale)

def DrawAngleLine(img,x,y,theta,color,thickness):
    return cv2.line(img,(x,y),(x + int(360 * math.cos(math.radians(theta))),y + int(360 * math.sin(math.radians(theta)))),color=color,thickness=thickness)

def CreteViewImage(img11,img12,img21,img22,img31,img32):
    a = np.hstack((img11,img12))
    b = np.hstack((img21,img22))
    c = np.hstack((img31,img32))
    result = np.vstack((a,b))
    result = np.vstack((result,c))

    return result
    

def IR(color_image,depth_scale,ir_image,robot_rotation,extMode=True):

    #索道などのパラメータ
    minDistance = 200
    maxDistance = 500
    overDistance = 3000

    #戻り値
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0

    #画像の標準サイズ
    w = 320
    h = 180

    #赤外線カメラとdepthカメラの位置変換用定数
    maxX = 13
    angle = 0
    scale = 1.4

    #リサイズとファイル形式の変換
    color_image = cv2.resize(color_image, (w,h))
    depth_scale = cv2.resize(depth_scale, (w,h))
    ir_image = cv2.resize(ir_image, (w,h))
    ir_image = cv2.cvtColor(ir_image,cv2.COLOR_GRAY2RGB)

    #画角合わせ
    color_image = np.delete(color_image,slice(0,maxX),1)
    depth_scale = np.delete(depth_scale,slice(0,maxX),1)
    ir_image = cvpaste(ir_image, np.zeros((h,w,3)), 0, 0, angle, scale)
    ir_image = np.delete(ir_image,slice(w - maxX,w),1)

    #画角の結果に合わせた画像サイズ修正
    w = w - maxX

    #表示用画像と処理用データをそれぞれ生成
    depth_image = ScalarImage2RGB(depth_scale,minDistance,overDistance)
    depth_image[:,int(depth_image.shape[1] / 2),:] = np.array([255,0,0])
    ir_scale = cv2.cvtColor(ir_image,cv2.COLOR_RGB2GRAY)

    #デバッグ用
    if(False):
        result = [depth_image,0,00,0,0,0,0,0,0,0,0,0,0]
        brendImage = cv2.addWeighted(ir_image, viewRate / 100, depth_image, 1 - viewRate / 100, 0)
        return [np.vstack((np.hstack((color_image,depth_image)),np.hstack((ir_image,brendImage)))),result[1],result[2],result[3],result[4],result[5],result[6],result[7],XLog]

    #領域拡張法
    if(False):
        flag = RegionGrowing(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale)
        ir_image2 = ir_image.copy()
        ir_image2[np.where(flag == 1)] = [255,0,0]
        ir_image2[np.where(flag == 2)] = [0,255,0]
        ir_image2[np.where(flag == 3)] = [0,0,255]

    #赤外線画像とdepth画像から2値画像を生成する
    if(True):
        flag,binaryScale = WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale)
        imageMap = ir_image.copy()
        imageMap[np.where(flag == 3)] = [0,0,255]#depth要素のある部分
        imageMap[np.where(flag == 4)] = [0,255,255]#depth要素が遠い部分
        imageMap[np.where(flag == 1)] = [255,0,0]#データが不正の位置
        imageMap[np.where(flag == 2)] = [0,255,0]#IR要素が小さい部分
        binaryScale = cv2.medianBlur(binaryScale,3)
        ir_image2 = cv2.cvtColor(binaryScale,cv2.COLOR_GRAY2RGB)

    #得られた二値画像から旋回角を求める
    step = 12
    branchsize = 24
    thickness = 5
    (maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,maxDoubel,PointList) = CalcTurningAngle(binaryScale,step,branchsize,thickness)
    doubelLog.append(maxDoubel)
    doubelLog.pop(0)
    valueLog.append(maxValue1 + maxValue2 + maxValue3)
    valueLog.pop(0)
    OutputController().msgPrint(maxValue1 + maxValue2 + maxValue3,maxDoubel)
    XLog.append(maxY)
    XLog.pop(0)
    
    #OutputController().msgPrint("doubelLog")
    #s = ""
    #for i in range(len(doubelLog)):
    #    s+='{:.2f}'.format(doubelLog[i]) + ","
    #OutputController().msgPrint(s)

    #OutputController().msgPrint("valueLog")
    #s = ""
    #for i in range(len(valueLog)):
    #    s+='{:.2f}'.format(valueLog[i]) + ","
    #OutputController().msgPrint(s)

    #OutputController().msgPrint("XLog")
    #s = ""
    #for i in range(len(XLog)):
    #    s+='{:.2f}'.format(XLog[i]) + ","
    #OutputController().msgPrint(s)

    #分岐の条件(N1=30、N2=10)
    #1.直近N1ログ以内のvalueLog平均値が300以上
    #3.直近N1ログ以内のvalueLog平均値が300以上の結果から推定された5ログあとのXLogの線形近似解がhを超えた
    N1 = 30
    N2 = 10

    hoge1 = np.array(valueLog[len(valueLog) - N1:])
    rule1 = np.average(hoge1) / 300
    OutputController().msgPrint(valueLog)
    OutputController().msgPrint("rule1達成率",rule1)

    #p = np.zeros(N2)
    #for i in range(N2):
    #    if(doubelLog[len(doubelLog) - N2 + i] == 0):p[i] = 1
    #    else:p[i] = doubelLog[len(doubelLog) - N2 + i ] /
    #    doubelLog[len(doubelLog) - N2 + i- 1]

    #if(np.average(p) == 0):rule2 = 0
    #else:rule2 = 0.8 / np.average(p)
    #OutputController().msgPrint("rule2達成率",rule2)
    #OutputController().msgPrint(p)

    hoge2 = np.array(XLog[len(XLog) - N1:])

    
    l1 = np.array([n for n in range(N1)])[np.nonzero(hoge1 > 300)]
    l2 = hoge2[np.nonzero(hoge1 > 300)]
    
    if(len(l1) < 2):rule3 = 0
    else:
        a,b = reg1dim(l1,l2) 
        rule3 = (b + a * (N1 + 5)) / h
        OutputController().msgPrint(a,b,rule3)
    OutputController().msgPrint(XLog)
    OutputController().msgPrint("rule3達成率",rule3)

    LElevationAngle,RElevationAngle,ElevationImage,LRE = CalcElevationAngle(depth_scale,maxY,maxX,maxAngle1,maxAngle2,maxAngle3,minDistance,maxDistance,branchsize,thickness,h,w,maxX,maxY)
    #ElevationImage=ir_image2.copy()
    #ElevationAngle=0
    #OutputController().msgPrint(maxX,maxY,LElevationAngle,RElevationAngle)

    #表示
    thickness = 5
    angleA = abs(maxAngle1 - 270)
    angleB = abs(maxAngle2 - 270)
    if(angleA > angleB):(angleB,angleA) = (angleA,angleB)
    cableway_image = DrawAngleLine(np.zeros((h,w,3),dtype=np.uint8),maxX,maxY,maxAngle1,(255,100,100,50),thickness)
    cableway_image = DrawAngleLine(cableway_image,maxX,maxY,maxAngle2,(100,255,100,50),thickness)
    cableway_image = DrawAngleLine(cableway_image,maxX,maxY,maxAngle3,(100,100,255,50),thickness)
    cableway_image = cv2.putText(cableway_image,str(int(angleA)) + "/" + str(int(angleB)),(0,160),cv2.FONT_HERSHEY_PLAIN,2,(255,255,255))
    rr,cc = disk((maxY,maxX), 24, shape=(h,w))
    mask = np.logical_and.reduce((rr >= 0, rr < h ,cc >= 0, cc < w))
    cableway_image[rr[mask],cc[mask]] = [255,255,0]
    brendImage = cv2.addWeighted(ir_image2, 0.5, cableway_image, 0.5, 0)

    #img,x,y,fortunity,GammalAngle,TurnAngle,Rangle,Langle,XLog
    return [CreteViewImage(color_image,depth_image,ir_image,brendImage,cvpaste(imageMap, np.zeros(ElevationImage.shape), 0, 0, 0,1),ElevationImage),maxY,maxX,rule1,rule3,LElevationAngle,RElevationAngle,LRE,angleA,angleB]



def ResetLog():
    for i in range(len(doubelLog)):
        doubelLog[i] = 0
        valueLog[i] = 0
        GammalAngleLog[i] = 0
        TurnAngleLog[i] = 0
        RangleLog[i] = 0
        LangleLog[i] = 0
        XLog[i] = 0

def diff(a,b):
    if(a > b):return a - b
    else: return b - a
