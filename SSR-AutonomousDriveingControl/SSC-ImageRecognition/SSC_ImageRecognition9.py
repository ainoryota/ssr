
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
from numba import jit, f8, i8, b1, void

@jit(nopython=False)
def IR(color_image,depth_image,ir_image,robot_rotation):

    minDistance = 0
    maxDistance = 600

    #640×320で入力された画像を切り出し
    color_image = color_image[0:320,320:640]
    depth_image = depth_image[0:320,320:640]
    ir_image = ir_image[0:320,320:640]

    if(False):
        return [np.hstack((depth_image,ir_image)),0,0,0,0,0,0,0,XLog]
    
    #OpenCV形式に変換
    depth_image = np.where(depth_image > maxDistance,0,depth_image)#一定以上の距離のデータは無視
    depth_image = cv2.cvtColor((cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08,beta =0), cv2.COLORMAP_JET)),cv2.COLOR_RGB2GRAY)
    _,depth_image = cv2.threshold(depth_image,38,255,cv2.THRESH_TOZERO)
    depth_image = cv2.cvtColor(depth_image,cv2.COLOR_GRAY2RGB)
    ir_image = cv2.cvtColor(ir_image,cv2.COLOR_GRAY2RGB)
    
    #フィルタ
    color_image = cv2.medianBlur(color_image,9)
    depth_image = cv2.medianBlur(depth_image,9)
    ir_image = cv2.medianBlur(ir_image,9)
    

    #画像処理
    result = ImageReconition(depth_image,ir_image,robot_rotation)
    double_image = np.hstack((result[0],ir_image))
    x = result[1]
    y = result[2]
    fortunity = result[3]
    GammalAngle = result[4]
    TurnAngle = result[5]
    Rangle = result[6]
    Langle = result[7]

    return [double_image,x,y,fortunity,GammalAngle,TurnAngle,Rangle,Langle,XLog]

def cv2pil(image):
    ''' OpenCV型 -> PIL型 '''
    new_image = image.copy()
    if new_image.ndim == 2:  # モノクロ
        pass
    elif new_image.shape[2] == 3:  # カラー
        new_image = cv2.cvtColor(new_image, cv2.COLOR_BGR2RGB)
    elif new_image.shape[2] == 4:  # 透過
        new_image = cv2.cvtColor(new_image, cv2.COLOR_BGRA2RGBA)
    new_image = Image.fromarray(new_image)
    return new_image

def IRRecognition(ir_image):
    ir_image = cv2.medianBlur(ir_image,9)
    #画像処理


    result = ImageReconition(ir_image,ir_image,0)
    double_image = np.hstack((result[0],ir_image))
    x = result[1]
    y = result[2]
    fortunity = result[3]
    GammalAngle = result[4]
    TurnAngle = result[5]
    Rangle = result[6]
    Langle = result[7]
    cv2.imshow("title",double_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def ImageReconition(depth_img,ir_img,rotation):
    img = ir_img

    #水平方向角度の取得
    (value,angle1,angle2,angle3,value1,value2,value3,y,x,doubel_max) = getTurnAngle(img)

    OutputController().msgPrint(angle1,angle2,angle3,value)

    #仰角の取得
    (topAngle1,topAngle2,topAngle3) = getTopAngle(img,y,x,angle1,angle2,angle3)

    #信頼度の導出
    fortunity = min((value1,value2,value3)) / max(1,value1 + value2 + value3)
    fortunity = 1 - doubel_max / max(1,value1 + value2 + value3)
    fortuneLog.pop(0)
    fortuneLog.append(fortunity)
    fortunity = sum(fortuneLog) / 5
    (GammalAngle,TurnAngle,Rangle,Langle) = getRobotAngle(img,y,x,angle1,angle2,angle3,rotation)

    #角度の平均化
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

    #描画
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

@jit(nopython=False)
def getTurnAngle(img):
    height, width, channels = img.shape[:3]
    field = img[:,:,0]
    #field = np.where(np.any(img > 0,2) == 0,0,1)

    value=-10000
    angle1=-1
    angle2=-1
    angle3=-1
    value1=-1
    value2=-1
    value3=-1
    x=0
    y=0
    doubel=-1


    #解を調べる
    (value,angle1,angle2,angle3,value1,value2,value3,x,y,doubel) = solveOptimizedScore(field,0,width,0,height,24,10)
    OutputController().msgPrint("aaaa",x,y,width,height)
    (value,angle1,angle2,angle3,value1,value2,value3,x,y,doubel) = solveOptimizedScore(field,x-24,x+24,y-24,y+24,5,5)

    return (value,angle1,angle2,angle3,value1,value2,value3,int(x),int(y),doubel) 

@jit(nopython=False)
def solveOptimizedScore(field,minX,maxX,minY,maxY,gridStep,angleStep):
    bestValue=-10000
    bestAngle1=-1
    bestAngle2=-1
    bestAngle3=-1
    bestValue1=-1
    bestValue2=-1
    bestValue3=-1
    bestX=0
    bestY=0
    bestDoubel=-1

    for x in range(int(minX),int(maxX),gridStep):
        for y in range(int(minY),int(maxY),gridStep):
            (value,angle1,angle2,angle3,value1,value2,value3,x,y,doubel)=CalcScore(field,x,y,angleStep)
            #OutputController().msgPrint(x,y,value,bestValue)
            if(bestValue<value):
                bestValue=value
                bestAngle1=angle1
                bestAngle2=angle2
                bestAngle3=angle3
                bestValue1=value1
                bestValue2=value2
                bestValue3=value3
                bestX=x
                bestY=y
                bestDoubel=doubel
    return [bestValue,bestAngle1,bestAngle2,bestAngle3,bestValue1,bestValue2,bestValue3,bestX,bestY,bestDoubel]

#分岐点位置を与えたときにその位置を評価する
@jit(nopython=False)
def CalcScore(field,x,y,angleStep):
    doubel_max = 0
    angleData=getAngleData(field,x,y,angleStep)

    count = np.count_nonzero(angleData > 0,0)[1]
    angleData = angleData[np.argsort(angleData[:,1])][::-1]
    angle270Score = 0

    bestValue=-10000
    bestAngle1=-1
    bestAngle2=-1
    bestAngle3=-1
    bestValue1=-1
    bestValue2=-1
    bestValue3=-1
    bestX=0
    bestY=0
    bestDoubel=-1

    for i in range(count):
        if(angleData[i,0] == 270):
            angle270Score = angleData[i,1]
            break

    for i in range(count):
        angle1 = int(angleData[i,0])            
        for j in range(i,count):
            angle2 = int(angleData[j,0])
            if(CalcDiffAngle(angle1,angle2) < 60):continue
            if(angleData[i,1] + angleData[j,1] > doubel_max):
                doubel_max = angleData[i,1] + angleData[j,1]
            if(angleData[i,1] + angleData[j,1] + angleData[0,1] < bestValue):break
            angle3 = 270
            if(CalcDiffAngle(angle2,angle3) < 60 or CalcDiffAngle(angle1,angle3) < 60):continue

            value = angleData[i,1] + angleData[j,1] + angle270Score
            if(bestValue<value):
                bestValue=value
                bestAngle1=angle1
                bestAngle2=angle2
                bestAngle3=angle3
                bestValue1=angleData[i,1]
                bestValue2=angleData[j,1]
                bestValue3=angle270Score
                bestX=x
                bestY=y
                bestDoubel=doubel_max
                break
    OutputController().msgPrint("■",bestValue,bestX,bestY,bestAngle1,bestAngle2,bestAngle3,bestValue1,bestValue2,bestValue3)
    return [bestValue,bestAngle1,bestAngle2,bestAngle3,bestValue1,bestValue2,bestValue3,bestX,bestY,bestDoubel]

#ある(x,y)に対してすべての角度に対する評価値のlistを返す
@jit(nopython=False)
def getAngleData(field,x,y,angleStep):
    height, width = field.shape

    cable_size = 24
    num = int(360 / angleStep)
    angleData = np.zeros((num,2))

    i = -1
    while i < num-1:
        i = i + 1
        angleData[i,0] = angleStep * i
                
        cos = math.cos(math.radians(angleStep * i))
        sin = math.sin(math.radians(angleStep * i))

        r = 0
        counter=0
        while True:
            r+=1
            originalX = x + r * cos
            originalY = y + r * sin
            if(originalX >= width or originalX < 0 or originalY >= height or originalY < 0):
                break

            for direction in [-1,1]:
                for thick in range(0,direction*cable_size,direction):
                    X1 = int(x + r * cos - thick * sin)#cos(theta+90)
                    Y1 = int(y + r * sin + thick * cos)#sin(theta+90)
                    if(X1 >= width or X1 < 0):break
                    if(Y1 >= height or Y1 < 0):break
                    if(field[X1,Y1] == 0):continue

                    X2 = int(x + (r-1) * cos - thick * sin)#cos(theta+90)
                    Y2 = int(y + (r-1) * sin + thick * cos)#sin(theta+90)
                    if(X2 >= width or X2 < 0):break
                    if(Y2 >= height or Y2 < 0):break

                    a=field[X1,Y1]
                    b=field[X2,Y2]
                        
                    if(a>b):
                        c=a-b
                    else:
                        c=b-a
                    
                    counter+=(1 - abs(float(thick) / cable_size) ** 2)
                    #c=255-a
                    #OutputController().msgPrint(x,y,X1,Y1,c)
                    #angleData[i,1]+=(1 - abs(float(thick) / cable_size) ** 2)
                    angleData[i,1]+=(1 - abs(float(thick) / cable_size) ** 2)*(255-c)
                    #if(x==24 and y==168 and (angleStep*i==30 or angleStep*i==330)):
                        #OutputController().msgPrint("AAAAAAA:",(1 - abs(float(thick) / cable_size) ** 2)*(255-c))
                        #OutputController().msgPrint("NOW!",angleStep,int(field[X1,Y1]),int(field[X2,Y2]),int(field[X1,Y1])-int(field[X2,Y2]),X1,Y1)

                    #OutputController().msgPrint(angleData[i,0],(1 - abs(float(thick) / cable_size) ** 2)*(255-abs(field[X1,Y1]-field[X2,Y2])))
                    #OutputController().msgPrint((1 - abs(float(thick) / cable_size) ** 2)*(255-abs(field[X1,Y1]-field[X2,Y2])))
        
        if(counter!=0):
            angleData[i,1]/=counter

        OutputController().msgPrint(x,y,angleStep * i,i,angleData[i,1])
        
    return angleData


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
        OutputController().msgPrint("reg1dim Error")
        a = 0
        b = 0

    return a, b

#unitの倍数になるようvalueを調節する
@jit(i8(f8,f8))
def getUnitValue(value,unit):
    try:
        if(value > 0):    return int((value + unit / 2) / unit) * unit
        else:    return int((value - unit / 2) / unit) * unit
    except:
        OutputController().msgPrint("getUnitValue Error")
        return 0


#2つの角度の差を求める
@jit(f8(f8,f8))
def CalcDiffAngle(angle1,angle2):
    ret = min(360,abs(angle1 - angle2))
    ret = min(ret,abs(angle1 - angle2 - 360))
    ret = min(ret,abs(angle1 - angle2 + 360))
    return ret




fortuneLog = [0,0,0,0,0]
GammalAngleLog = [0,0,0,0,0]
TurnAngleLog = [0,0,0,0,0]
RangleLog = [0,0,0,0,0]
LangleLog = [0,0,0,0,0]
XLog = [0,0,0,0,0]

@jit(nopython=False)
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
    
    
    angle1x = np.empty(1)
    angle1y = np.empty(1)
    angle2x = np.empty(1)
    angle2y = np.empty(1)
    angle3x = np.empty(1)
    angle3y = np.empty(1)

    
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
    #div = min(g,rotation.z) / g
    div = min(g,0) / g
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





def ResetLog():
    for i in range(5):
        fortuneLog[i] = 0
        GammalAngleLog[i] = 0
        TurnAngleLog[i] = 0
        RangleLog[i] = 0
        LangleLog[i] = 0
        XLog[i] = 0
