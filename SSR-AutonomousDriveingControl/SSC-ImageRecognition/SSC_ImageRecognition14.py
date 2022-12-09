import cv2 #pip install opencv-python
import numpy as np #pip install numpy
import math
import time
from Utilty import L_abs_minimize,reg1dim,getUnitValue,CalcDiffAngle,CalcDiffAngleNP,getWeightedLineArray,getDiskArray,cvpaste,CreteViewImage,ScalarImage2RGB,CalcDiffAngle
from functools import lru_cache

@lru_cache()
def getAngleList(anglestep,targetAngle,ruleAngle):
    angles = np.array(range(0,360,anglestep))
    angles = angles[np.where(((190 < angles) & (angles < 260)) | ((280 < angles) & (angles < 350)))]
    angles = angles[CalcDiffAngleNP(angles,targetAngle,ruleAngle)]    
    return angles

#分岐点位置を与えたときにその位置を評価する。boaderscoreを超えられないことがわかれば即座にリターン
def CalcScore(field,x,y,anglestep,thickness,boaderscore):
    h, w = field.shape
    maxValue = 0
    maxValue1 = 0
    maxValue2 = 0
    maxValue3 = 0
    maxAngle1 = -1
    maxAngle2 = -1
    ruleAngle = 80
    angle3 = 90

    angles = getAngleList(anglestep,angle3,ruleAngle)
    scores = np.array([np.sum(field[(getWeightedLineArray(angle,300,thickness,y,x,h,w))]) for angle in angles])
    score3 = np.sum(field[(getWeightedLineArray(90,300,thickness,y,x,h,w))])

    idx = np.where((scores > 0))
    angles = angles[idx]
    scores = scores[idx]

    idx = np.argsort(scores)[::-1]
    angles = angles[idx]
    scores = scores[idx]

    for i in range(0,len(angles) - 1):
        if(scores[i] + scores[i + 1] < maxValue):break
        if(scores[i] + scores[i + 1] + score3 < boaderscore):continue
        idx = -1
        for j,angle in enumerate(angles[i:]):
            if(CalcDiffAngle(angle, angles[i]) >= ruleAngle):
                idx = j
                break
        if(idx == -1):continue

        score1 = scores[i]
        score2 = scores[i:][idx]
        if(maxValue > score1 + score2):continue
        maxValue = score1 + score2
        maxAngle1 = angles[i]
        maxAngle2 = angles[i:][idx]
        maxValue1 = score1
        maxValue2 = score2



    return [maxAngle1,maxAngle2,angle3,maxValue + score3,maxValue1,maxValue2,score3]



def CalcTurningAngle(binryScale,step,branchsize,thickness):
    h, w = binryScale.shape
    plotMode = True
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
    anglestep = 3


    #解を調べる
    for x in range(int(w * 0.4),int(w * 0.6),step):
        for y in range(0,h,step):
            field = np.where(binryScale > 0,1,0)
            rr,cc = getDiskArray(y,x,branchsize,h,w)
            score = np.sum(field[rr,cc])
            field[rr,cc] = 0
            (angle1,angle2,angle3,value,value1,value2,value3) = CalcScore(field,x,y,anglestep,thickness,maxValue - score)
            value+=score
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

    return [maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3]

def CalcInclinationAngle(depthScale,y,x,angle1,angle2,angle3,minDistance,maxDistance,branchsize,thickness,h,w,branchX,branchY):
    InclinationAngle = 0
    
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

    #最小二乗法で平面の傾きを考える
    idx = np.nonzero(EffectiveDepthScale > 0)
    ydata = idx[0]
    Ymask = np.nonzero(ydata < branchY)
        
    data1_x = idx[1][Ymask]
    data1_depth = EffectiveDepthScale[idx][Ymask]
    maskL = np.nonzero((data1_x > branchX * 1.2))#リアル空間で左分岐=画像上で右分岐
    maskR = np.nonzero((data1_x < branchX * 0.7))

    if(len(data1_x[maskL]) < 2):(aL1,bL1) = (0,0)
    else:aL1,bL1 = L_abs_minimize(data1_x[maskL],data1_depth[maskL])
    if(len(data1_x[maskR]) < 2):(aR1,bR1) = (0,0)
    else:aR1,bR1 = L_abs_minimize(data1_x[maskR],data1_depth[maskR])
    leftY = bR1
    rightY = w * aR1 + bR1
    InclinationAngle = math.degrees(math.atan((rightY - leftY) / w))
    return InclinationAngle,EffectiveDepthScale




counter = 0
#赤外線画像とdepth画像を合成した二値画像を生成する
def WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale):
    global counter
    counter+=1
    #np.savetxt('result/data'+str(counter)+".csv", depth_scale)
    resultScale = np.zeros((h,w),dtype=np.uint8)
        

    #IR要素が小さい部分にフラグ2(緑)をつけ（だめな部分）、そうでない部分に5（黄色）をつける（不明な部分）
    flag = np.where((ir_scale <= 100) ,2,5)

    #遠くのdepth要素がある部分にフラグ4（水色）を付ける（だめな部分）
    flag = np.where((depth_scale > overDistance),4,flag)

    #データが不正の位置にフラグ1（赤）をつける（グレーな部分）
    flag = np.where((depth_scale == 0) ,1,flag)

    #depth要素のある部分にフラグ3（青）を付ける
    flag = np.where((1 <= depth_scale) & (depth_scale < maxDistance),3,flag)
    

    return flag

def IR(color_image,depth_image,ir_image,depth_scale,ir_scale,robot_rotation,minDistance,maxDistance,overDistance,extMode=True):
    #戻り値
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0

    #画角の結果に合わせた画像サイズ修正
    (h,w,d) = color_image.shape

    #赤外線画像とdepth画像から2値画像を生成する
    DepthIRFlag = WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale)
    #DepthIRFlag =depth_scale
    binaryScale = np.where(DepthIRFlag == 3,255,0).astype(np.uint8)
    binaryScale = cv2.medianBlur(binaryScale,3)

    #得られた二値画像から旋回角を求める
    step = 12
    branchsize = 24
    thickness = 5
    (maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3) = CalcTurningAngle(binaryScale,step,branchsize,thickness)

    LRE,EffectiveDepthScale = CalcInclinationAngle(depth_scale,maxY,maxX,maxAngle1,maxAngle2,maxAngle3,minDistance,maxDistance,branchsize,thickness,h,w,maxX,maxY)
    angle1L = maxAngle1 - 90
    if(angle1L < 0):angle1L = maxAngle1
    
    angle2L = maxAngle2 - 90
    if(angle2L < 0):angle2L = maxAngle2
    
    #小さい方が左分岐角
    if(angle1L < angle2L):
        angleA = abs(maxAngle1 - 270)
        angleB = abs(maxAngle2 - 270)
    else:
        angleA = abs(maxAngle2 - 270)
        angleB = abs(maxAngle1 - 270)
    

    branchValue = maxValue1 + maxValue2 + maxValue3

    return [maxY,maxX,branchValue,LRE,angleA,angleB,EffectiveDepthScale,DepthIRFlag]



def ResetLog():
    for i in range(len(doubelLog)):
        doubelLog[i] = 0
        valueLog[i] = 0
        GammalAngleLog[i] = 0
        TurnAngleLog[i] = 0
        RangleLog[i] = 0
        LangleLog[i] = 0
        XLog[i] = 0

