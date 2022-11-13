import cv2 #pip install opencv-python
import numpy as np #pip install numpy
import math
from Utilty import L_abs_minimize,reg1dim,getUnitValue,CalcDiffAngle,CalcDiffAngleNP,getWeightedLineArray,getDiskArray,cvpaste,CreteViewImage,ScalarImage2RGB

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


    ruleAngle = 80
    angle3 = 90
    angles = np.array(range(0,360,anglestep))
    angles = angles[np.where((CalcDiffAngleNP(angles,angle3) >= ruleAngle))]
    angles = angles[np.where(((190<angles)&(angles<260))|((280<angles)&(angles<350)))]
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
        if(maxValue > score1 + score2 + maxValue3):continue;
        maxValue = score1 + score2 + maxValue3
        maxAngle1 = angle1
        maxAngle2 = angle2
        maxAngle3 = angle3
        maxValue1 = score1
        maxValue2 = score2

    return [maxAngle1,maxAngle2,maxAngle3,maxValue,maxValue1,maxValue2,maxValue3]



def CalcTurningAngle(binryScale,step,branchsize,thickness):
    h, w = binryScale.shape
    plotMode=True
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
    maskL = np.nonzero((data1_x > branchX*1.2))#リアル空間で左分岐=画像上で右分岐
    maskR = np.nonzero((data1_x < branchX*0.7))

    if(len(data1_x[maskL]) < 2):(aL1,bL1) = (0,0)
    else:aL1,bL1 = L_abs_minimize(data1_x[maskL],data1_depth[maskL])
    if(len(data1_x[maskR]) < 2):(aR1,bR1) = (0,0)
    else:aR1,bR1 = L_abs_minimize(data1_x[maskR],data1_depth[maskR])
    leftY= bR1
    rightY=w * aR1 + bR1
    InclinationAngle = math.degrees(math.atan((rightY-leftY)/w))
    return InclinationAngle,EffectiveDepthScale




counter=0
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
    flag = np.where((depth_scale==0) ,1,flag)

    #depth要素のある部分にフラグ3（青）を付ける
    flag = np.where((1 <= depth_scale) & (depth_scale < maxDistance),3,flag)
    

    return flag

def IR(color_image,depth_iamge,ir_image,depth_scale,ir_scale,robot_rotation,minDistance,maxDistance,overDistance,extMode=True):
    #戻り値
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0

    #画角の結果に合わせた画像サイズ修正
    (h,w,d) =color_image.shape

    #赤外線画像とdepth画像から2値画像を生成する
    DepthIRFlag = WeightedIRImage(h,w,minDistance,maxDistance,overDistance,ir_scale,depth_scale)
    binaryScale = np.where(DepthIRFlag == 3,255,0).astype(np.uint8)
    binaryScale = cv2.medianBlur(binaryScale,3)

    #得られた二値画像から旋回角を求める
    step = 12
    branchsize = 24
    thickness = 5
    (maxX,maxY,maxAngle1,maxAngle2,maxAngle3,maxValue1,maxValue2,maxValue3,maxDoubel,PointList) = CalcTurningAngle(binaryScale,step,branchsize,thickness)

    LRE,EffectiveDepthScale = CalcInclinationAngle(depth_scale,maxY,maxX,maxAngle1,maxAngle2,maxAngle3,minDistance,maxDistance,branchsize,thickness,h,w,maxX,maxY)
    angleA = abs(maxAngle1 - 270)
    angleB = abs(maxAngle2 - 270)
    if(angleA > angleB):(angleB,angleA) = (angleA,angleB)
    branchValue=maxValue1 + maxValue2 + maxValue3

    return [maxY,maxX,branchValue,LRE,angleB,angleA,EffectiveDepthScale,DepthIRFlag]



def ResetLog():
    for i in range(len(doubelLog)):
        doubelLog[i] = 0
        valueLog[i] = 0
        GammalAngleLog[i] = 0
        TurnAngleLog[i] = 0
        RangleLog[i] = 0
        LangleLog[i] = 0
        XLog[i] = 0

