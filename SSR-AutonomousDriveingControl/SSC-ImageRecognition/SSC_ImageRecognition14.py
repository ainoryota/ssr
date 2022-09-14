


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
import scipy.ndimage
from scipy.optimize import minimize
from Utilty import cvpaste

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
        print("reg1dim Error",len(x),":",len(y))
        a = 0
        b = 0

    return a, b


#一次式で最小絶対値法近似する。かなり重たいので高速化の必要あり
def L_abs_minimize(x, y):
    init = np.array([0.0, 0.0])
    def loss(args):
        a, b = args
        return np.sum(np.abs(y - (a*x+b)))
    ret = minimize(loss, init)
    return  ret.x[0],ret.x[1]

#unitの倍数になるようvalueを調節する
def getUnitValue(value,unit):
    try:
        if(value > 0):    return int((value + unit / 2) / unit) * unit
        else:    return int((value - unit / 2) / unit) * unit
    except:
        print("getUnitValue Error")
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


    ruleAngle = 100
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


# ハフ変換
def houghConvertSub(theta, x, y):
    distance = []
    for t in theta:
        d = x * math.cos(t) + y * math.sin(t)
        distance.append(d)
    return distance

def plotLine(dst, x1, y1, x2, y2, broken=1, variance=1):
    xd = x2 - x1
    yd = y2 - y1
    xsize = len(dst[0])
    ysize = len(dst)

    if xd == 0:
        if y1 > y2:
            y1, y2 = y2, y1
        for i in range(int(y1), int(y2), 1):
            dst[i][x1] = 1
        return

    if yd == 0:
        if x1 > x2:
            x1, x2 = x2, x1
        for i in range(int(x1), int(x2), 1):
            dst[y1][i] = 1
        return

    rad = math.atan2(yd, xd)
    distance = math.sqrt((xd ** 2) + (yd ** 2))
    for i in range(int(distance)):
        if broken >= 2 and not (i % broken) == 0: continue
        x = x1 + i * math.cos(rad)
        y = y1 + i * math.sin(rad)
        if variance > 0:
            x += int(random.random() * variance) - (variance / 2)
            y += int(random.random() * variance) - (variance / 2)
        if x < 0: x = 0
        if x >= xsize: x = xsize - 1
        if y < 0: y = 0
        if y >= ysize: y = ysize - 1
        dst[int(y)][int(x)] = 1


def detectPeak1D(x):
    conv1 = np.convolve(x, [1, -1], mode="full")
    flag1 = (conv1 > 0).astype(int)
    flag2 = (conv1 <= 0).astype(int)
    flag1 = flag1[:-1]
    flag2 = flag2[1:]
    flag3 = flag1 & flag2
    return flag3

def detectPeak2D(x):
    peaks1 = []
    for ix in x:
        peak = detectPeak1D(ix)
        peaks1.append(peak)
    peaks1 = np.array(peaks1)
    peaks2 = []
    for ix in x.transpose():
        peak = detectPeak1D(ix)
        peaks2.append(peak)
    peaks2 = np.array(peaks2).transpose()
    flag = (peaks1 & peaks2).astype(int)
    return flag

def HoughConvert(imageData): # https://qiita.com/ikuo0/items/c591b61fe8a07546688b
    plotMode=False
    PY, PX = np.where(imageData == 255) # # ピクセル座標の取得
    DegreeDiscrete = 180 # pi をいくつに分割するか（粒度をどの程度にするか）、100とか200でも良い
    theta_arr = [math.pi * (i / DegreeDiscrete) for i in range(0, DegreeDiscrete)]
    PR = []
    for x, y in zip(PX, PY):
        p = houghConvertSub(theta_arr, x, y)
        PR.append(p)

    PR = np.array(PR)
    if plotMode:
        plt.plot(theta_arr, PR.transpose())
        plt.xlabel("Theta")
        plt.ylabel("r")#ハフ空間（波形全体） https://ja.wikipedia.org/wiki/%E3%83%8F%E3%83%95%E5%A4%89%E6%8F%9B　正弦波同士が重なった個数が多いところが直線。
        plt.show()



    # 2次元配列にして交差している点をカウントする
    # 対角線の長さを求め距離の最大値とする
    CANVAS_WIDTH = imageData.shape[1]
    CANVAS_HEIGHT = imageData.shape[0]

    diagonalDistance = math.sqrt((CANVAS_WIDTH ** 2) + (CANVAS_HEIGHT ** 2))
    diagonalDistance = round(diagonalDistance)

    # 交差点の交差数を投票する2次元配列を作成
    vote = np.zeros((diagonalDistance * 2, len(theta_arr)))

    for P in PR:
        for x, p in enumerate(P):
            y = int(round(p))
            vote[y + diagonalDistance][x] += 1

    # 標準化しておく
    vote = (vote - np.mean(vote)) / np.std(vote)

    # 角度がループする都合上、先頭列と列の末尾はデータとしては連続している
    # 列末尾に先頭列をいくつか追加しておく事で一括で処理できるよう準備する
    appendMat = np.flipud(vote[:,:10])
    newVote = np.c_[vote, appendMat]
    print("newVote.shape", newVote.shape)
    # ガウシアンフィルタをかけて頂点をできるだけ検出しやすくする
    filteredVote = scipy.ndimage.gaussian_filter(newVote, sigma=0.5)

    if plotMode:
        plt.figure(figsize=(6,6),dpi=200)#アキュムレータ平面
        plt.xlabel("Theta")
        plt.ylabel("r")#ハフ空間（すべての波形を重ね合わせた） https://ja.wikipedia.org/wiki/%E3%83%8F%E3%83%95%E5%A4%89%E6%8F%9B　明るいところは正弦波同士が重なり合った個数が多いところ
        plt.imshow(vote, cmap='jet', interpolation='nearest')
        plt.show()

    # ピーク検出を行う
    # 誤検知については後の処理で投票数の少ない頂点を間引く事で対応する
    peaks = detectPeak2D(filteredVote)
    xidx, yidx = np.where(peaks == 1)
    print("np.max(filteredVote)", np.max(filteredVote))
    for x, y in zip(xidx, yidx):
        if filteredVote[x][y] > 10:
            peaks[x][y] = 1
        else:
            peaks[x][y] = 0

    if plotMode:
        plt.figure(figsize=(8,8),dpi=200)
        plt.imshow(peaks, cmap='spring', interpolation='nearest')
        plt.savefig("./007_haough_convert_peaks.png")
        plt.show()

    # ピークからθのインデックスを取り出す
    idxs = np.where(peaks != 0)
    peakValues = filteredVote[idxs]

    print(idxs[0])
    print(idxs[1])
    distances = idxs[0]
    thetaIndexes  =[]
    for idx in idxs[1]:
        if idx >= DegreeDiscrete:# 先頭末尾を繋げた部分について、インデックスを修正する
            idx = idx - DegreeDiscrete
        thetaIndexes.append(idx)

    distances = np.array(distances)
    thetaIndexes = np.array(thetaIndexes)

    print(distances)
    print(thetaIndexes)
    print("len(PX)", len(PX))
    print("len(PY)", len(PY))
    print("len(PR)", len(PR))
    #print("len(THETA)", len(theta))

    # ピークの上位n個だけを採用する
    n=2
    idxs = np.argsort(peakValues)[::-1]
    idxs = idxs[:n]
    distances = distances[idxs]
    thetaIndexes = thetaIndexes[idxs]

    # 検出された直線を実際に描画して確認する
    #y = -(math.cos(rad) / math.sin(rad)) * x + ((yidx[0] - diagonalDistance) / math.sin(rad))
    #x = -(math.sin(rad) / math.cos(rad)) * y + ((yidx[0] - diagonalDistance) / math.cos(rad))
    x = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH)).astype(np.uint8)

    threshold1 = math.pi * (3/4)
    threshold2 = math.pi * (1/4)
    for idx, d in zip(thetaIndexes, distances):
        rad = theta_arr[idx]
        #x= -y sinθ/cosθ +ρ/ cosθ
        #x1 = 
        if rad > (threshold1) or rad < (threshold2):
            y1 = 0
            y2 = CANVAS_HEIGHT
            x1 = -(math.sin(rad) / math.cos(rad)) * y1 + ((d - diagonalDistance) / math.cos(rad))
            x2 = -(math.sin(rad) / math.cos(rad)) * y2 + ((d - diagonalDistance) / math.cos(rad))
        else:
            x1 = 0
            x2 = CANVAS_WIDTH
            y1 = -(math.cos(rad) / math.sin(rad)) * x1 + ((d - diagonalDistance) / math.sin(rad))
            y2 = -(math.cos(rad) / math.sin(rad)) * x2 + ((d - diagonalDistance) / math.sin(rad))
        if plotMode:
            plotLine(x, int(x1), int(y1), int(x2), int(y2), broken=1, variance=0)
    if plotMode:
        plt.imshow(x)
        plt.savefig("./009_haough_convert_lines.png")
        plt.show()



def CalcTurningAngle(binryScale,step,branchsize,thickness):
    h, w = binryScale.shape
    plotMode=True
    maxValue = 0
    _,edges=cv2.threshold(binryScale,1,255,cv2.THRESH_BINARY)
    #edges = cv2.Canny(binryScale,0,1,apertureSize = 3)
    image=cv2.cvtColor(edges,cv2.COLOR_GRAY2RGB)
    

    field = np.where(binryScale > 0,1,0)

    # ハフ変換で直線を検出する
    #lines = cv2.HoughLines(edges,1,np.pi/180,10)
    #plt.subplot(121),plt.imshow(binryScale,cmap = 'gray')
    #plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    #plt.subplot(122),plt.imshow(edges,cmap = 'gray')
    #plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

    #plt.show()
    #if(not(lines is None)):
    #    HoughConvert(edges)
    #    overlay=image.copy()
    #    size=image.shape;
    #    alpha=0.05;
    #    lineImage1 = np.zeros(size, np.uint8)
        
    #    print("lines are",len(lines))
    #    for line in lines:
    #        lineImage2 = np.zeros(size, np.uint8)
    #        rho,theta = line[0]
    #        a = np.cos(theta)
    #        b = np.sin(theta)
    #        x0 = a*rho
    #        y0 = b*rho
    #        x1 = int(x0 + 1000*(-b))
    #        y1 = int(y0 + 1000*(a))
    #        x2 = int(x0 - 1000*(-b))
    #        y2 = int(y0 - 1000*(a))

            
    #        if plotMode:
    #            cv2.line(lineImage2,(x1,y1),(x2,y2),color=(0,255,0),thickness=2)
    #            lineImage1=cv2.addWeighted(lineImage1, 1, lineImage2, alpha , 0)
        
    #    if plotMode:
    #        alpha2=0.7
    #        plt.subplot(1,1,1)
    #        plt.imshow( cv2.addWeighted(image, alpha2, lineImage1, 1 - alpha2, 0),cmap = 'gray')
    #        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

    #        plt.show()
    #        #cv2.imwrite('houghlines3.jpg',image)


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

    print(maxX,maxY,maxAngle1,maxAngle2,maxValue1,maxValue2,maxValue)
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
    doubelLog.pop(0)
    doubelLog.append(fortunity)
    fortunity = sum(doubelLog) / 5
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
    

def IR(color_image,depth_scale,ir_image,ir_scale,robot_rotation,minDistance,maxDistance,overDistance,extMode=True):
    #戻り値
    GammalAngle = 0
    TurnAngle = 0
    Rangle = 0
    Langle = 0

    #画角の結果に合わせた画像サイズ修正
    h =color_image.shape[0]
    w =color_image.shape[1]

    #処理用データを生成
    ir_scale = cv2.cvtColor(ir_image,cv2.COLOR_RGB2GRAY)

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
        imageMap[np.where(flag == 5)] = [255,255,0]#テスト用
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
    XLog.append(maxY)
    XLog.pop(0)

    #分岐の条件(N1=30、N2=10)
    #1.直近N1ログ以内のvalueLog平均値が300以上
    #3.直近N1ログ以内のvalueLog平均値が300以上の結果から推定された5ログあとのXLogの線形近似解がhを超えた
    N1 = 30
    N2 = 10

    hoge1 = np.array(valueLog[len(valueLog) - N1:])
    rule1 = np.average(hoge1) / 300

    hoge2 = np.array(XLog[len(XLog) - N1:])

    
    l1 = np.array([n for n in range(N1)])[np.nonzero(hoge1 > 300)]
    l2 = hoge2[np.nonzero(hoge1 > 300)]
    
    if(len(l1) < 2):rule3 = 0
    else:
        a,b = reg1dim(l1,l2) 
        rule3 = (b + a * (N1 + 5)) / h


    LRE,EffectiveDepthScale = CalcInclinationAngle(depth_scale,maxY,maxX,maxAngle1,maxAngle2,maxAngle3,minDistance,maxDistance,branchsize,thickness,h,w,maxX,maxY)
    angleA = abs(maxAngle1 - 270)
    angleB = abs(maxAngle2 - 270)
    if(angleA > angleB):(angleB,angleA) = (angleA,angleB)

    return [maxY,maxX,LRE,angleA,angleB,EffectiveDepthScale]



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
