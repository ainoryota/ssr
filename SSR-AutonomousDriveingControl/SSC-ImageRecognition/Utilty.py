import time
import threading
from functools import wraps
import numpy as np
import cv2
from scipy.optimize import minimize
from functools import lru_cache
import math
from skimage.draw import line #pip install scikit-image
from skimage.draw import disk #pip install scikit-image
from skimage import morphology #pip install scikit-image
from PIL import Image
import os

def processing_time(func):
    @wraps(func)
    def wrapper(*args, **keywords):
        st = time.time()  # 開始前の時間を記録
        result = func(*args, **keywords)  # 関数を実行
        print(f'time: {time.time() - st} s')  # 開始後の時間と開始前の時間の差を出力
        return result

    return wrapper

#画像を別の画像に回転・拡大縮小しながら貼り付ける
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


def CreteViewImage(img11,img12,img21,img22,img31,img32):
    a = np.hstack((img11,img12))
    b = np.hstack((img21,img22))
    c = np.hstack((img31,img32))
    result = np.vstack((a,b))
    result = np.vstack((result,c))

    return result


def ScalarImage2RGB(img,ClipMinDistance,ClipMaxDistance):
    rate = 255.0 / (ClipMaxDistance - ClipMinDistance)
    img = np.where((img <= ClipMinDistance) | (img >= ClipMaxDistance),255,rate * (img - ClipMinDistance))
    return np.dstack([img,img,img]).astype(np.uint8)

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

#2つの角度の配列を与えると角度の差の配列を返す
def CalcDiffAngleNP(angle1,angle2):
    angle1 = np.where(angle1 > 360,angle1 - 360,angle1)
    angle1 = np.where(angle1 > np.abs(angle1 - angle2),np.abs(angle1 - angle2),angle1)
    angle1 = np.where(angle1 > np.abs(angle1 - angle2 - 360),np.abs(angle1 - angle2 - 360),angle1)
    angle1 = np.where(angle1 > np.abs(angle1 - angle2 + 360),np.abs(angle1 - angle2 + 360),angle1)
    return angle1

#太さのある線の配列を返す
@lru_cache(maxsize=10000)
def getWeightedLineArray(theta,len,thickness,y,x,h,w):
    def weighted_line(y1, x1, y2, x2, w, rmin=0, rmax=np.inf):
        def trapez(y,y0,w):
            return np.clip(np.minimum(y + 1 + w / 2 - y0, -y + 1 + w / 2 + y0),0,1)

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

    cos = math.cos(math.radians(theta))
    sin = math.sin(math.radians(theta))
    rr, cc,_ = weighted_line(0, 0,  int(len * sin),int(len * cos),thickness)
    mask = np.logical_and.reduce((rr + y >= 0, rr + y < h ,cc + x >= 0, cc + x < w))
    return (rr[mask] + y,cc[mask] + x)

#ディスク型の図形を描画する
@lru_cache(maxsize=5000)
def getDiskArray(y,x,r,h,w):
    rr,cc = disk((y,x), r, shape=(h,w))
    mask = np.logical_and.reduce((rr >= 0, rr < h ,cc >= 0, cc < w))
    return (rr[mask],cc[mask])

#角度を持った線を引く
def DrawAngleLine(img,x,y,theta,color,thickness):
    return cv2.line(img,(x,y),(x + int(360 * math.cos(math.radians(theta))),y + int(360 * math.sin(math.radians(theta)))),color=color,thickness=thickness)

#abs関数と同じ
def diff(a,b):
    if(a > b):return a - b
    else: return b - a

#minとmaxの間にvalueがあるか判定する
def IsArea(value,min,max):
    if(min <= value and value <= max):
        return True
    else:
        return False

#現在のパスから画像ファイルを開く
def getImageFromFile(filePath):
    return Image.open(os.path.abspath("C:/Users/MSD/Documents/GitHub/SSR/SSR-AutonomousDriveingControl/SSC-ImageRecognition/"+filePath))

#指定の桁数に丸める
def rounddown(value,n):
    return math.floor(value * 10 ** n) / (10 ** n)