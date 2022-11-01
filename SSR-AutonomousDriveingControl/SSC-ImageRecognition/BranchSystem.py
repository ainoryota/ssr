from SSC_ImageRecognition14 import IR
import cv2
from Utilty import cvpaste,CreteViewImage,ScalarImage2RGB,reg1dim,getImageFromFile,rounddown,disk,DrawAngleLine,getLikeAngle
import numpy as np
from PIL import Image,ImageTk
import math

class BranchSystem:
    def __init__(self):
        self.IsBranch = False
        self.color_image = np.zeros((1,1,3))#RGB
        self.ir_image1 = np.zeros((1,1,3))#RGB
        self.ir_image2 = np.zeros((1,1,3))#RGB
        self.ir_scale = np.zeros((1,1))
        self.depth_image = np.zeros((1,1,3))#RGB
        self.depth_scale = np.zeros((1,1))
        self.EffectiveDepthScale = np.zeros(307)#x軸方向に探査したときの有効な深度情報
        self.DepthIRFlag = np.zeros((1,1))
        
        self.Error = False
        self.y = 0
        self.x = 0
        self.InclinationAngle = 0
        self.tangle = 0
        self.Rangle = 0
        self.Langle = 0
        self.branchValue = 0

        self.LogNumber = 100
        self.valueLog = [0 for a in range(self.LogNumber)]
        self.YLog = [0 for a in range(self.LogNumber)]


        #索道などのパラメータ
        self.minDistance = 200
        self.maxDistance = 500
        self.overDistance = 2000

        self.w = 0
        self.h = 0

    def setImage(self,color_image,depth_image,ir_image1,ir_image2):
        self.color_image = color_image
        self.ir_image1 = ir_image1
        self.ir_image2 = ir_image2
        self.depth_image = ScalarImage2RGB(depth_image,self.minDistance,self.overDistance)
        self.depth_scale = depth_image
        self.adjustView()
        self.ir_scale = cv2.cvtColor(self.ir_image1.copy(),cv2.COLOR_RGB2GRAY)

    def adjustView(self):
        #画像の標準サイズ
        w = 320
        h = 180

        #赤外線カメラとdepthカメラの位置変換用定数
        maxX = 13
        angle = 0
        scale = 1.4
        self.w = w - maxX
        self.h = h

        #リサイズとファイル形式の変換
        self.color_image = cv2.resize(self.color_image, (w,h))
        self.depth_image = cv2.resize(self.depth_image, (w,h))
        self.ir_image1 = cv2.resize(self.ir_image1, (w,h))
        self.ir_image1 = ScalarImage2RGB(self.ir_image1,0,255)
        self.depth_scale = cv2.resize(self.depth_scale, (w,h))
        self.ir_image2 = cv2.resize(self.ir_image2, (w,h))
        self.ir_image2 = ScalarImage2RGB(self.ir_image2,0,255)


        #画角合わせ（赤外線カメラ2を除く）
        self.color_image = np.delete(self.color_image,slice(0,maxX),1)
        self.depth_image = np.delete(self.depth_image,slice(0,maxX),1)
        self.depth_scale = np.delete(self.depth_scale,slice(0,maxX),1)
        self.ir_image1 = cvpaste(self.ir_image1, np.zeros((h,w,3)), 0, 0, angle, scale)#若干拡大する
        self.ir_image1 = np.delete(self.ir_image1,slice(w - maxX,w),1)

    def calcCablewayInf(self,accel,extMode):
        self.Error = False
        self.IsBranch = False
        if(np.sum(self.ir_image1) < 0.1 or np.sum(self.color_image) < 0.1):#カメラがほぼ真っ暗
            self.Error = True
            print("Camera is black")
            return
        result = IR(self.color_image,self.depth_image,self.ir_image1,self.depth_scale,self.ir_scale,accel,self.minDistance,self.maxDistance,self.overDistance,extMode)
        if(len(result) == 1):#不正ならFalseだけが返ってくる
            self.Error = True
            return
        (self.y,self.x,self.branchValue,self.InclinationAngle,self.Rangle,self.Langle,self.EffectiveDepthScale,self.DepthIRFlag) = result
        self.appendLog()
        self.tangle = math.degrees(-math.atan2(accel.y,accel.z))

        (rule1,rule2) = self.calcRule()
        print("y=",self.y,"x=",self.x,"仰角Ele=",rounddown(self.tangle,1),"回転角Inc=",rounddown(self.InclinationAngle,1),"RightAngle=",rounddown(self.Rangle,0),"LAngle=",rounddown(self.Langle,0),"branchValue=",self.branchValue,"rule1=",rounddown(rule1,2),"rule2=",rounddown(rule2,2))
        if(rule1 > 1 and rule2 > 1):
            self.IsBranch = True

    def getOutputImage(self):
        if(self.Error):return  ImageTk.PhotoImage(getImageFromFile("test.png"))
        depth_view_image = ScalarImage2RGB(self.depth_scale,self.minDistance,self.overDistance)
        InclinationImage = self.getInclinationImage()
        cablewayImage = self.getCablewayImage()
        imageMap = np.zeros(InclinationImage.shape)
        depthIRImage = cvpaste(self.getDepthIRMap(), np.zeros(InclinationImage.shape), 0, 0, 0,1)
        image = CreteViewImage(self.color_image,depth_view_image,self.ir_image1,cablewayImage,depthIRImage,InclinationImage)
        return ImageTk.PhotoImage(Image.fromarray(image.astype(np.uint8)))

    def appendLog(self):
        self.valueLog.append(self.branchValue)
        self.valueLog.pop(0)
        self.YLog.append(self.y)
        self.YLog.pop(0)

    def calcRule(self):
        #1.直近N1ログ以内のvalueLog平均値が300以上
        #2.直近N1ログ以内のvalueLog平均値が300以上の結果から推定された5ログあとのYLogの線形近似解が高さhの1.2倍を超えた
        N1 = 30
        valueLogList = np.array(self.valueLog[len(self.valueLog) - N1:])
        rule1 = np.average(valueLogList) / 300

        hoge2 = np.array(self.YLog[len(self.YLog) - N1:])
        l1 = np.array([n for n in range(N1)])[np.nonzero(valueLogList > 300)]
        l2 = hoge2[np.nonzero(valueLogList > 300)]
    
        if(len(l1) < 2):rule2 = 0
        else:
            a,b = reg1dim(l1,l2) 
            rule2 = (b + a * (N1 + 5)) / (self.h)
        return (rule1,rule2)

    def getBranch(self):#分岐すべきならそのタイミングや角度を返す
        SleepTime = 0
        InclinationAngle = 0
        ElevationAngle = 0
        RTurningAngle = 0
        LTurningAngle = 0
        return [self.IsBranch,SleepTime,InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle]

    def ResetLog(self):
        for data in self.valueLog:
            data = 0
        return 0
    
    def getInclinationImage(self):
        InclinationImage = np.zeros((int(self.maxDistance - self.minDistance),self.w,3),dtype=np.uint8) + 100
        idx = np.nonzero(self.EffectiveDepthScale > 0)
        if(len(idx) > 1 and len(idx[0]) > 0 and len(idx[1]) > 0):
            InclinationImage[self.maxDistance - self.EffectiveDepthScale[idx] - 1,idx[1]] = [255,255,255]
        return InclinationImage

    def getDepthIRMap(self):
        DepthIRMap = self.ir_image1.copy()
        DepthIRMap[np.where(self.DepthIRFlag == 3)] = [0,0,255]#depthの大きさから索道と予想される部分
        DepthIRMap[np.where(self.DepthIRFlag == 4)] = [0,255,255]#depth要素が遠い部分
        DepthIRMap[np.where(self.DepthIRFlag == 1)] = [255,0,0]#データが不正の位置
        DepthIRMap[np.where(self.DepthIRFlag == 2)] = [0,255,0]#IR要素が小さい部分
        DepthIRMap[np.where(self.DepthIRFlag == 5)] = [255,255,0]#テスト用
        return DepthIRMap

    def getCablewayImage(self):
        depthExtendImage = np.zeros(self.depth_image.shape)
        depthExtendImage[np.where(self.DepthIRFlag == 3)] = [255,255,255]#depthの大きさから索道と予想される部分
        maxX = self.x
        maxY = self.y
        maxAngle1 = 270 + self.Rangle
        maxAngle2 = 270 - self.Langle
        maxAngle3 = 90
        thickness = 15
        h = self.h
        w = self.w

        cablewayImage = DrawAngleLine(np.zeros((h,w,3),dtype=np.uint8),maxX,maxY,maxAngle1,(255,100,100,50),thickness)
        cablewayImage = DrawAngleLine(cablewayImage,maxX,maxY,maxAngle2,(100,255,100,50),thickness)
        cablewayImage = DrawAngleLine(cablewayImage,maxX,maxY,maxAngle3,(100,100,255,50),thickness)
        cablewayImage = cv2.putText(cablewayImage,str(int(self.Rangle)) + "/" + str(int(self.Langle)),(0,160),cv2.FONT_HERSHEY_PLAIN,2,(255,255,255))
        rr,cc = disk((maxY,maxX), 24, shape=(h,w))
        mask = np.logical_and.reduce((rr >= 0, rr < h ,cc >= 0, cc < w))
        cablewayImage[rr[mask],cc[mask]] = [255,255,0]
        brendImage = cv2.addWeighted(depthExtendImage, 0.5, cablewayImage, 0.5, 0,dtype=cv2.CV_32F)

        return brendImage