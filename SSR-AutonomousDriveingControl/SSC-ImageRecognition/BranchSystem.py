from SSC_ImageRecognition14 import IR
import cv2
from Utilty import cvpaste,CreteViewImage,ScalarImage2RGB
import numpy as np
from PIL import Image,ImageTk


class BranchSystem:
    def __init__(self):
        self.IsBranch=False
        self.color_image=np.zeros((1,1,3))#RGB
        self.ir_image1=np.zeros((1,1,3))#RGB
        self.ir_image2=np.zeros((1,1,3))#RGB
        self.ir_scale=np.zeros((1,1))
        self.depth_image=np.zeros((1,1,3))#RGB
        self.depth_scale=np.zeros((1,1))
        self.EffectiveDepthScale=np.zeros(307)#x軸方向に探査したときの有効な深度情報

        
        self.Error=False
        self.y=0
        self.x=0
        self.InclinationAngle=0
        self.Rangle=0
        self.Langle=0


        #索道などのパラメータ
        self.minDistance = 200
        self.maxDistance = 500
        self.overDistance = 2000

        self.w=0
        self.h=0

    def setImage(self,color_image,depth_image,ir_image1,ir_image2):
        self.color_image=color_image
        self.ir_image1=ir_image1
        self.ir_image2=ir_image2
        self.depth_image=depth_image#後にRGB画像になる
        self.adjustView()
        self.depth_scale=self.depth_image.copy()
        self.ir_scale=ir_image1.copy()

    def adjustView(self):
        #画像の標準サイズ
        w = 320
        h = 180

        #赤外線カメラとdepthカメラの位置変換用定数
        maxX = 13
        angle = 0
        scale = 1.4
        self.w=w-maxX
        self.h=h

        #リサイズとファイル形式の変換
        self.color_image = cv2.resize(self.color_image, (w,h))
        self.depth_image = cv2.resize(self.depth_image, (w,h))
        self.ir_image1 = cv2.resize(self.ir_image1, (w,h))
        self.ir_image1 = ScalarImage2RGB(self.ir_image1,0,255)
        self.ir_image2= cv2.resize(self.ir_image2, (w,h))
        self.ir_image2 = ScalarImage2RGB(self.ir_image2,0,255)

        #画角合わせ（赤外線カメラ2を除く）
        self.color_image = np.delete(self.color_image,slice(0,maxX),1)
        self.depth_image = np.delete(self.depth_image,slice(0,maxX),1)
        self.ir_image1 = cvpaste(self.ir_image1, np.zeros((h,w,3)), 0, 0, angle, scale)#若干拡大する
        self.ir_image1 = np.delete(self.ir_image1,slice(w - maxX,w),1)

    def calcCablewayInf(self,accel,extMode):
        self.Error=False
        self.IsBranch=False
        if(np.sum(self.ir_image1)<0.1):#赤外線カメラが真っ暗
            self.Error=True
            return
        result=IR(self.color_image,self.depth_image,self.ir_image1,self.ir_scale,accel,self.minDistance,self.maxDistance,self.overDistance,extMode)
        if(len(result)==1):#不正ならFalseだけが返ってくる
            self.Error=True
            return
        (self.y,self.x,self.InclinationAngle,self.Rangle,self.Langle,self.EffectiveDepthScale)=result;

        rule1=0
        if(rule1 > 1 and rule3 > 1):
            self.IsBranch=True
            tangle=math.degrees(-math.atan2(accel.y,accel.z))
            (tangle,LRE,angleA,angleB) = getLikeAngle(tangle,LRE,Rangle,Langle)

    def getOutputImage(self):
        if(self.Error):return  ImageTk.PhotoImage(Image.fromarray(Image.open('./test.png')))
        
        depth_view_image = ScalarImage2RGB(self.depth_image,300,2000)
        InclinationImage=self.getInclinationImage()
        brendImage=np.zeros(self.ir_image1.shape);
        imageMap=np.zeros(InclinationImage.shape);
        image=CreteViewImage(self.color_image,depth_view_image,self.ir_image1,brendImage,cvpaste(imageMap, np.zeros(InclinationImage.shape), 0, 0, 0,1),InclinationImage)
        return ImageTk.PhotoImage(Image.fromarray(image.astype(np.uint8)))

    def getBranch(self):#分岐すべきならそのタイミングや角度を返す
        SleepTime=0
        InclinationAngle=0
        ElevationAngle=0
        RTurningAngle=0
        LTurningAngle=0
        return [self.IsBranch,SleepTime,InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle]

    def ResetLog(self):
        return 0
    
    def getInclinationImage(self):
        InclinationImage = np.zeros((int(self.maxDistance - self.minDistance),self.w,3),dtype=np.uint8) + 100
        idx = np.nonzero(self.EffectiveDepthScale > 0)
        if(len(idx)>1):
            InclinationImage[self.maxDistance - self.EffectiveDepthScale[idx] - 1,idx[1]] = [255,255,255]
        return InclinationImage