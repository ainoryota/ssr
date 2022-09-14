from SSC_ImageRecognition14 import IR
import cv2
from Utilty import cvpaste
import numpy as np
import Image

class BranchSystem:
    def __init__(self):
        self.IsBranch=False
        self.color_image=np.zeros((1,1,3))
        self.ir_image1=np.zeros((1,1,3))
        self.ir_image2=np.zeros((1,1,3))
        self.depth_image=np.zeros((1,1,3))

        #索道などのパラメータ
        self.minDistance = 200
        self.maxDistance = 500
        self.overDistance = 2000

    def setImage(self,color_image,depth_image,ir_image1,ir_image2):
        self.color_image=color_image
        self.ir_image1=depth_image
        self.ir_image2=ir_image1
        self.depth_image=ir_image2


    def setCablewayInf(self,color_image,depth_image,ir_image1,ir_image2,accel,extMode):
        self.IsBranch=False
        if(np.sum(ir_image1)<0.1):#赤外線カメラが真っ暗
            return Image.open('test.png')
        self.setImage(color_image,depth_image,ir_image1,ir_image2)
        self.adjustView()
        print(self.color_image.shape,self.ir_image1.shape,self.depth_image.shape)

        result=IR(self.color_image,self.depth_image,self.ir_image1,accel,self.minDistance,self.maxDistance,self.overDistance,extMode)
        if(len(result)>1):#不正ならFalseだけが返る
            y = result[1]
            x = result[2]
            rule1 = result[3]
            rule3 = result[4]
            LElevationAngle = result[5]
            RElevationAngle = result[6]
            LRE = result[7]#仰角
            Rangle = result[8]
            Langle = result[9]

            if(rule1 > 1 and rule3 > 1):
                self.IsBranch=True
                tangle=math.degrees(-math.atan2(accel.y,accel.z))
                (tangle,LRE,angleA,angleB) = getLikeAngle(tangle,LRE,Rangle,Langle)
        return result[0]

    def adjustView(self):
        #画像の標準サイズ
        w = 320
        h = 180

        #赤外線カメラとdepthカメラの位置変換用定数
        maxX = 13
        angle = 0
        scale = 1.4

        #リサイズとファイル形式の変換
        self.color_image = cv2.resize(self.color_image, (w,h))
        self.depth_image = cv2.resize(self.depth_image, (w,h))
        self.ir_image1 = cv2.resize(self.ir_image1, (w,h))
        self.ir_image1 = cv2.cvtColor(self.ir_image1,cv2.COLOR_GRAY2RGB)
        self.ir_image2= cv2.resize(self.ir_image2, (w,h))
        self.ir_image2 = cv2.cvtColor(self.ir_image2,cv2.COLOR_GRAY2RGB)

        #画角合わせ（赤外線カメラ2を除く）
        self.color_image = np.delete(self.color_image,slice(0,maxX),1)
        self.depth_image = np.delete(self.depth_image,slice(0,maxX),1)
        self.ir_image1 = cvpaste(self.ir_image1, np.zeros((h,w,3)), 0, 0, angle, scale)#若干拡大する
        self.ir_image1 = np.delete(self.ir_image1,slice(w - maxX,w),1)

    def getBranch(self):#分岐すべきならそのタイミングや角度を返す
        SleepTime=0
        InclinationAngle=0
        ElevationAngle=0
        RTurningAngle=0
        LTurningAngle=0
        return [self.IsBranch,SleepTime,InclinationAngle,ElevationAngle,RTurningAngle,LTurningAngle]

    def ResetLog(self):
        return 0
    
