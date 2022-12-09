from SSC_ImageRecognition14 import IR
import cv2
from Utilty import cvpaste,CreteViewImage,ScalarImage2RGB,reg1dim,getImageFromFile,rounddown,disk,DrawAngleLine,getLikeAngle,DebugImage
import numpy as np
from PIL import Image,ImageTk
import math
from OutputController import OutputController
import numpy as np
from matplotlib import pyplot as plt
import tkinter as tk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.colors



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
        self.LAngleLog = [0 for a in range(self.LogNumber)]
        self.RAngleLog = [0 for a in range(self.LogNumber)]
        self.InclinationAngleLog = [0 for a in range(self.LogNumber)]
        self.tAngleLog = [0 for a in range(self.LogNumber)]
        #self.YLog = [0 for a in range(self.LogNumber)]


        #索道などのパラメータ
        self.minDistance = 0
        self.maxDistance = 500
        self.overDistance = 2000

        self.w = 0
        self.h = 0

    def setImage(self,color_image,depth_image,ir_image1,ir_image2,web_image):
        self.color_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2RGB)
        self.ir_image1 = ir_image1
        self.ir_image2 = ir_image2
        self.depth_image = ScalarImage2RGB(depth_image,self.minDistance,self.overDistance)
        self.depth_scale = depth_image
        self.web_image = web_image
        try:
            self.adjustView()
            self.ir_scale = cv2.cvtColor(self.ir_image1.copy(),cv2.COLOR_RGB2GRAY)
        except Exception as e:
            print("Error",e)
            pass
        

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
        try:
            self.ir_image1 = cvpaste(self.ir_image1, np.zeros((h,w,3)), 0, 0, angle, scale)#若干拡大する
            self.ir_image1 = np.delete(self.ir_image1,slice(w - maxX,w),1)
        except Exception as e:
            print("Error",e)
        


    def calcCablewayInf(self,accel,extMode):
        self.Error = False
        self.IsBranch = False
        if(np.sum(self.ir_image1) < 0.1 or np.sum(self.color_image) < 0.1):#カメラがほぼ真っ暗
            self.Error = True
            OutputController().msgPrint("Camera is black")
            return 0,0
        result = IR(self.color_image,self.depth_image,self.ir_image1,self.depth_scale,self.ir_scale,accel,self.minDistance,self.maxDistance,self.overDistance,extMode)
        if(len(result) == 1):#不正ならFalseだけが返ってくる
            self.Error = True
            OutputController().msgPrint("Invalid Camera error")
            return 0,0
        (self.y,self.x,self.branchValue,self.Rangle,self.Langle,self.DepthIRFlag) = result
        self.tangle = math.degrees(-math.atan2(accel.y,accel.z))
        self.appendLog()


        (rule1,rule2) = self.calcRule()
        if(rule1 > 1 and rule2 > 1):
            self.IsBranch = True
        return rule1,rule2

    def get3DImages(self):
        # データを用意する
        x = np.arange(-10, 10, 0.1) # x軸を作成
        y = np.arange(-10, 10, 0.1) # y軸を作成
        X, Y = np.meshgrid(x, y)    # グリッドデータの作成
        Z = np.sqrt((np.square(X) + np.square(Y))) #Z軸を作成

        # フォントの種類とサイズを設定する。
        plt.rcParams['font.size'] = 15
        plt.rcParams['font.family'] = 'Arial'

        # グラフの入れ物を用意する。
        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection='3d')  

        # 軸のラベルを設定する。
        ax1.set_xlabel('x', labelpad=10)
        ax1.set_ylabel('y', labelpad=10)
        ax1.set_zlabel('z', labelpad=10)

        # データプロットする。
        ax = ax1.plot_surface(X, Y, Z, cmap='jet',label="z")

        #カラーバーの設定
        cbar = fig.colorbar(ax, shrink = 0.6)
        cbar.set_label("Z", fontsize=15)

        return fig

    def getOutputImage(self):
        if(self.Error):return  ImageTk.PhotoImage(getImageFromFile("test.png"))
        depth_view_image = ScalarImage2RGB(self.depth_scale,self.minDistance,self.overDistance)
        InclinationImage = self.getInclinationImage()
        cablewayImage = self.getCablewayImage()
        imageMap = np.zeros(InclinationImage.shape)
        depthIRImage = cv2.resize(self.getDepthIRMap(depth_view_image),(307,230))
        self.web_image = cv2.resize(self.web_image,(307,230))

        image = CreteViewImage(self.color_image,depth_view_image,self.ir_image1,cablewayImage,depthIRImage,self.web_image)
        return ImageTk.PhotoImage(Image.fromarray(image.astype(np.uint8)))

    def appendLog(self):
        self.valueLog.append(self.branchValue)
        self.valueLog.pop(0)
        self.YLog.append(self.y)
        self.YLog.pop(0)
        self.LAngleLog.append(self.Langle)
        self.LAngleLog.pop(0)
        self.RAngleLog.append(self.Rangle)
        self.RAngleLog.pop(0)
        self.InclinationAngleLog.append(self.InclinationAngle)
        self.InclinationAngleLog.pop(0)
        self.tAngleLog.append(self.tangle)
        self.tAngleLog.pop(0)

    def getAverageData(self):
        hoge = np.where(np.array(self.valueLog) > 300)
        if(len(hoge) > 1):
            InclinationAngle = np.average(np.array((self.InclinationAngleLog)[hoge]))
            ElevationAngle = np.average(np.array((self.ElevationAngleLog)[hoge]))
            RAngle = np.average(np.array((self.RAngleLog)[hoge]))
            LAngle = np.average(np.array((self.LAngleLog)[hoge]))
            tAngle = np.average(np.array((self.tAngleLog)[hoge]))

            return (InclinationAngle,ElevationAngle,Rangle,Langle,tangle)
        else:     return (0,0,0,0,0)

    def calcRule(self):
        #1.直近N1ログ以内のvalueLog平均値が300以上
        #2.直近N1ログ以内のvalueLog平均値が300以上の結果から推定された5ログあとのYLogの線形近似解が高さhの1.2倍を超えた
        N1 = 15
        valueLogList = np.array(self.valueLog[len(self.valueLog) - N1:])
        rule1 = np.average(valueLogList) / 300

        hoge2 = np.array(self.YLog[len(self.YLog) - N1:])
        l1 = np.array([n for n in range(N1)])[np.nonzero(valueLogList > 300)]
        l2 = hoge2[np.nonzero(valueLogList > 300)]
    
        if(len(l1) < 2):rule2 = 0
        else:
            a,b = reg1dim(l1,l2) 
            rule2 = (l2[len(l2) - 1] + a * 5) / (self.h * 1.2)
            OutputController().msgPrint("a,b,l2max",a,b,l2[len(l2) - 1])
            #valueLogList:",valueLogList[len(valueLogList) - 1],rule1,rule2)
            #OutputController().msgPrint("calc rule a,b:",a,b)
            #OutputController().msgPrint("calc rule YLog:",self.YLog)
            #OutputController().msgPrint("calc rule l1:",l1)
            #OutputController().msgPrint("calc rule l2:",l2)
        return (rule1,rule2)


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

    def getDepthIRMap(self,depth_view_image):
        #DepthIRMap = self.depth_image.copy()
        #DepthIRMap =depth_view_image.copy()
        DepthIRMap = np.stack((self.depth_scale, self.depth_scale, self.depth_scale), axis=2)
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
        maxAngle1 = 270 - self.Rangle
        maxAngle2 = 270 + self.Langle
        maxAngle3 = 90
        thickness = 15
        h = self.h
        w = self.w

        cablewayImage = DrawAngleLine(np.zeros((h,w,3),dtype=np.uint8),maxX,maxY,maxAngle1,(255,100,100,50),thickness)
        cablewayImage = DrawAngleLine(cablewayImage,maxX,maxY,maxAngle2,(100,255,100,50),thickness)
        cablewayImage = DrawAngleLine(cablewayImage,maxX,maxY,maxAngle3,(100,100,255,50),thickness)
        #cablewayImage = cv2.putText(cablewayImage,str(int(self.Rangle)) + "/"
        #+
        #str(int(self.Langle)),(0,160),cv2.FONT_HERSHEY_PLAIN,2,(255,255,255))
        rr,cc = disk((maxY,maxX), 24, shape=(h,w))
        mask = np.logical_and.reduce((rr >= 0, rr < h ,cc >= 0, cc < w))
        cablewayImage[rr[mask],cc[mask]] = [255,255,0]
        brendImage = cv2.addWeighted(depthExtendImage, 0.5, cablewayImage, 0.5, 0,dtype=cv2.CV_32F)

        return brendImage