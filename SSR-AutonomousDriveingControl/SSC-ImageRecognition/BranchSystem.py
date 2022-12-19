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
        self.ProceedAngle = 0
        self.branchValue = 0

        self.LogNumber = 100
        self.valueLog = [0 for a in range(self.LogNumber)]
        self.YLog = [0 for a in range(self.LogNumber)]
        self.LAngleLog = [0 for a in range(self.LogNumber)]
        self.RAngleLog = [0 for a in range(self.LogNumber)]
        self.InclinationAngleLog = [0 for a in range(self.LogNumber)]
        self.tAngleLog = [0 for a in range(self.LogNumber)]
        self.Value1Log = [0 for a in range(self.LogNumber)]
        self.Value2Log = [0 for a in range(self.LogNumber)]
        self.Value3Log = [0 for a in range(self.LogNumber)]
        #self.YLog = [0 for a in range(self.LogNumber)]

        self.maxValue1 = 0
        self.maxValue2 = 0
        self.maxValue3 = 0

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
        


    def calcCablewayInf(self,accel,timerLog,extMode):
        self.Error = False
        if(np.sum(self.ir_image1) < 0.1 or np.sum(self.color_image) < 0.1):#カメラがほぼ真っ暗
            self.Error = True
            OutputController().msgPrint("Camera is black")
            return 0,0,0,0,0,0,0,0
        result = IR(self.color_image,self.depth_image,self.ir_image1,self.depth_scale,self.ir_scale,accel,self.minDistance,self.maxDistance,self.overDistance,extMode)
        if(len(result) == 1):#不正ならFalseだけが返ってくる
            self.Error = True
            OutputController().msgPrint("Invalid Camera error")
            return 0,0,0,0,0,0,0,0
        (self.y,self.x,self.branchValue,self.Rangle,self.Langle,self.ProceedAngle,self.DepthIRFlag,self.maxValue1,self.maxValue2,self.maxValue3) = result
        self.tangle = math.degrees(-math.atan2(accel.y,accel.z))
        self.appendLog()
        

        (rule0,rule1,rule2,rule5,rule6,rule7,a) = self.calcRule(timerLog)
        return self.branchValue,rule0,rule1,rule2,rule5,rule6,rule7,a

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
        self.Value1Log.append(self.maxValue1)
        self.Value1Log.pop(0)
        self.Value2Log.append(self.maxValue2)
        self.Value2Log.pop(0)
        self.Value3Log.append(self.maxValue3)
        self.Value3Log.pop(0)



    def calcRule(self,timerLog):
        #A→屋外実験により720以上必要であると確認。屋内実験で880以下でなければならないと確認済み
        #1.直近N1ログ以内のvalueLog平均値がA以上
        #2.直近N1ログ以内のvalueLog平均値がA以上の結果から推定された5ログあとのYLogの線形近似解が高さhの1.2倍を超えた
        N1 = 12 #処理時間が500msなら8、300msなら12

        #YLogList = np.array(self.YLog[len(self.YLog) - N1:])
        #rule0 = np.average(YLogList)
        start_time = timerLog[len(timerLog) - N1]

        valueLogList = np.array(self.valueLog[len(self.valueLog) - N1:])
        timerLogList = np.array(timerLog[len(timerLog) - N1:])
        Value1LogList = np.array(self.Value1Log[len(self.Value1Log) - N1:])
        Value2LogList = np.array(self.Value2Log[len(self.Value2Log) - N1:])
        Value3LogList = np.array(self.Value3Log[len(self.Value3Log) - N1:])
        
        
        
        
        
        rule1 = np.average(valueLogList) / 750

        hoge2 = np.array(self.YLog[len(self.YLog) - N1:])
        print("Ylog",hoge2)

        one_time,_ = reg1dim(np.array([n for n in range(len(timerLogList))]),timerLogList)
        

        l1 = timerLogList[np.nonzero(valueLogList > 750)]
        l2 = hoge2[np.nonzero(valueLogList > 750)]

        l3 = Value1LogList[np.nonzero(valueLogList > 750)]
        l4 = Value2LogList[np.nonzero(valueLogList > 750)]
        l5 = Value3LogList[np.nonzero(valueLogList > 750)]


        if(len(l2) > 0):
            foo = np.where(l2 < np.max(l2))
        
            print(np.max(l2))
            l1 = l1[foo]
            l2 = l2[foo]
            l3 = l3[foo]
            l4 = l4[foo]
            l5 = l5[foo]

        if(len(l1) < N1 / 2):
            if(len(l1) > 0):
                a,b = reg1dim(l1,l2)   
                rule2 = (b + a * (start_time + one_time * (5 + N1))) / (self.h * 1.2)
            else:
                a = 0
                b = 0
                rule2 = 0

            OutputController().msgPrint("▲:","len=",len(l1),a,b,rule2)
            rule2 = 0
            rule0 = 0
            rule5 = 0
            rule6 = 0
            rule7 = 0
        else:
            #OutputController().printmsg(len(l1) < 2)

            rule0 = np.corrcoef(l1,l2)[0,1]
            rule5 = np.corrcoef(l1,l3)[0,1]
            rule6 = np.corrcoef(l1,l4)[0,1]
            rule7 = -np.corrcoef(l1,l5)[0,1]

            a,b = reg1dim(l1,l2)
            #a,b = L_abs_minimize(l1,l2)
            
            
            rule2 = (b + a * (start_time + one_time * (5 + N1))) / (self.h * 1.2)

            OutputController().msgPrint("l2:","a=",a,"b=",b,"one_time=",one_time,"place=",(b + a * (one_time * (5 + N1))),"l2=",l2)

            #OutputController().msgPrint("a,b,l2max",a,b,l2[len(l2) - 1])
            #valueLogList:",valueLogList[len(valueLogList) - 1],rule1,rule2)
            #OutputController().msgPrint("calc rule a,b:",a,b)
            #OutputController().msgPrint("calc rule YLog:",self.YLog)
            #OutputController().msgPrint("calc rule l1:",l1)
            #OutputController().msgPrint("calc rule l2:",l2)
        return (rule0,rule1,rule2,rule5,rule6,rule7,a)


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
        maxAngle3 = self.ProceedAngle
        thickness = 10
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