import tkinter as tk
from functools import partial

from Robot import Robot
from Order import PosOrder
from Order import VelocityOrder
from Order import MotorModeOrder
from Order import ResetMotorOrder
from Order import ResetEncoderOrder
from OutputController import OutputController
from OutputController import MotorMode
from RealSense import RealSense
import numpy as np
import math
import time
import os



class Branch(object):
    """description of class"""
    def __init__(self,robot):
        self.robot = robot
        OutputController().msgPrint("Branch Start")

    def FileBranch(self,csvfile_original,runDirection,save,a=24):
        #分岐データの読み込み
        csvfile = "C:/Users/MSD/Documents/GitHub/Data_stop/" + csvfile_original
        if(not(os.path.exists(csvfile))):
            csvfile = "C:/Users/MSD/Documents/GitHub/Data_nonstop/" + csvfile_original

        OutputController().msgPrint("Branch",csvfile,"a=",round(a,2))
        
        
        data = np.loadtxt(csvfile,delimiter=",")
        (n,m) = data.shape
        log = [[0] * 21 for i in range(n)]#初期化
        time_start = time.perf_counter()
    
        #前進後退でモータに流す入力を変えるため
        if runDirection == True:
            id = [0,1,2,3,4,5,6,7,8,9,10]
            direct = 1
        else:
            id = [0,6,7,8,9,10,1,2,3,4,5]
            direct = -1

        t = 0

        direct*=1;
        for line in range(0,n):#行数-1
            #if(t%4==1):continue;#早めに後輪を分岐する
                                               #log[t][0] = time.perf_counter()
                                                                                  #-
                                                                                                                     #time_start

            #self.robot.motors[id[1]].insertOrder(PosOrder(round((data[line-1,0]+data[line,0]+data[line+1,0])/3 / math.pi * 180,2),t))#前のアームの根本 時計回りが正
            #self.robot.motors[id[2]].insertOrder(PosOrder(round((data[line-1,1]+data[line,1]+data[line+1,1])/3 / math.pi * 180,2),t))#前のアームの2番目
            #self.robot.motors[id[3]].insertOrder(PosOrder(round((data[line-1,2]+data[line,2]+data[line+1,2])/3 / math.pi * 180,2),t))#前のアームの3番目
            #self.robot.motors[id[6]].insertOrder(PosOrder(round((data[line-1,3]+data[line,3]+data[line+1,3])/3 / math.pi * 180,2),t))#後ろのアームの根本
            #self.robot.motors[id[7]].insertOrder(PosOrder(round((data[line-1,4]+data[line,4]+data[line+1,4])/3 / math.pi * 180,2),t))#後ろのアームの2番目
            #self.robot.motors[id[8]].insertOrder(PosOrder(round((data[line-1,5]+data[line,5]+data[line+1,5])/3 / math.pi * 180,2),t))#後ろのアームの3番目

            self.robot.motors[id[1]].insertOrder(PosOrder(round(data[line,0] / math.pi * 180,2),t))#前のアームの根本 時計回りが正
            self.robot.motors[id[2]].insertOrder(PosOrder(round(data[line,1] / math.pi * 180,2),t))#前のアームの2番目
            self.robot.motors[id[3]].insertOrder(PosOrder(round(data[line,2] / math.pi * 180,2),t))#前のアームの3番目
            self.robot.motors[id[6]].insertOrder(PosOrder(round(data[line,3] / math.pi * 180,2),t))#後ろのアームの根本
            self.robot.motors[id[7]].insertOrder(PosOrder(round(data[line,4] / math.pi * 180,2),t))#後ろのアームの2番目
            self.robot.motors[id[8]].insertOrder(PosOrder(round(data[line,5] / math.pi * 180,2),t))#後ろのアームの3番目

            self.robot.motors[id[4]].insertOrder(VelocityOrder(round(-direct * data[line,6]),t))
            self.robot.motors[id[5]].insertOrder(VelocityOrder(round(direct * data[line,6]),t))
            self.robot.motors[id[9]].insertOrder(VelocityOrder(round(-direct * data[line,7]),t))
            self.robot.motors[id[10]].insertOrder(VelocityOrder(round(direct * data[line,7]),t))
            
            #self.robot.motors[id[4]].insertOrder(VelocityOrder(round(-direct * #2.12206591 * 100),t))
            #self.robot.motors[id[5]].insertOrder(VelocityOrder(round(direct *
            #2.12206591 * 100),t))
            #self.robot.motors[id[9]].insertOrder(VelocityOrder(round(-direct *
            #2.12206591 * 100),t))
            #self.robot.motors[id[10]].insertOrder(VelocityOrder(round(direct *
            #2.12206591 * 100),t))
            if(line == n // 2):t+=1.4#0.7だめ
            #t+=0.024
            t+=0.036
            #t+=(0.024 * 100) / a #aが大きいほど小さな値にする。


        OutputController().pushStep()


        #log[t][1] = data[t,0]/pi*180;log[t][2] = data[t,1]/pi*180;log[t][3] =
        #data[t,2]/pi*180;log[t][4] = data[t,3]/pi*180;log[t][5] =
        #data[t,4]/pi*180;log[t][6] = data[t,5]/pi*180
        #log[t][7] = 0;log[t][8] = 0;log[t][9] = 0;log[t][10] = 0;
        #
        #ser.flushInput()#バッファのクリア
        #Control.Position_Read2(ser,1)#バッファに返信データを順に貯める
        #Control.Position_Read2(ser,2)
        #Control.Position_Read2(ser,3)
        #Control.Velocity_Read2(ser,4)
        #Control.Velocity_Read2(ser,5)
        #Control.Position_Read2(ser,6)
        #Control.Position_Read2(ser,7)
        #Control.Position_Read2(ser,8)
        #Control.Velocity_Read2(ser,9)
        #a = Control.Velocity_Read3(ser,10)#バッファから返信データを引き出す
        #
        #log[t][11] = Control.Position_Read4(a,1)#返信データを角度データに変換する
        #log[t][12] = Control.Position_Read4(a,2)
        #log[t][13] = Control.Position_Read4(a,3)
        #log[t][14] = Control.Position_Read4(a,6)
        #log[t][15] = Control.Position_Read4(a,7)
        #log[t][16] = Control.Position_Read4(a,8)
        #log[t][17] = Control.Position_Read4(a,4)
        #log[t][18] = Control.Position_Read4(a,5)
        #log[t][19] = Control.Position_Read4(a,9)
        #log[t][20] = Control.Position_Read4(a,10)
  
        #if save == True:
        #    #ファイルに書き出し
        #    #OutputController().msgPrint("abc")
        #    with open('log_'+csvfile,'w') as f:
        #        writer = csv.writer(f,lineterminator='\n')
        #        writer.writerows(log)

    def branchAngle(self,a,GammalAngle,TurnAngle,Langle,Rangle,runDirection,turnDirection,tension,save):
        if turnDirection == True:         #右分岐かどうか
            if runDirection == True:       #前進中かどうか
                mode = 1
            else:
                mode = 2
        else:
            if runDirection == True:       #前進中かどうか
                mode = 2
            else:
                mode = 1
        #while Langle + Rangle < 100:
        #    if Langle < 80:Langle = Langle + 5
        #    if Rangle < 80:Rangle = Rangle + 5
    
        value = str(GammalAngle) + "_" + str(TurnAngle) + "_" + str(Langle) + "_" + str(Rangle)
        csv_name = value + '_' + str(mode) + '.csv'    
        if tension == True:
            csv_name = value + '_' + str(mode) + '_T' + '.csv'
        
        OutputController().msgPrint(csv_name,"AngleMode")
        self.FileBranch(csv_name,runDirection,save,a)




        #    def branchAngle(self,GammalAngle,TurnAngle,Langle,Rangle,runDirection,turnDirection):
        #if self.v3.get() == True:         #右分岐かどうか
        #    if self.v1.get() == True:       #前進中かどうか
        #        mode = 1
        #    else:
        #        mode = 2
        #else:
        #    if self.v1.get() == True:       #前進中かどうか
        #        mode = 2
        #    else:
        #        mode = 1
        #while Langle + Rangle < 100:
        #    if Langle < 80:Langle = Langle + 5
        #    if Rangle < 80:Rangle = Rangle + 5
    
        #value = str(GammalAngle) + "_" + str(TurnAngle) + "_" + str(Langle) + "_" + str(Rangle)
        #csv_name = value + '_' + str(mode) + '.csv'    
        #if self.v7.get() == True:
        #    csv_name = value + '_' + str(mode) + '_T' + '.csv'
        #    self.v5.set(reverse(self.v5.get()))
        #    self.v6.set(reverse(self.v6.get()))
        
        #OutputController().msgPrint(csv_name,"AngleMode")
        #self.Branch(csv_name,self.v1.get(),self.v8.get())