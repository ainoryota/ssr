#GUIで用いるプログラム

import importlib
import serial
import time
import Control
import numpy as np
import math
import csv
import tkinter as tk

#円周率
pi = math.pi

importlib.reload(Control) #モジュールの再読み込み

def Branch(ser,csvfile,value,save):
    #分岐データの読み込み
    csvfile="Data/"+csvfile;
    data = np.loadtxt(csvfile,delimiter=",")
    (n,m)=data.shape
    log = [[0]*21for i in range(n)]#初期化
    time_start = time.perf_counter()
    
    #前進後退でモータに流す入力を変えるため
    if value == True:
        id = [0,1,2,3,4,5,6,7,8,9,10]
        direct = 1
    else:
        id = [0,6,7,8,9,10,1,2,3,4,5]
        direct = -1

    for t in range(0,n-1):#行数-1
        if(t%4==1):continue;#早めに後輪を分岐する
        log[t][0] = time.perf_counter() - time_start
        Control.Motor_pos6(ser,id[1],id[2],id[3],id[6],id[7],id[8],round(data[t,0]/pi*180,2),round(data[t,1]/pi*180,2),round(data[t,2]/pi*180,2),round(data[t,3]/pi*180,2),round(data[t,4]/pi*180,2),round(data[t,5]/pi*180,2))    
        Control.Motor_vel2(ser,id[4],id[5],round(-direct*data[t,6]),round(direct*data[t,6]))
        Control.Motor_vel2(ser,id[9],id[10],round(-direct*data[t,7]),round(direct*data[t,7]))
        
        log[t][1] = data[t,0]/pi*180;log[t][2] = data[t,1]/pi*180;log[t][3] = data[t,2]/pi*180;log[t][4] = data[t,3]/pi*180;log[t][5] = data[t,4]/pi*180;log[t][6] = data[t,5]/pi*180
        log[t][7] = 0;log[t][8] = 0;log[t][9] = 0;log[t][10] = 0;

        ser.flushInput()#バッファのクリア
        Control.Position_Read2(ser,1)#バッファに返信データを順に貯める
        Control.Position_Read2(ser,2)
        Control.Position_Read2(ser,3)
        Control.Velocity_Read2(ser,4)
        Control.Velocity_Read2(ser,5)
        Control.Position_Read2(ser,6)
        Control.Position_Read2(ser,7)
        Control.Position_Read2(ser,8)
        Control.Velocity_Read2(ser,9)
        a = Control.Velocity_Read3(ser,10)#バッファから返信データを引き出す
        
        log[t][11] = Control.Position_Read4(a,1)#返信データを角度データに変換する
        log[t][12] = Control.Position_Read4(a,2)
        log[t][13] = Control.Position_Read4(a,3)
        log[t][14] = Control.Position_Read4(a,6)
        log[t][15] = Control.Position_Read4(a,7)
        log[t][16] = Control.Position_Read4(a,8)
        log[t][17] = Control.Position_Read4(a,4)
        log[t][18] = Control.Position_Read4(a,5)
        log[t][19] = Control.Position_Read4(a,9)
        log[t][20] = Control.Position_Read4(a,10)
  
    if save == True:
        #ファイルに書き出し
        #print("abc")
        with open('log_'+csvfile,'w') as f:
            writer = csv.writer(f,lineterminator='\n')
            writer.writerows(log)


    
def Move(id,data,direct,log):


    return log;
