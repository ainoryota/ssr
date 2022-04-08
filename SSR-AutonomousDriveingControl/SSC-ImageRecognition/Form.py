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
import numpy as np
import math
import time

class PushButton:
    button = []
    def __init__(self,tk,win,row,column,k,com,num=-1):
        if(num == -1):button = tk.Button(win,text = k,command = com)
        else:button = tk.Button(win,text = k,command = partial(com, num))
        button.grid(row = row, column =column, padx = 5, pady = 5)
class Label:
    label = []
    def __init__(self,tk,win,row,column,k):
        label = tk.Label(win,text = k)
        label.grid(row = row, column =column, padx = 5, pady = 5)      

class Form(object):
    def __init__(self,robot):
        self.robot=robot

        #ウィンドウの生成
        self.root = tk.Tk()  
        self.root.title('GUIコントローラ')
        self.root.geometry('1000x720+150+150')

        #ラベルの生成
        Label(tk,self.root,0,0,'斜度_回転角_右旋回角_左旋回角')
        Label(tk,self.root,0,3,'     ')#空白ラベル
        Label(tk,self.root,0,9,'ロボットの状態') 

        #ボタンの作成
        PushButton(tk,self.root,0,4,' 前進 ',self.forward)
        PushButton(tk,self.root,0,5,' 後退',self.back)
        PushButton(tk,self.root,0,6,' 停止 ',self.stop)
        PushButton(tk,self.root,0,7,' フリー ',self.free)
        PushButton(tk,self.root,0,8,' 左右切り替え',self.change)
        PushButton(tk,self.root,0,9,' 前輪切り替え',self.change_f)
        PushButton(tk,self.root,0,10,' 後輪切り替え',self.change_r)
        
        PushButton(tk,self.root,1,8,' 巻取り ',self.wintchWind)
        PushButton(tk,self.root,1,9,' 繰り出し',self.wintchFeed)
        PushButton(tk,self.root,1,10,' 停止  ',self.wintchStop)
        
        PushButton(tk,self.root,2,8,' 初期化 ',self.init)

        #チェックボックスの作成
        self.v1 = tk.BooleanVar()
        self.v2 = tk.BooleanVar()
        self.v3 = tk.BooleanVar()
        self.v4 = tk.BooleanVar()
        self.v5 = tk.BooleanVar()
        self.v6 = tk.BooleanVar()
        self.v7 = tk.BooleanVar()
        self.v8 = tk.BooleanVar()
        self.v_auto = tk.BooleanVar()

        self.v1.set(True) 
        self.v2.set(False) 
        self.v3.set(True) 
        self.v4.set(False)
        self.v5.set(True) 
        self.v6.set(True)
        self.v7.set(True)
        self.v8.set(False)
        self.v_auto.set(False)

        self.cb1 = tk.Checkbutton(self.root,text='前進　',variable = self.v1)
        self.cb2 = tk.Checkbutton(self.root,text='後退　',variable = self.v2)
        self.cb3 = tk.Checkbutton(self.root,text='右分岐',variable = self.v3)
        self.cb4 = tk.Checkbutton(self.root,text='左分岐',variable = self.v4)
        self.cb_auto = tk.Checkbutton(self.root,text='auto分岐',variable = self.v_auto)
        self.cb5 = tk.Checkbutton(self.root,text='前輪T　',variable = self.v5)
        self.cb6 = tk.Checkbutton(self.root,text='後輪T　',variable = self.v6)
        self.cb7 = tk.Checkbutton(self.root,text='テンション分岐モード',variable = self.v7)
        self.cb8 = tk.Checkbutton(self.root,text='セーブモード',variable = self.v8)

        self.cb1.grid(row = 1, column =4, padx = 5, pady = 5)
        self.cb2.grid(row = 1, column =5, padx = 5, pady = 5)
        self.cb3.grid(row = 1, column =6, padx = 5, pady = 5)
        self.cb4.grid(row = 1, column =7, padx = 5, pady = 5)
        self.cb_auto.grid(row = 13, column =1, padx = 5, pady = 5)
        self.cb5.grid(row = 2, column =4, padx = 5, pady = 5)
        self.cb6.grid(row = 2, column =5, padx = 5, pady = 5)
        self.cb7.grid(row = 2, column =6, padx = 5, pady = 5)
        self.cb8.grid(row = 2, column =7, padx = 5, pady = 5)

        ##１セット目
        #入力ウィンドウの作成
        self.entry1 = tk.Entry(self.root,width = 25)
        self.entry1.grid(row = 1, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,1,1,'分岐',self.branch,1)

        self.entry2 = tk.Entry(self.root,width = 25)
        self.entry2.grid(row = 2, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,2,1,'分岐',self.branch,2)

        self.entry3 = tk.Entry(self.root,width = 25)
        self.entry3.grid(row = 3, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,3,1,'分岐',self.branch,3)

        self.entry4 = tk.Entry(self.root,width = 25)
        self.entry4.grid(row = 4, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,4,1,'分岐',self.branch,4)

        self.entry5 = tk.Entry(self.root,width = 25)
        self.entry5.grid(row = 5, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,5,1,'分岐',self.branch,5)

        self.entry6 = tk.Entry(self.root,width = 25)
        self.entry6.grid(row = 6, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,6,1,'分岐',self.branch,6)

        self.entry7 = tk.Entry(self.root,width = 25)
        self.entry7.grid(row = 7, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,7,1,'分岐',self.branch,7)
        
        self.entry8 = tk.Entry(self.root,width = 25)
        self.entry8.grid(row = 8, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,8,1,'分岐',self.branch,8)
        
        self.entry9 = tk.Entry(self.root,width = 25)
        self.entry9.grid(row = 9, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,9,1,'分岐',self.branch,9)

        self.entry10 = tk.Entry(self.root,width = 25)
        self.entry10.grid(row = 10, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,10,1,'分岐',self.branch,10)

        self.entry11 = tk.Entry(self.root,width = 25)
        self.entry11.grid(row = 11, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,11,1,'分岐',self.branch,11)

        self.branchEntry = tk.Entry(self.root,width = 25)
        self.branchEntry.grid(row = 12, column =0, padx = 5, pady = 5)
        
        self.entry1.delete(0,'end')
        self.entry1.insert(0,'0_-15_75_45')
        self.entry2.delete(0,'end')
        self.entry2.insert(0,'15_0_75_45')
        self.entry3.delete(0,'end')
        self.entry3.insert(0,'-15_0_40_10')
        self.entry4.delete(0,'end')
        self.entry4.insert(0,'-5_0_60_80')
        self.entry5.delete(0,'end')
        self.entry5.insert(0,'5_10_85_85')
        self.entry6.delete(0,'end')
        #self.#entry6.insert(0,'-2_4_80_51')
        self.entry7.delete(0,'end')
        #self.#entry7.insert(0,'5_0_55_60')
        self.entry8.delete(0,'end')
        self.entry8.insert(0,'2_6_70_53.5')
        self.entry9.delete(0,'end')
        self.entry9.insert(0,'6_35_70_22.5')
        self.entry10.delete(0,'end')
        self.entry10.insert(0,'-30_0_80_92')
        #self.#entry7.delete(0,'end')
        #self.#entry7.insert(0,'5_0_55_60')
        #self.#entry8.delete(0,'end')
        #self.#entry8.insert(0,'2_5_65_65')
        #self.#entry9.delete(0,'end')
        #self.#entry9.insert(0,'5_-28_45_60')
        #self.#entry10.delete(0,'end')
        #self.#entry10.insert(0,'-10_5_35_87.5')
        self.entry11.delete(0,'end')
        #self.#entry11.insert(0,'-3_0_45_65')
        self.branchEntry.delete(0,'end')
        self.branchEntry.insert(0,'1')
        
        self.root.mainloop()



    #ボタン関数の定義
    def init(self):
        self.robot.motors[0].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[1].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[2].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[3].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[4].insertOrder(MotorModeOrder(MotorMode.VelocityNormal,0))
        self.robot.motors[5].insertOrder(MotorModeOrder(MotorMode.VelocityNormal,0))
        self.robot.motors[6].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[7].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[8].insertOrder(MotorModeOrder(MotorMode.PosNormal,0))
        self.robot.motors[9].insertOrder(MotorModeOrder(MotorMode.VelocityNormal,0))
        self.robot.motors[10].insertOrder(MotorModeOrder(MotorMode.VelocityNormal,0))
        self.robot.motors[11].insertOrder(MotorModeOrder(MotorMode.VelocityNormal,0))

        #初期姿勢
        data = np.loadtxt("Data/normal_switching.csv",delimiter=",")
        self.robot.motors[0].insertOrder(PosOrder(0,0))
        self.robot.motors[1].insertOrder(PosOrder(round(data[0,0] / math.pi * 180,2),0))
        self.robot.motors[2].insertOrder(PosOrder(round(data[0,1] / math.pi * 180,2),0))
        self.robot.motors[3].insertOrder(PosOrder(round(data[0,2] / math.pi * 180,2),0))
        self.robot.motors[4].insertOrder(VelocityOrder(0,0))
        self.robot.motors[5].insertOrder(VelocityOrder(0,0))
        self.robot.motors[6].insertOrder(PosOrder(round(data[0,3] / math.pi * 180,2),0))
        self.robot.motors[7].insertOrder(PosOrder(round(data[0,4] / math.pi * 180,2),0))
        self.robot.motors[8].insertOrder(PosOrder(round(data[0,5] / math.pi * 180,2),0))
        self.robot.motors[9].insertOrder(VelocityOrder(0,0))
        self.robot.motors[10].insertOrder(VelocityOrder(0,0))
        self.robot.motors[11].insertOrder(VelocityOrder(0,0))
        OutputController().pushStep()       

    
    def forward(self):   
        print("○○○Forward○○○")
        self.v1.set(True) 
        self.v2.set(False)
        self.robot.motors[4].insertOrder(VelocityOrder(-212,0))
        self.robot.motors[5].insertOrder(VelocityOrder(212,0))
        self.robot.motors[9].insertOrder(VelocityOrder(-212,0))
        self.robot.motors[10].insertOrder(VelocityOrder(212,0))
        OutputController().pushStep()
    
    def back(self):            
        print("○○○Back○○○")
        self.v1.set(False) 
        self.v2.set(True)
        self.robot.motors[4].insertOrder(VelocityOrder(212,0))
        self.robot.motors[5].insertOrder(VelocityOrder(-212,0))
        self.robot.motors[9].insertOrder(VelocityOrder(212,0))
        self.robot.motors[10].insertOrder(VelocityOrder(-212,0))
        OutputController().pushStep()
    
    def stop(self):            
        print("○○○Stop○○○")
        self.robot.motors[4].insertOrder(VelocityOrder(0,0))
        self.robot.motors[5].insertOrder(VelocityOrder(0,0))
        self.robot.motors[9].insertOrder(VelocityOrder(0,0))
        self.robot.motors[10].insertOrder(VelocityOrder(0,0))
        OutputController().pushStep()
    
    def free(self):       
        print("○○○Free○○○")
        
        self.robot.motors[0].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[1].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[2].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[3].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[4].insertOrder(MotorModeOrder(MotorMode.VelocityFree,0))
        self.robot.motors[5].insertOrder(MotorModeOrder(MotorMode.VelocityFree,0))
        self.robot.motors[6].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[7].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[8].insertOrder(MotorModeOrder(MotorMode.PosFree,0))
        self.robot.motors[9].insertOrder(MotorModeOrder(MotorMode.VelocityFree,0))
        self.robot.motors[10].insertOrder(MotorModeOrder(MotorMode.VelocityFree,0))
        self.robot.motors[11].insertOrder(MotorModeOrder(MotorMode.VelocityFree,0))

        self.robot.motors[4].insertOrder(ResetMotorOrder(0))
        self.robot.motors[5].insertOrder(ResetMotorOrder(0))
        self.robot.motors[9].insertOrder(ResetMotorOrder(0))
        self.robot.motors[10].insertOrder(ResetMotorOrder(0))
        OutputController().pushStep()

    
    def wintchWind(self):
        self.robot.motors[11].insertOrder(VelocityOrder(-200,0))
        OutputController().pushStep()
    
    def wintchFeed(self):
        self.robot.motors[11].insertOrder(VelocityOrder(200,0))
        OutputController().pushStep()

    def wintchStop(self):
        self.robot.motors[11].insertOrder(VelocityOrder(0,0))
        OutputController().pushStep()
    
    def change(self):           
        print("○○○Change○○○")
        if self.v3.get() == True:
            if (self.v5.get() == False) and (self.v6.get() == False):
                self.Branch("normal_switching.csv",self.v3.get(),self.v8.get())
                self.v3.set(False) 
                self.v4.set(True) 
            elif (self.v5.get() == True) and (self.v6.get() == True):
                self.Branch("tension_switching.csv",self.v3.get(),self.v8.get())
                self.v3.set(False) 
                self.v4.set(True)
            else:
                print("左右分岐モードの切り替えはできません")
            
        else:
            if (self.v5.get() == False) and (self.v6.get() == False):
                self.Branch("normal_switching.csv",self.v3.get(),self.v8.get())
                self.v3.set(True) 
                self.v4.set(False)
            elif (self.v5.get() == True) and (self.v6.get() == True):
                self.Branch("tension_switching.csv",self.v3.get(),self.v8.get())
                self.v3.set(True) 
                self.v4.set(False)
            else:
                print("左右分岐モードの切り替えはできません")
        
    def change_f(self):            
        if self.v3.get() == True:                                     #通常どおりにプログラムを流すか(右分岐モードのとき)
            if self.v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("f_tension_r-n.csv",self.v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("f_tension_r-t.csv",self.v3.get(),False)
                self.v5.set(True)
            else:                                                    #前輪にテンションがかかっているとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("f_normal_r-n.csv",self.v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("f_normal_r-t.csv",self.v3.get(),False)
                self.v5.set(False)     
        else:                                                    #逆向きにプログラムを流すか(左分岐モードのとき)
            if self.v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("r_tension_f-n.csv",self.v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("r_tension_f-t.csv",self.v3.get(),False)
                self.v5.set(True)
            else:                                                    #前輪にテンションがかかっているとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("r_normal_f-n.csv",self.v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("r_normal_f-t.csv",self.v3.get(),False)
                self.v5.set(False)     
        
    def change_r(self):            
        if self.v3.get() == True:                                     #通常どおりにプログラムを流すか(右分岐モードのとき)
            if self.v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("r_tension_f-n.csv",self.v3.get(),False)
                    self.v6.set(True) 
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("r_normal_f-n.csv",self.v3.get(),False)
                    self.v6.set(False)
            else:                                                    #前輪にテンションがかかっているとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("r_tension_f-t.csv",self.v3.get(),False)
                    self.v6.set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("r_normal_f-t.csv",self.v3.get(),False)
                    self.v6.set(False)
                
        else:                                                    #逆向きにプログラムを流すか(左分岐モードのとき)
            if self.v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("f_tension_r-n.csv",self.v3.get(),False)
                    self.v6.set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("f_normal_r-n.csv",self.v3.get(),False)
                    self.v6.set(False)
            else:                                                    #前輪にテンションがかかっているとき
                if self.v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    self.Branch("f_tension_r-t.csv",self.v3.get(),False)
                    self.v6.set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    self.Branch("f_normal_r-t.csv",self.v3.get(),False)
                    self.v6.set(False)

    def branch(self,num):
        if self.v3.get() == True:         #右分岐かどうか
            if self.v1.get() == True:       #前進中かどうか
                mode = 1
            else:
                mode = 2
        else:
            if self.v1.get() == True:       #前進中かどうか
                mode = 2
            else:
                mode = 1
    
        
        if(num == 1):value = self.entry1.get()
        elif(num == 2):value = self.entry2.get()
        elif(num == 3):value = self.entry3.get()
        elif(num == 4):value = self.entry4.get()
        elif(num == 5):value = self.entry5.get()
        elif(num == 6):value = self.entry6.get()
        elif(num == 7):value = self.entry7.get()
        elif(num == 8):value = self.entry8.get()
        elif(num == 9):value = self.entry9.get()
        elif(num == 10):value = self.entry10.get()
        elif(num == 11):value = self.entry11.get()

        csv_name = value + '_' + str(mode) + '.csv'    
        if self.v7.get() == True:
            csv_name = value + '_' + str(mode) + '_T' + '.csv'
            self.v5.set(self.reverse(self.v5.get()))
            self.v6.set(self.reverse(self.v6.get()))
        
        print(csv_name,num)
        self.Branch(csv_name,self.v1.get(),self.v8.get())

    def Branch(self,csvfile,value,save):
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

        t=0
        for line in range(0,n-1):#行数-1
            #if(t%4==1):continue;#早めに後輪を分岐する
            #log[t][0] = time.perf_counter() - time_start


            self.robot.motors[id[1]].insertOrder(PosOrder(round(data[line,0]/math.pi*180,2),t))
            self.robot.motors[id[2]].insertOrder(PosOrder(round(data[line,1]/math.pi*180,2),t))
            self.robot.motors[id[3]].insertOrder(PosOrder(round(data[line,2]/math.pi*180,2),t))
            self.robot.motors[id[6]].insertOrder(PosOrder(round(data[line,3]/math.pi*180,2),t))
            self.robot.motors[id[7]].insertOrder(PosOrder(round(data[line,4]/math.pi*180,2),t))
            self.robot.motors[id[8]].insertOrder(PosOrder(round(data[line,5]/math.pi*180,2),t))

            self.robot.motors[id[4]].insertOrder(VelocityOrder(round(-direct*data[line,6]),t))
            self.robot.motors[id[5]].insertOrder(VelocityOrder(round(direct*data[line,6]),t))
            self.robot.motors[id[9]].insertOrder(VelocityOrder(round(-direct*data[line,7]),t))
            self.robot.motors[id[10]].insertOrder(VelocityOrder(round(direct*data[line,7]),t))
            
            t+=0.048

        OutputController().pushStep()


        #log[t][1] = data[t,0]/pi*180;log[t][2] = data[t,1]/pi*180;log[t][3] = data[t,2]/pi*180;log[t][4] = data[t,3]/pi*180;log[t][5] = data[t,4]/pi*180;log[t][6] = data[t,5]/pi*180
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
        #    #print("abc")
        #    with open('log_'+csvfile,'w') as f:
        #        writer = csv.writer(f,lineterminator='\n')
        #        writer.writerows(log)

    def reverse(self,value):
        if value == True:
            ret = False
        else:
            ret = True
        return ret

    
    def branchAngle(self,GammalAngle,TurnAngle,Langle,Rangle):
        if self.v3.get() == True:         #右分岐かどうか
            if self.v1.get() == True:       #前進中かどうか
                mode = 1
            else:
                mode = 2
        else:
            if self.v1.get() == True:       #前進中かどうか
                mode = 2
            else:
                mode = 1
        while Langle + Rangle < 100:
            if Langle < 80:Langle = Langle + 5
            if Rangle < 80:Rangle = Rangle + 5
    
        value = str(GammalAngle) + "_" + str(TurnAngle) + "_" + str(Langle) + "_" + str(Rangle)
        csv_name = value + '_' + str(mode) + '.csv'    
        if self.v7.get() == True:
            csv_name = value + '_' + str(mode) + '_T' + '.csv'
            self.v5.set(reverse(self.v5.get()))
            self.v6.set(reverse(self.v6.get()))
        
        print(csv_name,"AngleMode")
        Branch(csv_name,self.v1.get(),self.v8.get())

