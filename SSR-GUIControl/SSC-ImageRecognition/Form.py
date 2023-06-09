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
from Branch import Branch
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
        self.br=Branch(self.robot)

        
        #ウィンドウの生成
        self.root = tk.Tk()  
        self.root.title('GUIコントローラ')
        self.root.geometry('1000x500+150+150')


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
        self.data=dict();

        self.data["v1"]= tk.BooleanVar()#前進
        self.data["v2"]= tk.BooleanVar()#後退
        self.data["v3"]= tk.BooleanVar()#右分岐
        self.data["v4"]= tk.BooleanVar()#左分岐
        self.data["v5"]= tk.BooleanVar()#auto分岐
        self.data["v6"]= tk.BooleanVar()#前輪T
        self.data["v7"]= tk.BooleanVar()
        self.data["v8"]= tk.BooleanVar()
        self.data["v_auto"]= tk.BooleanVar()

        self.data["v1"].set(True)
        self.data["v2"].set(False) 
        self.data["v3"].set(True) 
        self.data["v4"].set(False)
        self.data["v5"].set(True) 
        self.data["v6"].set(True)
        self.data["v7"].set(False)
        self.data["v8"].set(False)
        self.data["v_auto"].set(False)

        self.data["cb1"]= tk.Checkbutton(self.root,text='前進　',variable = self.data["v1"])
        self.data["cb2"]= tk.Checkbutton(self.root,text='後退　',variable = self.data["v2"])
        self.data["cb3"]= tk.Checkbutton(self.root,text='右分岐',variable = self.data["v3"])
        self.data["cb4"]= tk.Checkbutton(self.root,text='左分岐',variable = self.data["v4"])
        self.data["cb_auto"]= tk.Checkbutton(self.root,text='auto分岐',variable = self.data["v_auto"])
        self.data["cb5"]= tk.Checkbutton(self.root,text='前輪T　',variable = self.data["v5"])
        self.data["cb6"]= tk.Checkbutton(self.root,text='後輪T　',variable = self.data["v6"])
        self.data["cb7"]= tk.Checkbutton(self.root,text='テンション分岐モード',variable = self.data["v7"])
        self.data["cb8"]= tk.Checkbutton(self.root,text='セーブモード',variable = self.data["v8"])

        self.data["cb1"].grid(row = 1, column =4, padx = 5, pady = 5)
        self.data["cb2"].grid(row = 1, column =5, padx = 5, pady = 5)
        self.data["cb3"].grid(row = 1, column =6, padx = 5, pady = 5)
        self.data["cb4"].grid(row = 1, column =7, padx = 5, pady = 5)
        self.data["cb_auto"].grid(row = 13, column =1, padx = 5, pady = 5)
        self.data["cb5"].grid(row = 2, column =4, padx = 5, pady = 5)
        self.data["cb6"].grid(row = 2, column =5, padx = 5, pady = 5)
        self.data["cb7"].grid(row = 2, column =6, padx = 5, pady = 5)
        self.data["cb8"].grid(row = 2, column =7, padx = 5, pady = 5)

        ##１セット目
        #入力ウィンドウの作成
        self.data["entry1"]= tk.Entry(self.root,width = 25)
        self.data["entry1"].grid(row = 1, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,1,1,'分岐',self.branch,1)

        self.data["entry2"]= tk.Entry(self.root,width = 25)
        self.data["entry2"].grid(row = 2, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,2,1,'分岐',self.branch,2)

        self.data["entry3"]= tk.Entry(self.root,width = 25)
        self.data["entry3"].grid(row = 3, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,3,1,'分岐',self.branch,3)

        self.data["entry4"]= tk.Entry(self.root,width = 25)
        self.data["entry4"].grid(row = 4, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,4,1,'分岐',self.branch,4)

        self.data["entry5"]= tk.Entry(self.root,width = 25)
        self.data["entry5"].grid(row = 5, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,5,1,'分岐',self.branch,5)

        self.data["entry6"]= tk.Entry(self.root,width = 25)
        self.data["entry6"].grid(row = 6, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,6,1,'分岐',self.branch,6)

        self.data["entry7"]= tk.Entry(self.root,width = 25)
        self.data["entry7"].grid(row = 7, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,7,1,'分岐',self.branch,7)
        
        self.data["entry8"]= tk.Entry(self.root,width = 25)
        self.data["entry8"].grid(row = 8, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,8,1,'分岐',self.branch,8)
        
        self.data["entry9"]= tk.Entry(self.root,width = 25)
        self.data["entry9"].grid(row = 9, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,9,1,'分岐',self.branch,9)

        self.data["entry10"]= tk.Entry(self.root,width = 25)
        self.data["entry10"].grid(row = 10, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,10,1,'分岐',self.branch,10)

        self.data["entry11"]= tk.Entry(self.root,width = 25)
        self.data["entry11"].grid(row = 11, column =0, padx = 5, pady = 5)
        PushButton(tk,self.root,11,1,'分岐',self.branch,11)

        self.data["branchEntry"]= tk.Entry(self.root,width = 25)
        self.data["branchEntry"].grid(row = 12, column =0, padx = 5, pady = 5)
        
        self.data["entry1"].delete(0,'end')
        self.data["entry1"].insert(0,'-10_-20_60_40')
        self.data["entry2"].delete(0,'end')
        self.data["entry2"].insert(0,'10_20_80_60')
        self.data["entry3"].delete(0,'end')
        self.data["entry3"].insert(0,'-0_0_70_50')
        self.data["entry4"].delete(0,'end')
        self.data["entry4"].insert(0,'-5_0_60_80')
        self.data["entry5"].delete(0,'end')
        self.data["entry5"].insert(0,'5_10_85_85')
        self.data["entry6"].delete(0,'end')
        #selfdata..#entry6.insert(0,'-2_4_80_51')
        self.data["entry7"].delete(0,'end')
        #selfdata..#entry7.insert(0,'5_0_55_60')
        self.data["entry8"].delete(0,'end')
        self.data["entry8"].insert(0,'2_6_70_53.5')
        self.data["entry9"].delete(0,'end')
        self.data["entry9"].insert(0,'6_35_70_22.5')
        self.data["entry10"].delete(0,'end')
        self.data["entry10"].insert(0,'-30_0_80_92')
        #self.#entry7.delete(0,'end')
        #self.#entry7.insert(0,'5_0_55_60')
        #self.#entry8.delete(0,'end')
        #self.#entry8.insert(0,'2_5_65_65')
        #self.#entry9.delete(0,'end')
        #self.#entry9.insert(0,'5_-28_45_60')
        #self.#entry10.delete(0,'end')
        #self.#entry10.insert(0,'-10_5_35_87.5')
        self.data["entry11"].delete(0,'end')
        #self.#entry11.insert(0,'-3_0_45_65')
        self.data["branchEntry"].delete(0,'end')
        self.data["branchEntry"].insert(0,'1')
        
        self.rs=RealSense(self.root,self.br,self.data);
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
        d = np.loadtxt("Data/normal_switching.csv",delimiter=",")
        self.robot.motors[0].insertOrder(PosOrder(0,0))
        self.robot.motors[1].insertOrder(PosOrder(round(d[0,0] / math.pi * 180,2),0))
        self.robot.motors[2].insertOrder(PosOrder(round(d[0,1] / math.pi * 180,2),0))
        self.robot.motors[3].insertOrder(PosOrder(round(d[0,2] / math.pi * 180,2),0))
        self.robot.motors[4].insertOrder(VelocityOrder(0,0))
        self.robot.motors[5].insertOrder(VelocityOrder(0,0))
        self.robot.motors[6].insertOrder(PosOrder(round(d[0,3] / math.pi * 180,2),0))
        self.robot.motors[7].insertOrder(PosOrder(round(d[0,4] / math.pi * 180,2),0))
        self.robot.motors[8].insertOrder(PosOrder(round(d[0,5] / math.pi * 180,2),0))
        self.robot.motors[9].insertOrder(VelocityOrder(0,0))
        self.robot.motors[10].insertOrder(VelocityOrder(0,0))
        self.robot.motors[11].insertOrder(VelocityOrder(0,0))
        OutputController().pushStep()       

    
    def forward(self):   
        print("○○○Forward○○○")
        self.data["v1"].set(True) 
        self.data["v2"].set(False)
        self.robot.motors[4].insertOrder(VelocityOrder(-212,0))
        self.robot.motors[5].insertOrder(VelocityOrder(212,0))
        self.robot.motors[9].insertOrder(VelocityOrder(-212,0))
        self.robot.motors[10].insertOrder(VelocityOrder(212,0))
        OutputController().pushStep()
    
    def back(self):            
        print("○○○Back○○○")
        self.data["v1"].set(False) 
        self.data["v2"].set(True)
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
        if self.data["v3"].get() == True:
            if (self.data["v5"].get() == False) and (self.data["v6"].get() == False):
                self.br.FileBranch("normal_switching.csv",self.data["v3"].get(),self.data["v8"].get())
                self.data["v3"].set(False) 
                self.data["v4"].set(True) 
            elif (self.data["v5"].get() == True) and (self.data["v6"].get() == True):
                self.br.FileBranch("tension_switching.csv",self.data["v3"].get(),self.data["v8"].get())
                self.data["v3"].set(False) 
                self.data["v4"].set(True)
            else:
                print("左右分岐モードの切り替えはできません")
            
        else:
            if (self.data["v5"].get() == False) and (self.data["v6"].get() == False):
                self.br.FileBranch("normal_switching.csv",self.data["v3"].get(),self.data["v8"].get())
                self.data["v3"].set(True) 
                self.data["v4"].set(False)
            elif (self.data["v5"].get() == True) and (self.data["v6"].get() == True):
                self.br.FileBranch("tension_switching.csv",self.data["v3"].get(),self.data["v8"].get())
                self.data["v3"].set(True) 
                self.data["v4"].set(False)
            else:
                print("左右分岐モードの切り替えはできません")
        
    def change_f(self):            
        if self.data["v3"].get() == True:                                     #通常どおりにプログラムを流すか(右分岐モードのとき)
            if self.data["v5"].get() == False:                                    #前輪にテンションがかかっていないとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("f_tension_r-n.csv",self.data["v3"].get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("f_tension_r-t.csv",self.data["v3"].get(),False)
                self.data["v5"].set(True)
            else:                                                    #前輪にテンションがかかっているとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("f_normal_r-n.csv",self.data["v3"].get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("f_normal_r-t.csv",self.data["v3"].get(),False)
                self.data["v5"].set(False)     
        else:                                                    #逆向きにプログラムを流すか(左分岐モードのとき)
            if self.data["v5"].get() == False:                                    #前輪にテンションがかかっていないとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("r_tension_f-n.csv",self.data["v3"].get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("r_tension_f-t.csv",self.data["v3"].get(),False)
                self.data["v5"].set(True)
            else:                                                    #前輪にテンションがかかっているとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("r_normal_f-n.csv",self.data["v3"].get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("r_normal_f-t.csv",self.data["v3"].get(),False)
                self.data["v5"].set(False)     
        
    def change_r(self):            
        if self.data["v3"].get() == True:                                     #通常どおりにプログラムを流すか(右分岐モードのとき)
            if self.data["v5"].get() == False:                                    #前輪にテンションがかかっていないとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("r_tension_f-n.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(True) 
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("r_normal_f-n.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(False)
            else:                                                    #前輪にテンションがかかっているとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("r_tension_f-t.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("r_normal_f-t.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(False)
                
        else:                                                    #逆向きにプログラムを流すか(左分岐モードのとき)
            if self.data["v5"].get() == False:                                    #前輪にテンションがかかっていないとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("f_tension_r-n.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("f_normal_r-n.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(False)
            else:                                                    #前輪にテンションがかかっているとき
                if self.data["v6"].get() == False:                                    #後輪にテンションがかかっていないとき
                    self.br.FileBranch("f_tension_r-t.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    self.br.FileBranch("f_normal_r-t.csv",self.data["v3"].get(),False)
                    self.data["v6"].set(False)

    def branch(self,num):
        if self.data["v3"].get() == True:         #右分岐かどうか
            if self.data["v1"].get() == True:       #前進中かどうか
                mode = 1
            else:
                mode = 2
        else:
            if self.data["v1"].get() == True:       #前進中かどうか
                mode = 2
            else:
                mode = 1
    
        
        if(num == 1):value = self.data["entry1"].get()
        elif(num == 2):value = self.data["entry2"].get()
        elif(num == 3):value = self.data["entry3"].get()
        elif(num == 4):value = self.data["entry4"].get()
        elif(num == 5):value = self.data["entry5"].get()
        elif(num == 6):value = self.data["entry6"].get()
        elif(num == 7):value = self.data["entry7"].get()
        elif(num == 8):value = self.data["entry8"].get()
        elif(num == 9):value = self.data["entry9"].get()
        elif(num == 10):value = self.data["entry10"].get()
        elif(num == 11):value = self.data["entry11"].get()

        csv_name = value + '_' + str(mode) + '.csv'    
        if self.data["v7"].get() == True:
            csv_name = value + '_' + str(mode) + '_T' + '.csv'
            self.data["v5"].set(self.reverse(self.data["v5"].get()))
            self.data["v6"].set(self.reverse(self.data["v6"].get()))
        
        print(csv_name,num)
        self.br.FileBranch(csv_name,self.data["v1"].get(),self.data["v8"].get())

 
    def reverse(self,value):
        if value == True:
            ret = False
        else:
            ret = True
        return ret

    


