import tkinter as tk
from functools import partial
import GUI_serial as Gs
from Robot import Robot
from Order import PosOrder
from Order import VelocityOrder
from Order import MotorModeOrder
from Order import ResetMotorOrder
from Order import ResetEncoderOrder
from OutputController import OutputController
from OutputController import MotorMode

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
        PushButton(tk,self.root,2,9,' 終了',self.fin)

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
        self.root.mainloop()

    def setRobot(self,robot):
        self.robot=robot

    #ボタン関数の定義
    def init(self):
        entry1.delete(0,'end')
        entry1.insert(0,'0_-15_75_45')
        entry2.delete(0,'end')
        entry2.insert(0,'15_0_75_45')
        entry3.delete(0,'end')
        entry3.insert(0,'-15_0_40_10')
        entry4.delete(0,'end')
        entry4.insert(0,'-5_0_60_80')
        entry5.delete(0,'end')
        entry5.insert(0,'5_10_85_85')
        entry6.delete(0,'end')
        #entry6.insert(0,'-2_4_80_51')
        entry7.delete(0,'end')
        #entry7.insert(0,'5_0_55_60')
        entry8.delete(0,'end')
        entry8.insert(0,'2_6_70_53.5')
        entry9.delete(0,'end')
        entry9.insert(0,'6_35_70_22.5')
        entry10.delete(0,'end')
        entry10.insert(0,'-30_0_80_92')
        #entry7.delete(0,'end')
        #entry7.insert(0,'5_0_55_60')
        #entry8.delete(0,'end')
        #entry8.insert(0,'2_5_65_65')
        #entry9.delete(0,'end')
        #entry9.insert(0,'5_-28_45_60')
        #entry10.delete(0,'end')
        #entry10.insert(0,'-10_5_35_87.5')
        entry11.delete(0,'end')
        #entry11.insert(0,'-3_0_45_65')
        branchEntry.delete(0,'end')
        branchEntry.insert(0,'1')

        Gs.Init(self.robot.serial)
    
    def fin(self):
        Gs.Fin(self.robot.serial)
    
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
        for motor in self.robot.motors:
            motor.insertOrder(MotorModeOrder(MotorMode.Free,0))
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
        if v3.get() == True:
            if (v5.get() == False) and (v6.get() == False):
                Gs.Branch(ser,"normal_switching.csv",v3.get(),v8.get())
                v3.set(False) 
                v4.set(True) 
            elif (v5.get() == True) and (v6.get() == True):
                Gs.Branch(ser,"tension_switching.csv",v3.get(),v8.get())
                v3.set(False) 
                v4.set(True)
            else:
                print("左右分岐モードの切り替えはできません")
            
        else:
            if (v5.get() == False) and (v6.get() == False):
                Gs.Branch(ser,"normal_switching.csv",v3.get(),v8.get())
                v3.set(True) 
                v4.set(False)
            elif (v5.get() == True) and (v6.get() == True):
                Gs.Branch(ser,"tension_switching.csv",v3.get(),v8.get())
                v3.set(True) 
                v4.set(False)
            else:
                print("左右分岐モードの切り替えはできません")
        
    def change_f(self):            
        if v3.get() == True:                                     #通常どおりにプログラムを流すか(右分岐モードのとき)
            if v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"f_tension_r-n.csv",v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"f_tension_r-t.csv",v3.get(),False)
                v5.set(True)
            else:                                                    #前輪にテンションがかかっているとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"f_normal_r-n.csv",v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"f_normal_r-t.csv",v3.get(),False)
                v5.set(False)     
        else:                                                    #逆向きにプログラムを流すか(左分岐モードのとき)
            if v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"r_tension_f-n.csv",v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"r_tension_f-t.csv",v3.get(),False)
                v5.set(True)
            else:                                                    #前輪にテンションがかかっているとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"r_normal_f-n.csv",v3.get(),False)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"r_normal_f-t.csv",v3.get(),False)
                v5.set(False)     
        
    def change_r(self):            
        if v3.get() == True:                                     #通常どおりにプログラムを流すか(右分岐モードのとき)
            if v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"r_tension_f-n.csv",v3.get(),False)
                    v6.set(True) 
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"r_normal_f-n.csv",v3.get(),False)
                    v6.set(False)
            else:                                                    #前輪にテンションがかかっているとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"r_tension_f-t.csv",v3.get(),False)
                    v6.set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"r_normal_f-t.csv",v3.get(),False)
                    v6.set(False)
                
        else:                                                    #逆向きにプログラムを流すか(左分岐モードのとき)
            if v5.get() == False:                                    #前輪にテンションがかかっていないとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"f_tension_r-n.csv",v3.get(),False)
                    v6.set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"f_normal_r-n.csv",v3.get(),False)
                    v6.set(False)
            else:                                                    #前輪にテンションがかかっているとき
                if v6.get() == False:                                    #後輪にテンションがかかっていないとき
                    Gs.Branch(ser,"f_tension_r-t.csv",v3.get(),False)
                    v6.set(True)
                else:                                                    #後輪にテンションがかかっているとき
                    Gs.Branch(ser,"f_normal_r-t.csv",v3.get(),False)
                    v6.set(False)
    #ウィンチ用のボタン
    def wind(self):            
        Gs.Wind(self.robot.serial)
    
    def feed(self):            
        Gs.Feed(self.robot.serial)
    
    def stop2(self):            
        Gs.Stop2(self.robot.serial)


    def branch(self,num):
        print(num)
        

