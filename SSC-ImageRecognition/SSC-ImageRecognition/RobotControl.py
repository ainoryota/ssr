#sudo apt update
#sudo apt -yV upgrade
#sudo apt -y install python-dev python-pip python-setuptools
#sudo apt -y install libopencv-dev opencv-data
#sudo pip3 install opencv-python
#sudo pip3 install opencv-contrib-python; sudo apt-get install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev  libqtgui4  libqt4-test
#sudo apt-get install python3-tk
#sudo pip3 install numba
#sudo pip3 install pyserial


import tkinter as tk
import GUI_serial as Gs
import serial#pip install pyserial
import platform
from functools import partial

import sys
import cv2
import time
import tkinter as tk
from PIL import Image,ImageTk #udo pip install pillow
from SSC_ImageRecognition5 import ImageReconition
from CalcBranch import IsBranch
import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit
from ctypes import windll

VIDEOMODE=False
MOVEMODE=True
branchcount=2#start branch No
windll.winmm.timeBeginPeriod(1)

class PushButton:
    button = []
    def __init__(self,tk,win,row,column,k,com,num=-1):
        if(num==-1):button = tk.Button(win,text = k,command = com)
        else:button = tk.Button(win,text = k,command = partial(com, num))
        button.grid(row = row, column =column, padx = 5, pady = 5)
class Label:
    label = []
    def __init__(self,tk,win,row,column,k):
        label = tk.Label(win,text = k)
        label.grid(row = row, column =column, padx = 5, pady = 5)        
def init():
    entry1.delete(0,'end')
    entry1.insert(0,'20_0_20_70')
    entry2.delete(0,'end')
    entry2.insert(0,'5_10_30_90')
    entry3.delete(0,'end')
    entry3.insert(0,'-10_0_80_30')
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

    Gs.Init(ser)
    
def Fin():
    Gs.Fin(ser)

def reverse(value):
    if value == True:
        ret = False
    else:
        ret = True
    return ret


#前進、後退、停止、フリー、チェックボックス等の設定
#ボタン関数の定義
def forward():            
    v1.set(True) 
    v2.set(False)
    Gs.Forward(ser)
    
def back():            
    v1.set(False) 
    v2.set(True)
    Gs.Back(ser)
    
def stop():            
    Gs.Stop(ser)
    
def free():            
    Gs.Free(ser)
    
def change():            
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
        
def change_f():            
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
        
def change_r():            
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
def wind():            
    Gs.Wind(ser)
    
def feed():            
    Gs.Feed(ser)
    
def stop2():            
    Gs.Stop2(ser)

def branch(num):                    #ボタンがクリックされたら実行
    if v3.get() == True:         #右分岐かどうか
        if v1.get() == True:       #前進中かどうか
            mode = 1
        else:
            mode = 2
    else:
        if v1.get() == True:       #前進中かどうか
            mode = 2
        else:
            mode = 1
    
        
    if(num==1):value=entry1.get();
    elif(num==2):value=entry2.get();
    elif(num==3):value=entry3.get();
    elif(num==4):value=entry4.get();
    elif(num==5):value=entry5.get();
    elif(num==6):value=entry6.get();
    elif(num==7):value=entry7.get();
    elif(num==8):value=entry8.get();
    elif(num==9):value=entry9.get();
    elif(num==10):value=entry10.get();
    elif(num==11):value=entry11.get();

    csv_name = value + '_' + str(mode) + '.csv'    
    if v7.get() == True:
        csv_name = value+ '_' + str(mode) + '_T' + '.csv'
        v5.set(reverse(v5.get()))
        v6.set(reverse(v6.get()))
        
    print(csv_name,num)
    Gs.Branch(ser,csv_name,v1.get(),v8.get())

def getCam():
    start = time.time()
    global cc, il,timerlabel
    global branchdata
    rr, img = cc.read()
    image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # imreadはBGRなのでRGBに変換
    image_pil = Image.fromarray(image_rgb) # RGBからPILフォーマットへ変換
    testImg=ImageTk.PhotoImage( image_pil)

    result=ImageReconition(image_rgb);
    testImg=result[0]
    testImg = Image.fromarray(testImg)
    testImg=ImageTk.PhotoImage( testImg)

    

    il.configure(image=testImg)
    il.image=testImg;
    
    timer=math.floor((time.time()-start)*1000);
    timerlabel.configure(text="{0} ms".format(timer))
    branchdata.append([result[1],result[2],timer])

    if(IsBranch(branchdata)):
        print("分岐",entry1.get())
        if(v_auto.get()):
            time.sleep(1.5)
            num=int(branchEntry.get())
            branch(num)
            branchEntry.delete(0,'end')
            if(num==5):branchEntry.insert(0,str(1))
            else:branchEntry.insert(0,str(num+1))
        branchdata.clear();
        #for hoge in range(10):
            #branchdata.pop(len(branchdata)-1)
        #branchdata.clear();

    root.after(10,getCam)

def cam_init():
    global cc
    global il,timerlabel
    
    if(VIDEOMODE):
        cc = cv2.VideoCapture("robotvideo2.mp4")
    else:
        if(platform.system()=="Windows"):
            cc = cv2.VideoCapture(1)
        else:
            cc = cv2.VideoCapture(0)
        cc.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cc.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc("H","2","6","4"));

    rr, img = cc.read()

    
    img=Image.open('testResult/test.png')
    testImg=ImageTk.PhotoImage(img)
    il=tk.Label(root,image=testImg)
    il.grid(row=3, column=4,columnspan=30,rowspan=10)

    timerlabel=tk.Label(root,text="")
    timerlabel.grid(row=33, column=7)
    getCam();

def realsense_init():
    global cc
    global il,timerlabel
    global align,pipeline


    # ストリーム(Depth/Color)の設定
    config = rs.config()
    #config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    #config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    #
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared,1,640, 360, rs.format.y8, 30)
    #
    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    #config.enable_stream(rs.stream.confidence, 640, 360,rs.format.raw8, 30)
    #config.enable_stream(rs.stream.fisheye, 1)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    #config.enable_stream(rs.stream.pose)
    #

    
    # ストリーミング開始
    pipeline = rs.pipeline()
    profile = pipeline.start(config)

    # Alignオブジェクト生成
    align_to = rs.stream.color
    align = rs.align(align_to)

    
    img=Image.open('testResult/test.png')
    testImg=ImageTk.PhotoImage(img)
    il=tk.Label(root,image=testImg)
    il.grid(row=3, column=4,columnspan=30,rowspan=10)

    timerlabel=tk.Label(root,text="")
    timerlabel.grid(row=33, column=7)
    getRealsense();



#@jit("f8[:,:]()")
def getRealsense():
    start = time.time()
    global cc, il,timerlabel
    global branchdata
    global align,pipeline
    

    # フレーム待ち(Color & Depth)
    frames = pipeline.wait_for_frames()

    #accel=frames[0].as_motion_frame()
    #a=accel.get_motion_data();
    #for frame in frames:
    #    motion_data = frame.as_motion_frame().get_motion_data()
    #    print(motion_data)
        # prints: x: -0.0294199, y: -7.21769, z: -6.41355 for me
        # to get numpy array:
        #print(np.array([motion_data.x, motion_data.y, motion_data.z]))

    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    sekigai_frame = aligned_frames.get_infrared_frame()


    #sekigai_frame = aligned_frames.get_confidence_frame()
    #sekigai_frame = aligned_frames.get_infrared_frame()
    #sekigai_frame = aligned_frames.get_infrared_frame()
    #sekigai_frame = aligned_frames.get_infrared_frame()
    #sekigai_frame = aligned_frames.get_infrared_frame()

    #imageをnumpy arrayに
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    sekigai_frame=np.asanyarray(sekigai_frame.get_data())

    #sekigai_frame=np.asanyarray(sekigai_frame.get_data())
    #sekigai_frame=np.asanyarray(sekigai_frame.get_data())
    #sekigai_frame=np.asanyarray(sekigai_frame.get_data())
    #sekigai_frame=np.asanyarray(sekigai_frame.get_data())
    #sekigai_frame=np.asanyarray(sekigai_frame.get_data())


    
    #depth imageをカラーマップに変換

    minDistance=100
    maxDistance=300

    for x in range(640):
        for y in range(360):
            if(depth_image[y][x]>5000):depth_image[y][x]=0;
            depth_image[y][x]=depth_image[y][x];

    original_depth_colormap = cv2.resize(cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET),(640,360)).copy()

    
    for x in range(640):
        for y in range(360):
            if(depth_image[y][x]==0):
                original_depth_colormap[y][x][0]=0;
                original_depth_colormap[y][x][1]=0;
                original_depth_colormap[y][x][2]=0;
            original_depth_colormap[y][x]=original_depth_colormap[y][x];

    for x in range(640):
        for y in range(360):
            if(depth_image[y][x]>maxDistance):depth_image[y][x]=0;
            depth_image[y][x]=depth_image[y][x];
    
    
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0), cv2.COLORMAP_JET)
    #original_depth_colormap=depth_colormap.copy();
    #original_depth_colormap=cv2.cvtColor(original_depth_colormap, cv2.COLOR_GRAY2RGB)
    #original_depth_colormap_s=cv2.resize(original_depth_colormap, (640, 360))

    for x in range(640):
        for y in range(360):
            depth_colormap[y][x]=max(0,depth_image[y][x]-minDistance)*(255/(maxDistance-minDistance))

    for x in range(640):
        for y in range(360):
            if(depth_image[y][x]==0):
                depth_colormap[y][x]=[0,0,0];
            #elif(depth_image[y][x]>400):
            #    depth_colormap[y][x]=[0,0,0];
            #else:
            #    depth_colormap[y][x]=[255,0,0];

    #depth_colormap = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2GRAY)
    #depth_colormap=cv2.blur(depth_colormap,(5,5))

    #depth_colormap = cv2.Canny(depth_colormap, 50, 110)
    #depth_colormap = cv2.cvtColor(depth_colormap, cv2.COLOR_GRAY2RGB)
    #画像表示
    color_image_s = cv2.resize(color_image, (640, 360))
    depth_colormap_s = cv2.resize(depth_colormap, (640, 360))

    sekigai_image=cv2.cvtColor(sekigai_frame,cv2.COLOR_GRAY2RGB)

    #images = np.hstack((depth_colormap_s,sekigai_image))
    #images = np.hstack((depth_colormap_s,sekigai_image))
    images = np.hstack((depth_colormap_s,original_depth_colormap))
    
    img=images
    

    image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # imreadはBGRなのでRGBに変換
    image_pil = Image.fromarray(image_rgb) # RGBからPILフォーマットへ変換
    testImg=ImageTk.PhotoImage( image_pil)

    result=ImageReconition(image_rgb);
    testImg=result[0]
    testImg=cv2.resize(testImg,dsize=(640,360))


    testImg = Image.fromarray(testImg)
    testImg=ImageTk.PhotoImage( testImg)

    

    il.configure(image=testImg)
    il.image=testImg;
    
    timer=math.floor((time.time()-start)*1000);
    timerlabel.configure(text="{0} ms".format(timer))
    branchdata.append([result[1],result[2],timer])


    #if(IsBranch(branchdata)):
    #    print("分岐",entry1.get())
    #    if(v_auto.get()):
    #        time.sleep(1.5)
    #        num=int(branchEntry.get())
    #        branch(num)
    #        branchEntry.delete(0,'end')
    #        if(num==5):branchEntry.insert(0,str(1))
    #        else:branchEntry.insert(0,str(num+1))
    #    branchdata.clear();
    if(result[3]>0.1 and result[1]<100):
        print("分岐",entry1.get())
        if(v_auto.get()):
            #time.sleep(0.5)
            num=int(branchEntry.get())
            branch(num)
            branchEntry.delete(0,'end')
            if(num==5):branchEntry.insert(0,str(1))
            else:branchEntry.insert(0,str(num+1))

    root.after(10,getRealsense)

#電源をつけてからGUIを起動する
print()



if(MOVEMODE):
    if(platform.system()!="Windows"):
        ser = serial.Serial('/dev/ttyUSB0',115200)    
    else:
        ser = serial.Serial("COM4", 115200)


#ウィンドウの生成
root = tk.Tk()  
root.title('GUIコントローラ')
root.geometry('1000x720+150+150')


#ラベルの生成
Label(tk,root,0,0,'斜度_回転角_右旋回角_左旋回角')
Label(tk,root,0,3,'     ')#空白ラベル
Label(tk,root,0,9,'ロボットの状態') 

#ボタンの作成
PushButton(tk,root,0,4,' 前進 ',forward)
PushButton(tk,root,0,5,' 後退',back)
PushButton(tk,root,0,6,' 停止 ',stop)
PushButton(tk,root,0,7,' フリー ',free)
PushButton(tk,root,0,8,' 左右切り替え',change)
PushButton(tk,root,0,9,' 前輪切り替え',change_f)
PushButton(tk,root,0,10,' 後輪切り替え',change_r)

PushButton(tk,root,1,8,' 巻取り ',wind)
PushButton(tk,root,1,9,' 繰り出し',feed)
PushButton(tk,root,1,10,' 停止  ',stop2)

PushButton(tk,root,2,8,' 初期化 ',init)
PushButton(tk,root,2,9,' 終了',Fin)




#チェックボックスの作成
v1 = tk.BooleanVar()
v2 = tk.BooleanVar()
v3 = tk.BooleanVar()
v4 = tk.BooleanVar()
v5 = tk.BooleanVar()
v6 = tk.BooleanVar()
v7 = tk.BooleanVar()
v8 = tk.BooleanVar()
v_auto = tk.BooleanVar()

v1.set(True) 
v2.set(False) 
v3.set(True) 
v4.set(False)
v5.set(False) 
v6.set(False)
v7.set(False)
v8.set(False)
v_auto.set(False)

cb1 = tk.Checkbutton(root,text='前進　',variable = v1)
cb2 = tk.Checkbutton(root,text='後退　',variable = v2)
cb3 = tk.Checkbutton(root,text='右分岐',variable = v3)
cb4 = tk.Checkbutton(root,text='左分岐',variable = v4)
cb_auto = tk.Checkbutton(root,text='auto分岐',variable = v_auto)
cb5 = tk.Checkbutton(root,text='前輪　',variable = v5)
cb6 = tk.Checkbutton(root,text='後輪　',variable = v6)
cb7 = tk.Checkbutton(root,text='テンション分岐モード',variable = v7)
cb8 = tk.Checkbutton(root,text='セーブモード',variable = v8)

cb1.grid(row = 1, column =4, padx = 5, pady = 5)
cb2.grid(row = 1, column =5, padx = 5, pady = 5)
cb3.grid(row = 1, column =6, padx = 5, pady = 5)
cb4.grid(row = 1, column =7, padx = 5, pady = 5)
cb_auto.grid(row = 13, column =1, padx = 5, pady = 5)
cb5.grid(row = 2, column =4, padx = 5, pady = 5)
cb6.grid(row = 2, column =5, padx = 5, pady = 5)
cb7.grid(row = 2, column =6, padx = 5, pady = 5)
cb8.grid(row = 2, column =7, padx = 5, pady = 5)





##１セット目
#入力ウィンドウの作成
entry1 = tk.Entry(root,width = 25)
entry1.grid(row = 1, column =0, padx = 5, pady = 5)
PushButton(tk,root,1,1,'分岐',branch,1)

entry2 = tk.Entry(root,width = 25)
entry2.grid(row = 2, column =0, padx = 5, pady = 5)
PushButton(tk,root,2,1,'分岐',branch,2)

entry3 = tk.Entry(root,width = 25)
entry3.grid(row = 3, column =0, padx = 5, pady = 5)
PushButton(tk,root,3,1,'分岐',branch,3)

entry4 = tk.Entry(root,width = 25)
entry4.grid(row = 4, column =0, padx = 5, pady = 5)
PushButton(tk,root,4,1,'分岐',branch,4)

entry5 = tk.Entry(root,width = 25)
entry5.grid(row = 5, column =0, padx = 5, pady = 5)
PushButton(tk,root,5,1,'分岐',branch,5)

entry6 = tk.Entry(root,width = 25)
entry6.grid(row = 6, column =0, padx = 5, pady = 5)
PushButton(tk,root,6,1,'分岐',branch,6)

entry7 = tk.Entry(root,width = 25)
entry7.grid(row = 7, column =0, padx = 5, pady = 5)
PushButton(tk,root,7,1,'分岐',branch,7)
entry8 = tk.Entry(root,width = 25)
entry8.grid(row = 8, column =0, padx = 5, pady = 5)
PushButton(tk,root,8,1,'分岐',branch,8)
entry9 = tk.Entry(root,width = 25)
entry9.grid(row = 9, column =0, padx = 5, pady = 5)
PushButton(tk,root,9,1,'分岐',branch,9)
entry10 = tk.Entry(root,width = 25)
entry10.grid(row = 10, column =0, padx = 5, pady = 5)
PushButton(tk,root,10,1,'分岐',branch,10)
entry11 = tk.Entry(root,width = 25)
entry11.grid(row = 11, column =0, padx = 5, pady = 5)
PushButton(tk,root,11,1,'分岐',branch,11)

branchEntry = tk.Entry(root,width = 25)
branchEntry.grid(row = 12, column =0, padx = 5, pady = 5)



cc=0;
il=0;
timerlabel=0;
branchdata=[];


sys.stderr.write("*** 開始 ***\n")
#cam_init()
realsense_init()
root.mainloop()
sys.stderr.write("*** 終了 ***\n")
