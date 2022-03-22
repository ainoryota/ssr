
import GUI_serial as Gs

import platform
from functools import partial

import sys
import cv2
import time

from PIL import Image,ImageTk #udo pip install pillow
from SSC_ImageRecognition7 import ImageReconition
from SSC_ImageRecognition7 import ResetLog

from CalcBranch import IsBranch
import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit


from threading import Thread
import threading

from System import System
from Robot import RealRobot
from Form import Form


print("Start")
system=System();
robot=RealRobot();
form=Form(robot);
quit()


VIDEOMODE = False
MOVEMODE = True
branchcount = 2#start branch No



def reverse(value):
    if value == True:
        ret = False
    else:
        ret = True
    return ret




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
    
        
    if(num == 1):value = entry1.get()
    elif(num == 2):value = entry2.get()
    elif(num == 3):value = entry3.get()
    elif(num == 4):value = entry4.get()
    elif(num == 5):value = entry5.get()
    elif(num == 6):value = entry6.get()
    elif(num == 7):value = entry7.get()
    elif(num == 8):value = entry8.get()
    elif(num == 9):value = entry9.get()
    elif(num == 10):value = entry10.get()
    elif(num == 11):value = entry11.get()

    csv_name = value + '_' + str(mode) + '.csv'    
    if v7.get() == True:
        csv_name = value + '_' + str(mode) + '_T' + '.csv'
        v5.set(reverse(v5.get()))
        v6.set(reverse(v6.get()))
        
    print(csv_name,num)
    Gs.Branch(ser,csv_name,v1.get(),v8.get())

def branchAngle(GammalAngle,TurnAngle,Langle,Rangle):
    global IsMovingNow;
    IsMovingNow=True;
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
    while Langle+Rangle<100:
        if Langle<80:Langle=Langle+5
        if Rangle<80:Rangle=Rangle+5
    
    value = str(GammalAngle) + "_" + str(TurnAngle) + "_" + str(Langle) + "_" + str(Rangle)
    csv_name = value + '_' + str(mode) + '.csv'    
    if v7.get() == True:
        csv_name = value + '_' + str(mode) + '_T' + '.csv'
        v5.set(reverse(v5.get()))
        v6.set(reverse(v6.get()))
        
    print(csv_name,"AngleMode")

    Gs.Branch(ser,csv_name,v1.get(),v8.get())
    IsMovingNow=False;



class VideoStream:
    def __init__(self, resolution=(640, 360), framerate=15):
        self.debugMode = False
        self.color_image = np.zeros((resolution[0], resolution[1]))
        self.depth_image = self.color_image
        self.resolution = resolution
        self.framerate = framerate
        
        self.imu_pipe = rs.pipeline()
        self.imu_config = rs.config()
        self.imu_config.enable_stream(rs.stream.gyro)
        self.imu_config.enable_stream(rs.stream.accel)
        self.acc = []
        self.gyro = []
        
        self.vid_pipe = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, framerate)
        self.config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, framerate)
        
        
    def start_camera(self):
        self.vid_pipe.start(self.config)
        Thread(target=self.update_cam).start()       

    def start_imu(self):
        self.imu_pipe.start(self.imu_config)
        Thread(target=self.update_imu).start()

    def update_cam(self):
        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                vid_frames = self.vid_pipe.wait_for_frames()
                depth_frame = vid_frames.get_depth_frame()
                color_frame = vid_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                self.depth_image = np.asanyarray(depth_frame.get_data())
                self.color_image = np.asanyarray(color_frame.get_data())
                if(self.debugMode):break
        except:
            self.vid_pipe.stop()
            print("Error in Vision", sys.exc_info())

        finally:
            self.vid_pipe.stop()


    def update_imu(self):
        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                mot_frames = self.imu_pipe.wait_for_frames()
                self.acc = mot_frames[0].as_motion_frame().get_motion_data()
                self.gyro = mot_frames[1].as_motion_frame().get_motion_data()
                if(self.debugMode):break
        except:
            self.imu_pipe.stop()
            print("Error in Vision", sys.exc_info())

        finally:
            self.imu_pipe.stop()



def realsense_init():
    global cc
    global il,timerlabel
    global align,pipeline
    global vs


    vs = VideoStream()
    time.sleep(1)
    vs.start_imu()
    vs.start_camera()
    time.sleep(1)
    
    img = Image.open('testResult/test.png')
    testImgA = ImageTk.PhotoImage(img)
    il = tk.Label(root,image=testImgA)
    il.grid(row=3, column=4,columnspan=30,rowspan=10)

    timerlabel = tk.Label(root,text="")
    timerlabel.grid(row=33, column=7)
    getRealsense()
    



#@jit("f8[:,:]()")
def getRealsense():

    start = time.time()
    global cc, il,timerlabel
    global branchdata
    global align,pipeline
    minDistance = 0
    maxDistance = 1500
    
    
    accel = vs.acc
    gyro = vs.gyro
    color_image = vs.color_image
    depth_image = vs.depth_image


    depth_image = np.where(depth_image > 5000,0,depth_image)#視認性の観点から5000以上なら0に
    original_depth_colormap = cv2.resize(cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET),(640,360)).copy()
    for p in list(zip(*np.where(depth_image == 0))):
        original_depth_colormap[p] = [0,0,0]

    depth_image = np.where(depth_image > maxDistance,0,depth_image)#索道らしくない距離を除去

    #これがないと型が合わない。例えばdepth_colormap=depth_image-minDistance;はだめ
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0), cv2.COLORMAP_JET)

    for x in range(640):
        for y in range(360):
            depth_colormap[y][x] = max(0,depth_image[y][x] - minDistance) * (255 / (maxDistance - minDistance))

    for p in list(zip(*np.where(depth_image == 0))):
        depth_colormap[p] = [0,0,0]

    #画像表示
    color_image_s = cv2.resize(color_image, (640, 360))
    depth_colormap_s = cv2.resize(depth_colormap, (640, 360))

    images = np.hstack((depth_colormap_s,original_depth_colormap))
    
    img = images
    

    image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # imreadはBGRなのでRGBに変換
    image_pil = Image.fromarray(image_rgb) # RGBからPILフォーマットへ変換
    testImg = ImageTk.PhotoImage(image_pil)

    result = ImageReconition(image_rgb,accel)
    testImg = image_rgb
    testImg = result[0]
    testImg = cv2.resize(testImg,dsize=(640,360))


    testImg = Image.fromarray(testImg)
    testImg = ImageTk.PhotoImage(testImg)

    

    il.configure(image=testImg)
    il.image = testImg
    
    timer = math.floor((time.time() - start) * 1000)
    timerlabel.configure(text="{0} ms".format(timer))
    branchdata.append([result[1],result[2],timer])
    print("★",'{:.2f}'.format(result[3]),result[1],result[8])
    if(result[3] > 0.13 and result[1] < 150 and IsMovingNow==False):
        print("■■■■■分岐",result[4],result[5],result[6],result[7],result[8])
        if(v_auto.get()):
            SleepLength = 0
            TimeCounter = 1
            for i in range(4):
                SleepLength+=max(0,result[8][i] - result[8][i + 1])
                if(result[8][i] - result[8][i + 1] > 0):TimeCounter+=700
            #TimeCounterでSleepLengthだけ進んでいる
            SleepVel = 1000 * SleepLength / TimeCounter

            SleepTime = result[1] / SleepVel
            SleepTime = max(0,SleepTime)

            print("Sleep",SleepTime)
            ResetLog()


            time.sleep(SleepTime)
            result[4] = max(-20,result[4])
            result[4] = min(20,result[4])
            result[5] = max(-20,result[5])
            result[5] = min(20,result[5])
            result[6] = max(-80,result[6])
            result[6] = min(80,result[6])
            result[7] = max(-80,result[7])
            result[7] = min(80,result[7])
            branchAngle(result[4],result[5],result[6],result[7])
            



    root.after(10,getRealsense)

#電源をつけてからGUIを起動する
print()




if(MOVEMODE):
    if(platform.system() != "Windows"):
        ser = serial.Serial('/dev/ttyUSB0',115200)    
    else:
        ser = serial.Serial("COM4", 115200)





cc = 0
il = 0
timerlabel = 0
branchdata = []
IsMovingNow=False;


sys.stderr.write("*** 開始 ***\n")
#cam_init()
realsense_init()
root.mainloop()
sys.stderr.write("*** 終了 ***\n")
