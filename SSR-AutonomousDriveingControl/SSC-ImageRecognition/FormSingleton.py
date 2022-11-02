
import time
import serial#pip install pyserial
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder
from Order import MotorModeOrder
from Order import ResetMotorOrder
from Order import ResetEncoderOrder
import math
import copy
import threading
from enum import IntEnum
import queue
from multiprocessing import Process
import platform
import threading
import tkinter as tk
import numpy as np
from matplotlib import pyplot as plt
import tkinter as tk
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.colors
from mpl_toolkits.mplot3d import Axes3D # ３Dグラフ作成のため

class FormSingleton(object):

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                print("New Singleton is OutputController")
                cls._instance = super().__new__(cls)
        return cls._instance

    def setFormData(self,data):
        self.data=data
    
    def getFormData(self):
        return self.data

    def setThreeGraph(self,root):
        self.graphRoot=root

        plt.rcParams['font.size'] = 15
        plt.rcParams['font.family'] = 'Arial'

        fig = plt.figure(figsize=(3, 3), dpi=100)

        self.ax = Axes3D(fig)
        self.ax.set_xlabel('x', labelpad=10)
        self.ax.set_ylabel('y', labelpad=10)
        self.ax.set_zlabel('z', labelpad=10)

        self.graph = FigureCanvasTkAgg(fig, master=root)

        self.updateThreeGraph([0,1,2],[0,2,5],[0,3,4])

        return self.graph

    def updateThreeGraph(self,x,y,z):
        self.ax.cla()
        self.ax.scatter(x, y, z)

        try:
            # do fit
            tmp_A = []
            tmp_b = []
            for i in range(len(x)):
                tmp_A.append([x[i], y[i], 1])
                tmp_b.append(z[i])
            b = np.matrix(tmp_b).T
            A = np.matrix(tmp_A)
            fit = (A.T * A).I * A.T * b
            errors = b - A * fit
            residual = np.linalg.norm(errors)


            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            X,Y = np.meshgrid(np.arange(xlim[0], xlim[1],100),
                              np.arange(ylim[0], ylim[1],100))
            Z = np.zeros(X.shape)
            for r in range(X.shape[0]):
                for c in range(X.shape[1]):
                    Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
            self.ax.plot_wireframe(X,Y,Z, color='k')
        except:
            pass
        
        
        self.graph.draw()




def OutputDone(stepQueue):
    output = Serial()

    while(True):
        if(stepQueue.empty()):
            time.sleep(0.1)
            continue
        orderList = stepQueue.get()
        #OutputController().msgPrint("Start Output:",len(orderList),"datas")


        timer = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            time.sleep(max(data.delay - timer,0))
            timer = data.delay + data.sleepTime

            if(isinstance(data,InitOrder)):#初期化に関する司令を1つのみ処理
                #OutputController().msgPrint("　Init",data)
                output.outputInit(data)

            elif(isinstance(data,PosOrder)):#位置に関する司令を複数個同時に処理
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],PosOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                #OutputController().msgPrint("　Move Pos:",len(dataList),"motors")
                output.outputPos(dataList)

            elif(isinstance(data,VelocityOrder)):#速度に関する司令を複数個同時に処理
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],VelocityOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                #OutputController().msgPrint("　Move Velocity:",len(dataList),"motors")
                output.outputVelocity(dataList)

            elif(isinstance(data,MotorModeOrder)):#ノーマルモードへの変更に関する司令を複数個同時に処理
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],MotorModeOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                #OutputController().msgPrint("　Set MotorMode:",len(dataList),"motors")
                output.setMotorMode(dataList)

            elif(isinstance(data,ResetMotorOrder)):#モータのリセット
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],ResetMotorOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                #OutputController().msgPrint("　Reset Motor:",len(dataList),"motors")
                output.resetMotor(dataList)

            elif(isinstance(data,ResetEncoderOrder)):#エンコーダのリセット
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],ResetEncoderOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                #OutputController().msgPrint("　Reset Encoder:",len(dataList),"motors")
                output.resetEncoder(dataList)
        #OutputController().msgPrint("End Output")
                                                
class MotorMode(IntEnum):
    PosNormal=0x00
    PosHold=0x01
    PosFree=0x02
    VelocityNormal=0x04
    VelocityFree=0x06
    VelocityHold=0x07
    Pos=100
    Velocity=200
    

        
class Serial(object):
    def __init__(self):
        #OutputController().msgPrint("Open Serial Port")
        if(platform.system() != "Windows"):
            self.serial = serial.Serial('/dev/ttyUSB0',115200)    
        else:
            self.serial = serial.Serial("COM4", 115200)

    def write(self,command):
        self.serial.write(command)

    def outputInit(self,order):
        if order.mode == MotorMode.Pos:
            data1 = int(MotorMode.PosFree)
            data2 = int(MotorMode.PosNormal)
        elif order.mode == MotorMode.Velocity:
            data1 = int(MotorMode.VelocityFree)
            data2 = int(MotorMode.VelocityNormal)
        else:
            OutputController().msgPrint("statusの値が不正")

        #OutputController().msgPrint("　　id:",order.id,"mode",order.mode)

                #        Size CMD OP Data ADR CNT
        list1 = [0X08,0X04,0X00,order.id,data1,0X28,0X01]#制御モードの変更
        list2 = [0X08,0X04,0X00,order.id,order.gain,0X5C,0X01]#ゲインを変更
        list3 = [0X08,0X04,0X00,order.id,data2,0X28,0X01]#ノーマルモードに切り替え
        
        list1.insert(7,sum(list1))
        list2.insert(7,sum(list2))
        list3.insert(7,sum(list3))

        self.write(list1)
        time.sleep(order.sleepTime / 3)
        self.write(list2)
        time.sleep(order.sleepTime / 3)
        self.write(list3)
        time.sleep(order.sleepTime / 3)

    def outputPos(self,orderList):
        dataList = []
        count = len(orderList)
        sleepTime = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            #OutputController().msgPrint("　　id:",data.id,"pos:",data.pos)
            dataList+=[data.id,(0X10000 + int(data.pos * 100)) & 0XFF,((0X10000 + int(data.pos * 100)) // 256) & 0XFF]
            sleepTime = max(sleepTime,data.sleepTime)
   
        #       Size CMD OP ID/Data1/Data2 ADR CNT
        list = [6 + count * 3,0X04,0X00] + dataList + [0X2A,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)

    def outputVelocity(self,orderList):
        dataList = []
        count = len(orderList)
        sleepTime = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            #OutputController().msgPrint("　　id:",data.id,"pos:",data.velocity)
            dataList+=[data.id,(0X10000 + int(data.velocity * 100)) & 0XFF, ((0X10000 + int(data.velocity * 100)) // 256) & 0XFF]
            sleepTime = max(sleepTime,data.sleepTime)

        #       Size CMD OP ID/Data1/Data2 ADR CNT
        list = [6 + count * 3,0X04,0X00] + dataList + [0X30,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)


    def setMotorMode(self,orderList):
        dataList = []
        count = len(orderList)
        sleepTime = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            #OutputController().msgPrint("　　id:",data.id,"motorMode:",data.motorMode)
            dataList+=[data.id,int(data.motorMode)]
            sleepTime = max(sleepTime,data.sleepTime)

        #       Size CMD OP ID/Data1 ADR CNT
        list = [6 + count * 2,0X04,0X00] + dataList + [0X28,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)
    
    #エンコーダの初期化
    def resetEncoder(self,orderList):
        dataList = []
        count = len(orderList)
        sleepTime = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            #OutputController().msgPrint("　　id:",data.id)
            dataList+=[data.id,0x00,0x00,0x00,0x00]
            sleepTime = max(sleepTime,data.sleepTime)

        #       Size CMD OP ID/Data/Data/Data/Data ADR CNT
        list = [6 + count * 5,0X05,0X00] + dataList + [0X28,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)




    def resetMotor(self,orderList):
        dataList = []
        count = len(orderList)
        sleepTime = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            #OutputController().msgPrint("　　id:",data.id)
            dataList+=[data.id]
            sleepTime = max(sleepTime,data.sleepTime)

        #       Size CMD OP ID ADR CNT
        list = [5 + count * 1,0X05,0X00] + dataList + [0X28,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)


    #def Position_Read2(ser,ID):
    #     #       Size CMD  OP  ID ADR  Len
    #    list = [0X07,0X03,0X00,ID,0X2C,0X02]
    #    list.insert(6,sum(list)&0XFF)
    #    ser.write(list)
    #    sleep(0.003)
    #
    #def Position_Read4(list2,n):
    #  
    #    pos = (256*list2[5+7*(n-1)]+list2[4+7*(n-1)])/100
    #    return pos
    #
    #def Velocity_Read2(ser,ID):
    #     #       Size CMD  OP  ID ADR  Len
    #    list = [0X07,0X03,0X00,ID,0X32,0X02]
    #    list.insert(6,sum(list)&0XFF)
    #    ser.write(list)
    #    sleep(0.003)    
    #
    #def Velocity_Read3(ser,ID):
    #     #       Size CMD  OP  ID ADR  Len
    #    list = [0X07,0X03,0X00,ID,0X32,0X02]
    #    list.insert(6,sum(list)&0XFF)
    #    ser.write(list)     
    #    list2 = ser.read(70)
    #    sleep(0.004)
    #    return list2
    
