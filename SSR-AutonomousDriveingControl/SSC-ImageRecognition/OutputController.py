import time
import serial#pip install pyserial
from Order import Mode
from Order import InitOrder
from Order import PosOrder
from Order import VelocityOrder
import math
import copy
import threading
import queue
from multiprocessing import Process
import platform

import threading

class OutputController(object):

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                print("New Singleton is OutputController")
                cls._instance = super().__new__(cls)
                cls.orderList = []
                cls.stepQueue = queue.Queue()
        return cls._instance

    def setStepQueue(self,stepQueue):
        self.stepQueue = stepQueue

    def insertOrder(self,order):
        self.orderList.insert(0,order)
        print("Order Set:",order)

    def write(self,command,timer):
        print("Warn: Debug Command is used")
        self.serial.write(command)

    def pushStep(self):
        self.stepQueue.put(sorted(self.orderList,key=lambda x: -x.delay - (0.001 if isinstance(x,PosOrder) else 0.002 if  isinstance(x,VelocityOrder) else 0)))
        self.orderList.clear()



def OutputDone(stepQueue):
    output = Serial()

    while(True):
        if(stepQueue.empty()):
            time.sleep(0.1)
            continue
        orderList = stepQueue.get()
        print("Start Output:",len(orderList),"datas")


        timer = 0
        while(len(orderList) > 0):
            data = orderList.pop()
            time.sleep(max(data.delay - timer,0))
            timer = data.delay + data.sleepTime

            if(isinstance(data,InitOrder)):#初期化に関する司令は1つずつしか送れない
                print("　Init",data)
                output.outputInit(data)

            elif(isinstance(data,PosOrder)):#位置に関する司令は複数同時に送信することができる
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],PosOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                print("　Move Pos:",len(dataList),"motors")
                output.outputPos(dataList)

            elif(isinstance(data,VelocityOrder)):#速度に関する司令は複数同時に送信することができる
                dataList = [data]
                while(len(orderList) > 0 and isinstance(orderList[-1],VelocityOrder) and orderList[-1].delay == data.delay):
                    dataList.append(orderList.pop())
                print("　Move Velocity:",len(dataList),"motors")
                output.outputVelocity(dataList)
        print("End Output")
                                                

        
class Serial(object):
    def __init__(self):
        print("Open Serial Port")
        if(platform.system() != "Windows"):
            self.serial = serial.Serial('/dev/ttyUSB0',115200)    
        else:
            self.serial = serial.Serial("COM4", 115200)

    def write(self,command):
        self.serial.write(command)

    def outputInit(self,order):
        if order.mode == Mode.Pos:
            data1 = 0X02 #位置制御モードかつフリー
            data2 = 0X00 #位置制御モードかつノーマル
        elif order.mode == Mode.Velocity:
            data1 = 0X06 #速度制御モードかつフリー
            data2 = 0X04 #速度制御モードかつノーマル
        else:
            print("statusの値が不正")

        print("　　id:",order.id,"mode",order.mode)

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
            print("　　id:",data.id,"pos:",data.pos)
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
            print("　　id:",data.id,"pos:",data.velocity)
            dataList+=[data.id,(0X10000 + int(data.velocity * 100)) & 0XFF, ((0X10000 + int(data.velocity * 100)) // 256) & 0XFF]
            sleepTime = max(sleepTime,data.sleepTime)

        #       Size CMD OP ID/Data1/Data2 ADR CNT
        list = [6 + count * 3,0X04,0X00] + dataList + [0X30,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)
