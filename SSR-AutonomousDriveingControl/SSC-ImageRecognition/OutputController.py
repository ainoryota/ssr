import time
import serial#pip install pyserial
from Order import Mode
from Order import InitOrder
from Order import PosOrder
from Order import SpeedOrder
import math

class OutputController(object):
    @classmethod
    def get_instance(self):
        if not hasattr(self, "_instance"):
            print("new singleton")
            self._instance = self()
        return self._instance

    def __init__(self):
        self.orderList = []
        self.output = Serial()

    def insertOrder(self,order):
        self.orderList.insert(0,order)
        print("Set:",order)

    def write(self,command,timer):
        print("Warn: Debug Command is used")
        self.serial.write(command)

    def done(self):
        self.orderList.sort(key=lambda x: -x.delay)

        timer = 0
        while(len(self.orderList) > 0):
            data = self.orderList.pop()
            time.sleep(max(data.delay - timer,0))
            timer = data.delay + data.sleepTime

            if(isinstance(data,InitOrder)):#初期化に関する司令は1つずつしか送れない
                print("Init",data)
                self.output.outputInit(data)

            elif(isinstance(data,PosOrder)):#位置に関する司令は複数同時に送信することができる
                dataList = [data];
                while(len(self.orderList)>0 and isinstance(self.orderList[-1],PosOrder) and self.orderList[-1].delay==data.delay):
                    dataList.append(self.orderList.pop())
                print("Pos",dataList)
                self.output.outputPos(dataList)

            elif(isinstance(data,SpeedOrder)):
                print("Speed")

            
        
class Serial(object):
    def __init__(self):
        self.serial = serial.Serial("COM4", 115200)

    def write(self,command):
        self.serial.write(command)

    def outputInit(self,order):
        if order.mode == Mode.Pos:
            data1 = 0X02 #位置制御モードかつフリー
            data2 = 0X00 #位置制御モードかつノーマル
        elif order.mode == Mode.Speed:
            data1 = 0X06 #速度制御モードかつフリー
            data2 = 0X04 #速度制御モードかつノーマル
        else:
            print("statusの値が不正")

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
            dataList+=[data.id,(0X10000 + int(data.pos * 100)) & 0XFF,((0X10000 + int(data.pos * 100)) // 256) & 0XFF]
            sleepTime = max(sleepTime,data.sleepTime)
   
        #       Size CMD OP ID/Data1/Data2 ADR CNT
        list = [6 + count * 3,0X04,0X00] + dataList + [0X2A,count]
        list.append(sum(list) & 0XFF)
        self.write(list)
        time.sleep(sleepTime)


