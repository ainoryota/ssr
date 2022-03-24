#モータをコントロールする関数
import serial
import time

def sleep(timer):
    #start=time.perf_counter()
    #print("sleep start",timer)
    time.sleep(timer)
    #print("sleep end",time.perf_counter()-start)



def Position_Read2(ser,ID):
     #       Size CMD  OP  ID ADR  Len
    list = [0X07,0X03,0X00,ID,0X2C,0X02]
    list.insert(6,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)

def Position_Read4(list2,n):
      
    pos = (256*list2[5+7*(n-1)]+list2[4+7*(n-1)])/100
    return pos
    
def Velocity_Read2(ser,ID):
     #       Size CMD  OP  ID ADR  Len
    list = [0X07,0X03,0X00,ID,0X32,0X02]
    list.insert(6,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)    

def Velocity_Read3(ser,ID):
     #       Size CMD  OP  ID ADR  Len
    list = [0X07,0X03,0X00,ID,0X32,0X02]
    list.insert(6,sum(list)&0XFF)
    ser.write(list)     
    list2 = ser.read(70)
    sleep(0.004)
    return list2

