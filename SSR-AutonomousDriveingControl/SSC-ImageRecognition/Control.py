#モータをコントロールする関数
import serial
import time

def sleep(timer):
    #start=time.perf_counter()
    #print("sleep start",timer)
    time.sleep(timer)
    #print("sleep end",time.perf_counter()-start)

#2つのモータをノーマルモードにする（駆動部）
def Motor_vel_normal2(ser,ID1,ID2,w1,w2):
    #       Size CMD   OP  ID  Data ID  Data ADR  CNT
    list = [0X0A,0X04,0X00,ID1,0X04,ID2,0X04,0X28,0X02]#速度モードかつノーマル
    list.insert(9,sum(list)&0XFF)
    ser.write(list)
    sleep(0.1)

#モータをホールドモードにする（駆動部）
def Motor_vel_hold(ser,ID1):
    #       Size CMD   OP  ID  Data ADR  CNT
    list = [0X08,0X04,0X00,ID1,0X07,0X28,0X01]#速度モードかつホールド
    list.insert(8,sum(list)&0XFF)
    ser.write(list)
    sleep(0.1)
    
#2つのモータをホールドモードにする（駆動部）
def Motor_vel_hold2(ser,ID1,ID2,w1,w2):
    #       Size CMD   OP  ID  Data ID  Data ADR  CNT
    list = [0X0A,0X04,0X00,ID1,0X07,ID2,0X07,0X28,0X02]#速度モードかつホールド
    list.insert(9,sum(list)&0XFF)
    ser.write(list)
    sleep(0.1)
    
#モータをフリーモードに
def Motor_Free(ser,ID):
    #       Size CMD   OP     Data ADR  CNT
    list = [0X08,0X04,0X00,ID,0X02,0X28,0X01]
    list.insert(7,sum(list)&0XFF)
    ser.write(list)
    sleep(0.1)

#エンコーダの初期化
def Enc_Init(ser,ID):
     #       Size CMD  OP  ID  Data Data Data Data ADR  CNT
    list = [0X0B,0X04,0X00,ID,0X00,0X00,0X00,0X00,0X52,0X01]
    list.insert(10,sum(list)&0XFF)
    ser.write(list)
    sleep(0.1)




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



def Motor_Reset(ser,ID1,ID2,ID3,ID4):
    #       Size CMD  OP  ID                Time 
    list = [0X09,0X05,0X00,ID1,ID2,ID3,ID4,0X01]
    list.insert(8,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)
    