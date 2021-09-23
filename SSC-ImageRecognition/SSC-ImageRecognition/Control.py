#モータをコントロールする関数
import serial
import time

def sleep(timer):
    #start=time.perf_counter()
    #print("sleep start",timer)
    time.sleep(timer)
    #print("sleep end",time.perf_counter()-start)


#モータの制御モード変更（とりあえず位置と速度のみ)軌道生成もとりあえずなし。後で要比較
def Motor_Init(ser,ID,status):
    if status == 0:
        data1 = 0X02 #位置制御モードかつフリー
        data2 = 0X00 #位置制御モードかつノーマル
    elif status == 1:
        data1 = 0X06 #速度制御モードかつフリー
        data2 = 0X04 #速度制御モードかつノーマル
    else:
        print("statusの値が不正")
    
    #        Size CMD   OP     Data ADR  CNT
    list1 = [0X08,0X04,0X00,ID,data1,0X28,0X01]#制御モードの変更
    list2 = [0X08,0X04,0X00,ID,0X00,0X5C,0X01]#ゲインを変更
    list3 = [0X08,0X04,0X00,ID,data2,0X28,0X01]#ノーマルモードに切り替え
        
    list1.insert(7,sum(list1))
    list2.insert(7,sum(list2))
    list3.insert(7,sum(list3))
        
    ser.write(list1)
    sleep(0.1)
    ser.write(list2)
    sleep(0.1)
    ser.write(list3)
    sleep(0.1)
        
        
#1つのモータに位置の指令を出す
def Motor_pos(ser,ID1,the1):
    #       Size CMD   OP  ID     Data1       Data2        ADR  CNT
    list = [0X09,0X04,0X00,ID1,the1*100%256,the1*100//256,0X2A,0X01]  
    list.insert(8,sum(list)&0XFF)
    print(list)
    ser.write(list)
    sleep(0.1)
    
    
#３つのモータに位置の指令を出す(最終的には6個)
def Motor_pos3(ser,ID1,ID2,ID3,the1,the2,the3):

    data11=(0X10000+int(the1*100))&0XFF   #roundだと.0になって
    data12=((0X10000+int(the1*100))//256)&0XFF
    data21=(0X10000+int(the2*100))&0XFF
    data22=((0X10000+int(the2*100))//256)&0XFF
    data31=(0X10000+int(the3*100))&0XFF
    data32=((0X10000+int(the3*100))//256)&0XFF
    
    #       Size CMD   OP  ID   Data1  Data2  ID  Data1  Data2  ID  Data1  Data2  ADR  CNT
    list = [0X0F,0X04,0X00,ID1,data11,data12,ID2,data21,data22,ID3,data31,data32,0X2A,0X03]
    list.insert(14,sum(list)&0XFF)
    ser.write(list)
    sleep(0.02)
    
#6つのモータに位置の指令を出す
def Motor_pos6(ser,ID1,ID2,ID3,ID6,ID7,ID8,the1,the2,the3,the6,the7,the8):

    data11=(0X10000+int(the1*100))&0XFF   #roundだと.0になって
    data12=((0X10000+int(the1*100))//256)&0XFF
    data21=(0X10000+int(the2*100))&0XFF
    data22=((0X10000+int(the2*100))//256)&0XFF
    data31=(0X10000+int(the3*100))&0XFF
    data32=((0X10000+int(the3*100))//256)&0XFF
    data61=(0X10000+int(the6*100))&0XFF   #roundだと.0になって
    data62=((0X10000+int(the6*100))//256)&0XFF
    data71=(0X10000+int(the7*100))&0XFF
    data72=((0X10000+int(the7*100))//256)&0XFF
    data81=(0X10000+int(the8*100))&0XFF
    data82=((0X10000+int(the8*100))//256)&0XFF
    
    #       Size CMD   OP  ID   Data1  Data2  ID  Data1  Data2  ID  Data1  Data2 ID   Data1  Data2  ID  Data1  Data2  ID  Data1  Data2  ADR  CNT
    list = [0X18,0X04,0X00,ID1,data11,data12,ID2,data21,data22,ID3,data31,data32,ID6,data61,data62,ID7,data71,data72,ID8,data81,data82,0X2A,0X06]
    list.insert(23,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)
    
#1つのモータに速度の指令を出す
def Motor_vel(ser,ID1,w1):
    data11 = (0X10000+int(w1*100))&0XFF
    data12 = ((0X10000+int(w1*100))//256)&0XFF
    
    #       Size CMD   OP  ID    Data1 Data2   ADR  CNT
    list = [0X09,0X04,0X00,ID1,data11,data12,0X30,0X01]  
    list.insert(8,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)

#2つのモータに角速度の指令を出す
def Motor_vel2(ser,ID1,ID2,w1,w2):
    data11 = (0X10000+int(w1*100))&0XFF
    data12 = ((0X10000+int(w1*100))//256)&0XFF
    data21 = (0X10000+int(w2*100))&0XFF
    data22 = ((0X10000+int(w2*100))//256)&0XFF
    
    #       Size CMD   OP  ID  Data1  Data2  ID  Data1  Data2  ADR  CNT
    list = [0X0C,0X04,0X00,ID1,data11,data12,ID2,data21,data22,0X30,0X02]
    list.insert(11,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)


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


#角度の読み取り
def Position_Read(ser,ID):
     #       Size CMD  OP  ID ADR  Len
    list = [0X07,0X03,0X00,ID,0X2C,0X02]
    list.insert(6,sum(list)&0XFF)
    
    ser.flushInput()#バッファのクリア
    ser.write(list)     
    list2 = ser.read(0X07)
    pos = (256*list2[5]+list2[4])/100
    return pos
    
    
    
#角速度の読み取り
def Velocity_Read(ser,ID):
     #       Size CMD  OP  ID ADR  Len
    list = [0X07,0X03,0X00,ID,0X32,0X02]
    list.insert(6,sum(list)&0XFF)
    ser.flushInput()#バッファのクリア
    ser.write(list)
    list2 = ser.read(0X07)
    vel = (256*list2[5]+list2[4])/100
    return vel


def Position_Read2(ser,ID):
     #       Size CMD  OP  ID ADR  Len
    list = [0X07,0X03,0X00,ID,0X2C,0X02]
    list.insert(6,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)
    
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

def Position_Read4(list2,n):
      
    pos = (256*list2[5+7*(n-1)]+list2[4+7*(n-1)])/100
    return pos

def Motor_Reset(ser,ID1,ID2,ID3,ID4):
    #       Size CMD  OP  ID                Time 
    list = [0X09,0X05,0X00,ID1,ID2,ID3,ID4,0X01]
    list.insert(8,sum(list)&0XFF)
    ser.write(list)
    sleep(0.003)
    