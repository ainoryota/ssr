# -*- coding: utf-8 -*-
"""
Created on Sun May  5 16:09:23 2024

@author: kanam
"""
import numpy as np
#from matplotlib import pyplot as plt
#import matplotlib.pyplot as plt
from scipy.optimize import brentq
#from scipy.optimize import root_scalar

#import cv2
import math
#from scipy.optimize import fsolve

a = 10
b = 5
c = 50
right = 60  #仮置き

gamma1 = a*math.pi/180     #ケーブル斜度
phi_n = b*math.pi/180      #ケーブル平面の回転角
the_nR = c*math.pi/180     #右旋回角
the_nL = right*math.pi/180 #左旋回角
    
#機構定数など
Rw = 83/2      #タイヤの直径，内径
rw = 22.185/2
d = 6          #ケーブル直径
phi_nw = np.arctan(12/23.5)   #p88タイヤの形状によってケーブルを抜く角度

rm = 45/2  #基準円柱半径（モータ部分）
lm = 50  #基準円柱長さ（モータ部分）
rj = 35  #接触判定用の球の半径
    
a1 = 79/180*math.pi         #ねじれ角
a2 = 64/180*math.pi
a3 = 15/180*math.pi         #1軸目の取り付け角（論文ではa0）  
    
#角度の入力追加、1が自分側、2が二台目
theta1 = 10/180*math.pi
theta2 = 10/180*math.pi
#TN = 20;

phi_c = 0.5
OnOrOf = 0

#各リンクの重さ，重心位置，長さ
Lc = 300
l2_min = (Lc-80)/2
l1 = 90                #各モーターの中心点からの距離
l2 = l2_min/np.sin(a2)
l3 = 185
l4 = 185
l5 = l2_min/np.sin(a2)
l6 = 90
La = 215
    
w0 = 800
#w1 = 146.34;
w2 = 154
#w3 = 635+865;
w3 = 700+700
w4 = 154   
#w5 = 146.34;
w6 = 800
##wC = 100;   #カメラ重さ、とりあえず適当に入れた
    
#重心位置
g0 = np.array([
    [0],
    [0],
    [0]
])
#g1 = [0;-19.248;91.207];
g2 = np.array([
    [0],
    [0],
    [l2]
])
g3 = np.array([
    [Lc/2],
    [0],
    [La]
])      ##baseplateの重心
g4 = np.array([
    [0],
    [0],
    [l2]
])
#g5 = [0;-83.118;42.198];
g6 = np.array([
    [0],
    [0],
    [0]
])
g7 = np.array([
    [Lc/2],
    [0],
    [La]   ##張力働く場所、未確定
])
#初期値
the_xr = -90/180*np.pi
the_yr = 0/180*np.pi
the_zr = 20/180*np.pi
the_xf = -90/180*np.pi
the_yf = 0/180*np.pi
the_zf = -20/180*np.pi

the_1f = -0.619904961
the_2f = 1.969332926	
the_3f = -0.980051317	
the_1r = 0.553278267	
the_2r = -1.874767008	
the_3r = 0.923916082

phi_nr1 = 0/180*np.pi
the_nr = 0/180*np.pi
phi_nr2 = 0/180*np.pi
phi_nf1 = 0/180*np.pi
the_nf = 0/180*np.pi
phi_nf2 = 0/180*np.pi

Ln1 = 450

#x = 0.1    #仮置き,実際は計測する部分
T = 1000         #仮置き
#Base0 = -0.3

mode = 1

#def MAIN():
    #リアルセンス
    
    
#    the_1f, the_2f, the_3f, the_1r, the_2r, the_3r = main()
    #モーター
    

def position(Base0):

    #XA = brentq(FUN, 0, np.pi/2)
    #print(XA)
# 配列の形状を確認
    #print(f"w0_ の形状: {(OrOf() / np.linalg.norm(OrOf())).flatten().shape}")
    try:
        beta_solution = brentq(FUN, 0, np.pi/2)
    except Exception as e:
        the_1f = -0.619904961
        the_2f = 1.969332926	
        the_3f = -0.980051317	
        the_1r = 0.553278267	
        the_2r = -1.874767008	
        the_3r = 0.923916082
        #print(the_1f, the_2f, the_3f, the_1r, the_2r, the_3r)
        P = np.array([the_1f, the_2f, the_3f, the_1r, the_2r, the_3r])
        return   P
    else:
        newX = brentq(Theta, -np.pi/4, np.pi/4)
        #print(newX)
        the_1f, the_2f, the_3f, the_1r, the_2r, the_3r = THE(newX)
        #print(the_1f, the_2f, the_3f, the_1r, the_2r, the_3r)      
        P = np.array([the_1f, the_2f, the_3f, the_1r, the_2r, the_3r])
        return   P    #自走機のアーム角度を返す

def FUN(beta):
    #ワイヤー角度βを求める関数
        
    # 重心計算（the_xの）決定

    #Or基準,ケーブル座標系(後ろ駆動輪とケーブルの接触点)
    #少しトリッキー%なことしている(最終的にはちゃんとした座標系で計算を行うのが望ましいが，傾向は把握のため，各リンクの重心は，対偶位置に一致すると仮定)
    #(リンク１の重量はリンク0に，リンク5の重量は,リンク6に含まれると仮定する)
    #(すなわちリンク3以外は，重さの成分はｚ成分しかない)
    ##m3がBasePlate
    ##g7の位置に張力が働くとする 
    
    TN = T*np.cos(beta)
    TNX = TN*np.sin(theta1)*np.sin(gamma1)
    TNY = TN*np.cos(theta1)
    TNZ = TN*np.sin(theta1)*np.cos(gamma1)
    
    w0_ = np.array([
        [-w0*np.sin(gamma1)],
        [-w0*np.cos(gamma1)*np.sin(Base0)],
        [w0*np.cos(gamma1)*np.cos(Base0)]
    ])
    
    w2_ = np.array([
        [-w2*np.sin(gamma1)],
        [-w2*np.cos(gamma1)*np.sin(Base0)],
        [w2*np.cos(gamma1)*np.cos(Base0)]
    ])
    
    w3_ = np.array([
        [-w3*np.sin(gamma1)],
        [-w3*np.cos(gamma1)*np.sin(Base0)],
        [w3*np.cos(gamma1)*np.cos(Base0)]
    ])
    
    w4_ = np.array([
        [-w4*np.sin(gamma1)],
        [-w4*np.cos(gamma1)*np.sin(Base0)],
        [w4*np.cos(gamma1)*np.cos(Base0)]
    ])
    
    w6_ = np.array([
        [-w6*np.sin(gamma1)],
        [-w6*np.cos(gamma1)*np.sin(Base0)],
        [w6*np.cos(gamma1)*np.cos(Base0)]
    ])
    
    wT = np.array([
        [TNX],
        [TNY*np.cos(Base0)-TNZ*np.sin(Base0)],
        [-TNY*np.sin(Base0)+TNZ*np.cos(Base0)]
    ])

    m0 = np.cross((OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@g0).T,w0_.T)
    m2 = np.cross((OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@g2).T,w2_.T)
    m3 = np.cross((OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",a3)@g3).T,w3_.T)
    m4 = np.cross((OrCf() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",2*a3)@R("z",-np.pi/2)@R("z",the_1f)@R("x",a1)@g4).T,w4_.T)
    m6 = np.cross((OrCf() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",2*a3)@R("z",-np.pi/2)@R("z",the_1f)@R("x",a1)@R("z",the_2f)@R("x",-a2)@R("z",the_3f + np.pi/2)@g6).T,w6_.T)
    mT = np.cross((OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",a3)@g7).T,wT.T)
    
    OrOf_flat = (OrOf() / np.linalg.norm(OrOf())).flatten()
    m0_flat = m0.flatten()
    m2_flat = m2.flatten()
    m3_flat = m3.flatten()
    m4_flat = m4.flatten()
    m6_flat = m6.flatten()
    mT_flat = mT.flatten()
    #評価関数（モーメント）
    return np.dot(m0_flat, OrOf_flat) + np.dot(m2_flat, OrOf_flat) + np.dot(m3_flat, OrOf_flat) + np.dot(m4_flat, OrOf_flat) + np.dot(m6_flat, OrOf_flat) + np.dot(mT_flat, OrOf_flat)
#p = OrOf/norm(OrOf);
#F = 2*dot(m0,p)+2*dot(m1,p)+2*dot(m2,p)+2*dot(m3,p)+dot(mx,p)+dot(mT,p);

def Theta(x):
    beta = brentq(FUN, 0, np.pi/2)
    
    M_0f_ = M_0f(x)
    M_0r_ = M_0r(x)
    #逆運動学解（後輪）
    if mode == 1:
        the_2r = -np.arccos((-np.cos(a1)*np.cos(a2)+M_0r_[2,2])/(np.sin(a1)*np.sin(a2)))
    else:
        the_2r =  np.arccos((-np.cos(a1)*np.cos(a2)+M_0r_[2,2])/(np.sin(a1)*np.sin(a2)))
                
    #逆運動学解（前輪）
    if mode == 1:
        the_2f =  np.arccos((-np.cos(a1)*np.cos(a2)+M_0f_[2,2])/(np.sin(a1)*np.sin(a2)))
    else:
        the_2f = -np.arccos((-np.cos(a1)*np.cos(a2)+M_0f_[2,2])/(np.sin(a1)*np.sin(a2)))
    

        
    Af1 = -np.sin(a2)*np.sin(the_2f)
    Bf1 = np.cos(a1)*np.sin(a2)*np.cos(the_2f) - np.sin(a1)*np.cos(a2)
    Cf1 = M_0f_[0,2]
    Df1 = M_0f_[1,2]
    Af3 = -np.sin(a1)*np.sin(the_2f)
    Bf3 = np.sin(a1)*np.cos(a2)*np.cos(the_2f) - np.cos(a1)*np.sin(a2)
    Cf3 = M_0f_[2,1]
    Df3 = M_0f_[2,0]
        
    the_1f = np.arctan2(Af1*Df1-Bf1*Cf1,Af1*Cf1+Bf1*Df1)
    the_3f = np.arctan2(Af3*Df3-Bf3*Cf3,Af3*Cf3+Bf3*Df3)
    
    Ar1 = -np.sin(a2)*np.sin(the_2r)
    Br1 =  np.cos(a1)*np.sin(a2)*np.cos(the_2r) - np.sin(a1)*np.cos(a2)
    Cr1 = M_0r_[0,2]
    Dr1 = M_0r_[1,2]
    Ar3 =  np.sin(a1)*np.sin(the_2r)
    Br3 = -np.sin(a1)*np.cos(a2)*np.cos(the_2r) + np.cos(a1)*np.sin(a2)
    Cr3 = M_0r_[2,1]
    Dr3 = M_0r_[2,0]
    
    the_1r = np.arctan2(Ar1*Dr1-Br1*Cr1,Ar1*Cr1+Br1*Dr1)
    the_3r = np.arctan2(Ar3*Dr3-Br3*Cr3,Ar3*Cr3+Br3*Dr3)
        
    ## 重心計算（the_xの）決定

    #Or基準,ケーブル座標系(後ろ駆動輪とケーブルの接触点)
    #少しトリッキー%なことしている(最終的にはちゃんとした座標系で計算を行うのが望ましいが，傾向は把握のため，各リンクの重心は，対偶位置に一致すると仮定)
    #(リンク１の重量はリンク0に，リンク5の重量は,リンク6に含まれると仮定する)
    #(すなわちリンク3以外は，重さの成分はｚ成分しかない)
    ##m3がBasePlate
    ##g7の位置に張力が働くとする
    OrCrnorm = np.linalg.norm(OrCr())
    Base0 = np.arctan(La*np.sin(x)/(OrCrnorm+La*np.cos(x)))
    
    TN = T*np.cos(beta)
    TNX = TN*np.sin(theta1)*np.sin(gamma1)
    TNY = TN*np.cos(theta1)
    TNZ = TN*np.sin(theta1)*np.cos(gamma1)
    
    #転置した状態で書いている
    w0_ = np.array([-w0*np.sin(gamma1),-w0*np.cos(gamma1)*np.sin(Base0),w0*np.cos(gamma1)*np.cos(Base0)])
    w2_ = np.array([-w2*np.sin(gamma1),-w2*np.cos(gamma1)*np.sin(Base0),w2*np.cos(gamma1)*np.cos(Base0)]) 
    w3_ = np.array([-w3*np.sin(gamma1),-w3*np.cos(gamma1)*np.sin(Base0),w3*np.cos(gamma1)*np.cos(Base0)])    
    w4_ = np.array([-w4*np.sin(gamma1),-w4*np.cos(gamma1)*np.sin(Base0),w4*np.cos(gamma1)*np.cos(Base0)]) 
    w6_ = np.array([-w6*np.sin(gamma1),-w6*np.cos(gamma1)*np.sin(Base0),w6*np.cos(gamma1)*np.cos(Base0)])    
    wT = np.array([TNX,TNY*np.cos(Base0)-TNZ*np.sin(Base0),-TNY*np.sin(Base0)+TNZ*np.cos(Base0)])
    
    m0_ = (OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@g0).flatten()
    m2_ = (OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@g2).flatten()
    m3_ = (OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",a3)@g3).flatten()
    m4_ = (OrCf() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",2*a3)@R("z",-np.pi/2)@R("z",the_1f)@R("x",a1)@g4).flatten()
    m6_ = (OrCf() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",2*a3)@R("z",-np.pi/2)@R("z",the_1f)@R("x",a1)@R("z",the_2f)@R("x",-a2)@R("z",the_3f + np.pi/2)@g6).flatten()
    mT_ = (OrCr() + R("x",the_xr)@R("y",the_yr)@R("z",the_zr)@R("z",-the_3r+np.pi/2)@R("x",a2)@R("z",-the_2r)@R("x",-a1)@R("z",-the_1r)@R("z",-np.pi/2)@R("y",a3)@g7).flatten()

    m0 = np.cross(m0_,w0_)
    m2 = np.cross(m2_,w2_)
    m3 = np.cross(m3_,w3_)
    m4 = np.cross(m4_,w4_)
    m6 = np.cross(m6_,w6_)
    mT = np.cross(mT_,wT)
    
    OrOf_flat = (OrOf() / np.linalg.norm(OrOf())).flatten()
    #評価関数（モーメント）
    return np.dot(m0, OrOf_flat) + np.dot(m2, OrOf_flat) + np.dot(m3, OrOf_flat) + np.dot(m4, OrOf_flat) + np.dot(m6, OrOf_flat) + np.dot(mT, OrOf_flat)

def THE(x):
 
    M_0f_ = M_0f(x)
    M_0r_ = M_0r(x)
    #逆運動学解（後輪）
    if mode == 1:
        the_2r = -np.arccos((-np.cos(a1)*np.cos(a2)+M_0r_[2,2])/(np.sin(a1)*np.sin(a2))) #後輪
    else:
        the_2r =  np.arccos((-np.cos(a1)*np.cos(a2)+M_0r_[2,2])/(np.sin(a1)*np.sin(a2)))
              
    #逆運動学解（前輪）
    if mode == 1:
        the_2f =  np.arccos((-np.cos(a1)*np.cos(a2)+M_0f_[2,2])/(np.sin(a1)*np.sin(a2)))
    else:
        the_2f = -np.arccos((-np.cos(a1)*np.cos(a2)+M_0f_[2,2])/(np.sin(a1)*np.sin(a2)))


    Af1 = -np.sin(a2)*np.sin(the_2f)
    Bf1 = np.cos(a1)*np.sin(a2)*np.cos(the_2f) - np.sin(a1)*np.cos(a2)
    Cf1 = M_0f_[0,2]
    Df1 = M_0f_[1,2]
    Af3 = -np.sin(a1)*np.sin(the_2f)
    Bf3 = np.sin(a1)*np.cos(a2)*np.cos(the_2f) - np.cos(a1)*np.sin(a2)
    Cf3 = M_0f_[2,1]
    Df3 = M_0f_[2,0]
        
    the_1f = np.arctan2(Af1*Df1-Bf1*Cf1,Af1*Cf1+Bf1*Df1)
    the_3f = np.arctan2(Af3*Df3-Bf3*Cf3,Af3*Cf3+Bf3*Df3)
    
    Ar1 = -np.sin(a2)*np.sin(the_2r)
    Br1 =  np.cos(a1)*np.sin(a2)*np.cos(the_2r) - np.sin(a1)*np.cos(a2)
    Cr1 = M_0r_[0,2]
    Dr1 = M_0r_[1,2]
    Ar3 =  np.sin(a1)*np.sin(the_2r)
    Br3 = -np.sin(a1)*np.cos(a2)*np.cos(the_2r) + np.cos(a1)*np.sin(a2)
    Cr3 = M_0r_[2,1]
    Dr3 = M_0r_[2,0]        
    
    the_1r = np.arctan2(Ar1*Dr1-Br1*Cr1,Ar1*Cr1+Br1*Dr1)
    the_3r = np.arctan2(Ar3*Dr3-Br3*Cr3,Ar3*Cr3+Br3*Dr3)

    return  the_1f, the_2f, the_3f, the_1r, the_2r, the_3r   

    ## 回転行列
def R(i, j):
    if i == "x":
        return np.array([[1, 0, 0], 
                         [0, np.cos(j), -np.sin(j)], 
                         [0, np.sin(j), np.cos(j)]])
    elif i == "y":
        return np.array([[np.cos(j), 0, np.sin(j)],
                         [0, 1, 0],
                         [-np.sin(j), 0, np.cos(j)]])
    elif i == "z":
        return np.array([[np.cos(j), -np.sin(j), 0],
                         [np.sin(j), np.cos(j), 0], 
                         [0, 0, 1]])

# Rod_R(n1, n2, n3, the) の関数
def Rod_R(n1, n2, n3, the):
    cos_the = np.cos(the)
    sin_the = np.sin(the)
    one_minus_cos_the = 1 - cos_the

    # 回転行列の各要素を計算
    X = np.array([
        [cos_the + one_minus_cos_the * n1**2, n1 * n2 * one_minus_cos_the - n3 * sin_the, n1 * n3 * one_minus_cos_the + n2 * sin_the],
        [n3 * sin_the + n2 * n1 * one_minus_cos_the, cos_the + one_minus_cos_the * n2**2, n2 * n3 * one_minus_cos_the - n1 * sin_the],
        [-n2 * sin_the + n3 * n1 * one_minus_cos_the, n3 * n2 * one_minus_cos_the + n1 * sin_the, cos_the + one_minus_cos_the * n3**2]
    ])

    return X

    #リンク0の座標系から駆動部（リンク3の座標系）への座標変換行列
def M_0r(x):  #Co_0r\R("x",the_xr)*R("y",the_yr)*R("z",the_zr)
    Rotation_r = R("x",the_xr) @ R("y",the_yr) @ R("z",the_zr)
    M_0r = np.linalg.lstsq(Co_0r(x), Rotation_r, rcond=None)[0]
    return M_0r
              
def M_0f(x):  #Co_0f\R("x",the_xf)*R("y",the_yf)*R("z",the_zf)   
    Rotation_f = R("x",the_xf) @ R("y",the_yf) @ R("z",the_zf)
    M_0f = np.linalg.lstsq(Co_0f(x), Rotation_f, rcond=None)[0]   
    return M_0f

    #後輪
def OrCr():
    OrCrM = np.array([
        [Rw*np.sin(the_zr)],
        [d/2+rw-Rw*np.cos(the_zr)],
        [0]
    ])
    OrCr = R("x",the_xr) @ R("y",the_yr) @ OrCrM
    return OrCr

def OrOw1r():
    OrOw1rM = np.array([
        [0],
        [d/2+rw],
        [0]
    ])
    OrOw1r = R("x",the_xr) @ R("y",the_yr) @ OrOw1rM
    return OrOw1r
    
def OrOw2r():
    OrOw2rM = np.array([
        [2*Rw*np.sin(the_zr)],
        [d/2+rw-2*Rw*np.cos(the_zr)],
        [0]
    ])
    OrOw2r = R("x",the_xr) @ R("y",the_yr) @ OrOw2rM
    return OrOw2r

    #前輪
def OfCf():
    OfCfM = np.array([
        [Rw*np.sin(the_zf)],
        [d/2+rw-Rw*np.cos(the_zf)],
        [0]
    ])
    OfCf =   R("x",the_xf) @ R("y",the_yf) @ OfCfM
    return OfCf
    
def OfOw1f():
    OfOw1fM = np.array([
        [0],
        [d/2+rw],
        [0]
    ])
    OfOw1f = R("x",the_xf) @ R("y",the_yf) @ OfOw1fM
    return OfOw1f

def OfOw2f():
    OfOw2fM = np.array([
        [2*Rw*np.sin(the_zf)],
        [d/2+rw-2*Rw*np.cos(the_zf)],
        [0]
    ])
    OfOw2f = R("x",the_xf) @ R("y",the_yf) @ OfOw2fM
    return OfOw2f
    
    #OrOf間の計算
def R1():
    R1 = R("x",phi_nr1) @ R("z",the_nr)
    return R1
def R2():
    R2 = R("x",phi_nf1) @ R("z",the_nf)
    return R2

def Ln2(): 
    OfCf_ = OfCf()
    OrCr_ = OrCr()
    
    x_ = OfCf_[0]-OrCr_[0]
    y_ = OfCf_[1]-OrCr_[1]
    z_ = OfCf_[2]-OrCr_[2]
    
    ap = [1, 0, 0] @ R1() @ [[1],[0],[0]]
    bp = [0, 1, 0] @ R1() @ [[1],[0],[0]]
    cp = [0, 0, 1] @ R1() @ [[1],[0],[0]]
    dp = [1, 0, 0] @ R2() @ [[1],[0],[0]]
    ep = [0, 1, 0] @ R2() @ [[1],[0],[0]]
    fp = [0, 0, 1] @ R2() @ [[1],[0],[0]]
    
    A = ap[0]**2+bp[0]**2+cp[0]**2
    B = dp[0]**2+ep[0]**2+fp[0]**2
    C = x_**2+y_**2+z_**2
    D = ap[0]*x_+bp[0]*y_+cp[0]*z_
    E = dp[0]*x_+ep[0]*y_+fp[0]*z_
    F = ap@dp+bp@ep+cp@fp
    
    Ln2 = (-(E+F*Ln1)+np.sqrt((E+F*Ln1)**2-B*(A*Ln1*Ln1+2*D*Ln1+C-Lc**2)))/B
    return Ln2[0]

def OrOf():
    vec1 = np.array([
        [Ln1],
        [0],
        [0]
    ])
    Ln2_ = Ln2()
    vec2 = np.array([
        [Ln2_],
        [0],
        [0]
    ])
    OrOf = R1() @ vec1 + R2() @ vec2
    return OrOf

   ## リンク3から見た駆動部の姿勢角の導出(p23~)
def OrCf():
   OrCf = OrOf()+OfCf()            #後ろ駆動輪の接地点から前駆動輪の原点に向けたベクトル
   return OrCf
def CrCf():
   CrCf = OrCf() - OrCr()       #後ろ駆動輪の原点から前駆動輪の原点に向けたベクトル
   return CrCf

def Co_base(x):
    xb0 = CrCf()/Lc
    yb0M = np.array([
        [-np.sin(gamma1)],
        [0],
        [np.cos(gamma1)]
    ])
    yb0 = (np.cross(yb0M.T,xb0.T).T)/np.linalg.norm(np.cross(yb0M.T,xb0.T).T)
    zb0 = np.cross(xb0.T,yb0.T).T 
    Co_base0 = [
        [xb0[0,0],yb0[0,0],zb0[0,0]],    #base座標系への座標変換行列
        [xb0[1,0],yb0[1,0],zb0[1,0]],
        [xb0[2,0],yb0[2,0],zb0[2,0]]
    ]
    Co_base = Rod_R(xb0[0,0],xb0[1,0],xb0[2,0],x)@Co_base0  #xb0軸周りにｘだけ回転させ，base座標系への座標変換行列を求める．
    return Co_base

def Co_0r(x):
    Co_ = Co_base(x)
    xb = Co_[:,0]
    yb = Co_[:,1]
    zb = Co_[:,2]  
    xbybzb = np.column_stack((xb, yb, zb))
    Co_0r_ = Rod_R(yb[0],yb[1],yb[2],-a3) @ xbybzb
    Co_0r = Rod_R(Co_0r_[0,2],Co_0r_[1,2],Co_0r_[2,2],np.pi/2)@Co_0r_       #可動範囲を考慮した座標系の設定
    return Co_0r
def Co_0f(x):
    Co_ = Co_base(x)
    xb = Co_[:,0]
    yb = Co_[:,1]
    zb = Co_[:,2] 
    xbybzb = np.column_stack((xb, yb, zb))
    Co_0f_ = Rod_R(yb[0],yb[1],yb[2],a3)@xbybzb
    Co_0f = Rod_R(Co_0f_[0,2],Co_0f_[1,2],Co_0f_[2,2],-np.pi/2)@Co_0f_
    return Co_0f



#if __name__ == "__main__":
#    main()
