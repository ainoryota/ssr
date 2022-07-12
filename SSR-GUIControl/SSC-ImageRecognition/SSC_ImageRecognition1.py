import cv2 #pip install opencv-python
import numpy as np #pip install numpy
from matplotlib import pyplot as plt
import os
import math
import glob
import time
from PIL import Image,ImageTk


def ImageReconition(original_img,count):
    cable_size=22
    circle_diameter=82

    #kernel = np.ones((7,7),np.float32)/25
    #img = cv2.filter2D(img,-1,kernel)
    #img=cv2.resize(img,dsize=(100,400))


    
    height, width, channels = original_img.shape[:3]

    loop_count=50

    checkW=[]
    for i in range(-5,6,1):
        checkW.append(math.floor(width*(0.50+0.025*i)));


    maxX=0;
    maxStartPoint=0;
    maxValue=0;

    count=0
    for x in checkW:
        count=count+1
        img=original_img
        R=0
        G=0
        B=0


        for i in range(loop_count):
            pixelValue = img[height-i-1, x]
            R+=pixelValue[0]
            G+=pixelValue[1]
            B+=pixelValue[2]

        value=100*B/(B+G+R)
        print(count,R,G,B,value)
        B/=loop_count
        G/=loop_count
        R/=loop_count
        if(value>maxValue):
            maxValue=value;
            maxX=x;
    
    x=maxX
    min_rate=0.5
    max_rate=2
    color_range=40

    min_R=float(max(0,R-color_range))
    min_G=float(max(0,G-color_range))
    min_B=float(max(0,B-color_range))
    max_R=float(min(255,R+color_range))
    max_G=float(min(255,G+color_range))
    max_B=float(min(255,B+color_range))

    bgrLower = np.array([min_R,min_G,min_B])    # 抽出する色の下限(BGR)
    bgrUpper = np.array([max_R,max_G,max_B])    # 抽出する色の上限(BGR)
    img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
    img = cv2.bitwise_and(img, img, mask=img_mask) # 元画像とマスクを合成
    _,img=cv2.threshold(img,0,255,cv2.THRESH_BINARY)
    img = cv2.medianBlur(img,3)

    height, width, channels = img.shape[:3]
    black_counter=0
    start_point=0
    BLACK_RESETPOINT=100

    for i in range(height):
        pixelValue = img[i, x]
        if(pixelValue[2]==255):
            if(start_point==0):start_point=i
            black_counter=0
        else: 
            black_counter+=1

        if(black_counter>BLACK_RESETPOINT):
            start_point=0
            black_counter=0
        
    print(x,start_point);
    maxSmaxStartPoint=start_point-BLACK_RESETPOINT;


    #img=analyze_img

    result_img=original_img
    
    for x in checkW:
        result_img = cv2.line(result_img,( x,0),( x,height),(0,0,0),1);
    result_img = cv2.line(result_img,( maxX,0),( maxX,height),(255,0,0),2);
    if(maxSmaxStartPoint>circle_diameter and maxValue>40): result_img = cv2.circle(original_img,( maxX,maxSmaxStartPoint+math.floor(circle_diameter/2)), 10, (255,0,0), -1)

    #cv2.imwrite("result/"+str(count)+".png",result_img)
    return result_img
    #cv2.imshow("img", result_img)
    #cv2.waitKey()
    #cv2.destroyAllWindows()

#test=np.array(Image.open('Image/SSC1.png'))
#test2=cv2.cvtColor(test, cv2.COLOR_BGR2RGB)
#ImageReconition(test2,4)