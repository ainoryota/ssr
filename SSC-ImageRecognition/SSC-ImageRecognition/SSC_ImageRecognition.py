import cv2 #pip install opencv-python
import numpy as np #pip install numpy
from matplotlib import pyplot as plt
import os
import math
import glob

path="D:\Programing\SSR\SSC-ImageRecognition\SSC-ImageRecognition\\"
cable_size=22
circle_diameter=82


print(glob.glob(path+'Image\\*.jpg'))
for data in glob.glob(path+'Image\\*.jpg'):
    img = cv2.imread(data)
    original_img=img
    height, width, channels = img.shape[:3]

    loop_count=50
   
    B=0
    G=0
    R=0
   
    for i in range(loop_count):
        pixelValue = img[height-i-1, math.floor(width/2)]
        B+=pixelValue[0]
        G+=pixelValue[1]
        R+=pixelValue[2]
    B/=loop_count
    G/=loop_count
    R/=loop_count
    




    print(B,G,R)
    min_rate=0.5
    max_rate=2
    color_range=40
#[174+25, 119+25, 20+25])
    min_B=max(0,B-color_range)
    min_G=max(0,G-color_range)
    min_R=max(0,R-color_range)
    max_B=min(255,B+color_range)
    max_G=min(255,G+color_range)
    max_R=min(255,R+color_range)
    

    print([min_B,min_G,min_R])
    print([max_B,max_G,max_R])
    bgrLower = np.array([min_B,min_G,min_R])    # 抽出する色の下限(BGR)
    bgrUpper = np.array([max_B,max_G,max_R])    # 抽出する色の上限(BGR)
    bgrLower = np.array([min_B,min_G,min_R])    # 抽出する色の下限(BGR)
    bgrUpper = np.array([max_B,max_G,max_R])    # 抽出する色の上限(BGR)
    img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
    img = cv2.bitwise_and(img, img, mask=img_mask) # 元画像とマスクを合成
    _,img=cv2.threshold(img,0,255,cv2.THRESH_BINARY)
    img = cv2.medianBlur(img,3)

    height, width, channels = img.shape[:3]
    black_counter=0
    start_point=0
    BLACK_RESETPOINT=50

    for i in range(height):
        pixelValue = img[i, math.floor(width/2)]
        if(pixelValue[0]==255):
            if(start_point==0):start_point=i
            black_counter=0
        else: 
            black_counter+=1

        if(black_counter>BLACK_RESETPOINT):
            start_point=0
            black_counter=0
    print(start_point)

    result_img = cv2.circle(original_img,( math.floor(width/2),start_point+math.floor(circle_diameter/2)), 5, (0,0,255), -1)


    cv2.imwrite(path+"result\\"+os.path.basename(data),result_img)
    cv2.imshow("img_th", result_img)
    cv2.waitKey()
    cv2.destroyAllWindows()