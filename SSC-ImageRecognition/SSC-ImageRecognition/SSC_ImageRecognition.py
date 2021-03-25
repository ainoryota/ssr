import cv2 #pip install opencv-python
import numpy as np #pip install numpy
from matplotlib import pyplot as plt
import os

path="D:\Programing\SSR\SSC-ImageRecognition\SSC-ImageRecognition\\"

img = cv2.imread(path+'Image\\box.png')
bgrLower = np.array([174-25, 119-25, 20-15])    # 抽出する色の下限(BGR)
bgrUpper = np.array([174+25, 119+25, 20+25])    # 抽出する色の上限(BGR)
img_mask = cv2.inRange(img, bgrLower, bgrUpper) # BGRからマスクを作成
img = cv2.bitwise_and(img, img, mask=img_mask) # 元画像とマスクを合成
_,img=cv2.threshold(img,0,255,cv2.THRESH_BINARY)
img = cv2.medianBlur(img,5)


cv2.imshow("img_th", img)
cv2.waitKey()
cv2.destroyAllWindows()