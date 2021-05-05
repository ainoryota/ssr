#! /usr/bin/python3
# -*- coding: utf-8 -*-
#
#   usb_camera.py
#
#                       Oct/17/2020
# --------------------------------------------------------------------
#sudo apt update
#sudo apt -yV upgrade
#sudo apt -y install python-dev python-pip python-setuptools
#sudo apt -y install libopencv-dev opencv-data
#sudo pip3 install opencv-python
#sudo pip3 install opencv-contrib-python; sudo apt-get install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev  libqtgui4  libqt4-test
#sudo apt-get install python3-tk

import sys
import cv2
import time
import tkinter as tk
from PIL import Image,ImageTk


cc=0;
il=0;
def getCam():
    start = time.time()
    global cc
    global il
    rr, img = cc.read()
    #cv2.imwrite('testResult/'+ str(a) +'.jpg', img)
    #cv2.destroyAllWindows()
    #cv2.imshow(str(a),img)
    #cv2.waitKey(1000);
    #img=Image.open('testResult/'+ str(a) +'.jpg')
    image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # imreadはBGRなのでRGBに変換
    image_pil = Image.fromarray(image_rgb) # RGBからPILフォーマットへ変換
    testImg=ImageTk.PhotoImage( image_pil)
    il.configure(image=testImg)
    il.image=testImg;
    root.after(10,getCam)
    print("elapsed_time:{0}".format((time.time()-start)*1000))


#
sys.stderr.write("*** 開始 ***\n")
cc = cv2.VideoCapture(0)
cc.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cc.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc("H","2","6","4"));
rr, img = cc.read()

root=tk.Tk()
root.title("test")
root.minsize(400,400)
root.geometry("400x400+50+50")

canvas=tk.Canvas(bg="black",width=400,height=400)
canvas.place(x=0,y=0)
img=Image.open('testResult/test.png')
testImg=ImageTk.PhotoImage(img)
il=tk.Label(root,image=testImg)
il.pack()


getCam();
print("drawed")

root.mainloop()


sys.stderr.write("*** 終了 ***\n")


# --------------------------------------------------------------------