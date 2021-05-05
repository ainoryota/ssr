#! /usr/bin/python3
# -*- coding: utf-8 -*-
#
#   usb_camera.py
#
#                       Oct/17/2020
# --------------------------------------------------------------------
import sys
import cv2
import time
import tkinter as tk
from PIL import Image,ImageTk


a=0;
cc=0;
canvas=0;
il=0;
def getCam():
	global a
	global cc
	global canvas
	global il
	a=a+1;
	print(a)
	rr, img = cc.read()
	
	cv2.imwrite('testResult/'+ str(a) +'.jpg', img)
	cv2.destroyAllWindows()
	#cv2.imshow(str(a),img)
	#cv2.waitKey(1000);
	img=Image.open('testResult/'+ str(a) +'.jpg')
	testImg=ImageTk.PhotoImage(img)
	#canvas.create_image(0,0,image=testImg,anchor="nw")
	il.configure(image=testImg)
	il.image=testImg;
	root.after(20,getCam)


#
sys.stderr.write("*** 開始 ***\n")
cc = cv2.VideoCapture(0)
cc.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cc.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc("H","2","6","4"));
start = time.time()
count=100
rr, img = cc.read()

root=tk.Tk()
root.title("test")
root.minsize(400,400)

canvas=tk.Canvas(bg="black",width=400,height=400)
canvas.place(x=0,y=0)
img=Image.open('testResult/0.jpg')
testImg=ImageTk.PhotoImage(img)
il=tk.Label(root,image=testImg)
il.pack()



#canvas=tk.Canvas(bg="black",width=400,height=400)
#canvas.place(x=0,y=0)
#img=Image.open("Image/SSC1.png")
#testImg=ImageTk.PhotoImage(img)
#canvas.create_image(0,0,image=testImg,anchor="nw")
#root.mainloop()
root.after(20,getCam)
root.mainloop()


#time.sleep(1)
#
print("elapsed_time:{0}".format((time.time()-start)*1000/count))
sys.stderr.write("*** 終了 ***\n")


# --------------------------------------------------------------------