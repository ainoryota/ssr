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
#
sys.stderr.write("*** 開始 ***\n")
cc = cv2.VideoCapture(0)
cc.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc("H","2","6","4"));
start = time.time()
count=100
for a in range(count):
	rr, img = cc.read()
	print(a)
	cv2.imwrite('testResult/'+ str(a) +'.jpg', img)
	cv2.destroyAllWindows()
	cv2.imshow(str(a),img)
	cv2.waitKey(1000);
	#time.sleep(1)
#
print("elapsed_time:{0}".format((time.time()-start)*1000/count))
sys.stderr.write("*** 終了 ***\n")
# --------------------------------------------------------------------