import cv2 #pip install opencv-python
from SSC_ImageRecognition12 import IRRecognition
ir_image=cv2.imread('ir.png')
IRRecognition(ir_image)
