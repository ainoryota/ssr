from SSC_ImageRecognition14 import IR
import numpy as np
import time

color_image = np.load("np/color_image.npy")
depth_image = np.load("np/depth_image.npy")
ir_image = np.load("np/ir_image.npy")
depth_scale = np.load("np/depth_scale.npy")
ir_scale = np.load("np/ir_scale.npy")

for i in range(10):
    start = time.time()
    result = IR(color_image,depth_image,ir_image,depth_scale,ir_scale,[],0,500,2000,True)
    (y,x,branchValue,Rangle,Langle,ProceedAngle,DepthIRFlag,maxValue1,maxValue2,maxValue3) = result
    print((y,x,branchValue,Rangle,Langle,ProceedAngle,maxValue1,maxValue2,maxValue3) == (120,182,1320,57,30,90,749,428,143),round(time.time() - start,2))