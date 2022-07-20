import numpy as np
import pyrealsense2 as rs

###
import platform
from functools import partial

import sys
import tkinter as tk
from PIL import Image,ImageTk #udo pip install pillow

import math
import pyrealsense2 as rs
import numpy as np

import sys
from numba import jit
from ctypes import windll

from threading import Thread
import threading


class VideoStream(object):
        def __init__(self, resolution=(640, 360), framerate=15):

            #devices = rs.context().query_devices();
            #print(devices)
            #for  device in devices:
            #    print(device);

            #print(rs.device_list) 
            #print(rs.context.query_devices().size());
            #print(rs.context.query_devices()[0]);
            #print(rs.context.query_devices()[1]);
            #print(rs.context.query_devices());
            self.USE_IMU=False

            self.debugMode = False
            self.color_image = np.zeros((resolution[0], resolution[1]))
            self.depth_image = self.color_image
            self.ir_image1 = self.color_image
            self.ir_image2 = self.color_image
            self.resolution = resolution
            self.framerate = framerate
        
            if(self.USE_IMU):
                self.imu_pipe = rs.pipeline()
                self.imu_config = rs.config()
                self.imu_config.enable_stream(rs.stream.gyro)
                self.imu_config.enable_stream(rs.stream.accel)
            
            self.acc = []
            self.gyro = []
        
            self.vid_pipe = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, framerate)
            self.config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, framerate)
            self.config.enable_stream(rs.stream.infrared, 1, resolution[0], resolution[1], rs.format.y8, framerate)
            self.config.enable_stream(rs.stream.infrared, 2, resolution[0], resolution[1], rs.format.y8, framerate)
        
        def start(self):
            if(self.USE_IMU):
                self.start_imu()
            self.start_camera()

        def start_camera(self):
            self.vid_pipe.start(self.config)
            Thread(target=self.update_cam).start()       

        def start_imu(self):
            self.imu_pipe.start(self.imu_config)
            Thread(target=self.update_imu).start()

        def update_cam(self):
            try:
                while True:
                    # Wait for a coherent pair of frames: depth and color
                    vid_frames = self.vid_pipe.wait_for_frames()
                    depth_frame = vid_frames.get_depth_frame()
                    color_frame = vid_frames.get_color_frame()
                    ir_frame1 = vid_frames.get_infrared_frame(1)
                    ir_frame2 = vid_frames.get_infrared_frame(2)
                    if not depth_frame or not color_frame:
                        continue

                    # Convert images to numpy arrays
                    self.depth_image = np.asanyarray(depth_frame.get_data())
                    self.color_image = np.asanyarray(color_frame.get_data())
                    self.ir_image1 = np.asanyarray(ir_frame1.get_data())
                    self.ir_image2 = np.asanyarray(ir_frame2.get_data())
                    if(self.debugMode):break
            except:
                self.vid_pipe.stop()
                print("Error in Vision", sys.exc_info())

            finally:
                self.vid_pipe.stop()


        def update_imu(self):
            try:
                while True:
                    # Wait for a coherent pair of frames: depth and color
                    mot_frames = self.imu_pipe.wait_for_frames()
                    self.acc = mot_frames[0].as_motion_frame().get_motion_data()
                    self.gyro = mot_frames[1].as_motion_frame().get_motion_data()
                    if(self.debugMode):break
            except:
                self.imu_pipe.stop()
                print("Error in Vision", sys.exc_info())

            finally:
                self.imu_pipe.stop()