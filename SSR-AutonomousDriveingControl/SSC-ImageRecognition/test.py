from functools import partial
import sys
import cv2
import time
import math
import pyrealsense2 as rs
import numpy as np
from numba import jit
from threading import Thread
import threading
from System import System
from Robot import RealRobot
from Form import Form
from OutputController import OutputDone
from OutputController import OutputController
from multiprocessing import Process, Manager
import tkinter as tk
import platform
from PIL import Image,ImageTk #udo pip install pillow
from ctypes import alignment, windll
import os


system = System()
robot = RealRobot()
time.sleep(2);
form = Form(robot)