from ctypes import windll

class System(object):
    """description of class"""
    def __init__(self):
        windll.winmm.timeBeginPeriod(1)

