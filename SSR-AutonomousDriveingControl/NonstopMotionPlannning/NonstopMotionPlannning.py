import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from Utility import TableData
import random
import math

from re import M
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Utility import ConvertXYZCoodinate,CalcCylinderNP,cross
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation



class Parameter:
    def __init__(self, DataMax, minValue, maxValue, DefinedIdx, DefinedValue):
        self.DataMax = DataMax
        self.MinValue = minValue
        self.MaxValue = maxValue
        self.DefinedIdx = DefinedIdx
        self.DefinedValue = DefinedValue

        self.InitValue = 0
        self.real_value = np.zeros(self.DataMax)
        self.d_value = np.zeros(self.DataMax)
        self.p_value = np.zeros(self.DataMax)

    def GetRand(self, min, max):
        return random.random() * (max - min) + min

    def SetRand(self):
        for i in range(self.DataMax):
            self.d_value[i] = self.GetRand(self.MinValue, self.MaxValue)

    def SetResult(self):
        for i in range(self.DataMax):
            self.real_value[i] = np.sum(self.d_value[:i]) + self.InitValue

        self.real_value[self.DefinedIdx] = self.DefinedValue
        self.d_value[self.DefinedIdx - 1] = self.real_value[self.DefinedIdx] - self.real_value[self.DefinedIdx - 1]
        self.d_value[self.DefinedIdx] = self.real_value[self.DefinedIdx + 1] - self.real_value[self.DefinedIdx]

        for i in range(self.DataMax):
            self.p_value[i] = self.d_value[i] ** 2 / 100

    def SetRound(self):
        for i in range(self.DataMax):
            self.d_value[i] = round(self.d_value[i], 2)
            self.real_value[i] = round(self.real_value[i], 2)
            self.p_value[i] = round(self.p_value[i], 2)


class NonStopMotionPlannning:
    def __init__(self, DataMax):
        self.DataMax = DataMax
        self.alpha = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)
        self.beta = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)
        self.gamma = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)
        self.omega = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)

        self.t = np.zeros(self.DataMax)
        self.P = np.zeros(self.DataMax)
        self.data = [self.alpha, self.beta, self.gamma, self.omega]
        self.line_len_a = 400
        self.line_len_b = 400
        self.line_len_c = 400
        self.theta_L = 30
        self.theta_R = 45

        self.r = 60


    def SolveP(self):
        for i in range(self.DataMax):
            self.P[i] = 0
            for d in self.data:
                self.P[i] += d.p_value[i]
            self.P[i] = round(self.P[i], 2)

    def run(self):
        self.alpha.SetRand()
        self.beta.SetRand()
        self.gamma.SetRand()
        self.omega.SetRand()

        self.alpha.SetResult()
        self.beta.SetResult()
        self.gamma.SetResult()
        self.omega.SetResult()

        self.alpha.SetRound()
        self.beta.SetRound()
        self.gamma.SetRound()
        self.omega.SetRound()

        self.SolveP()

        table = TableData([str(n) for n in range(self.DataMax)],["t","α","β","γ","ω","Δα","Δβ","Δγ","Δω","P_α","P_β","P_γ","P_ω","P"])
        values = [self.t]

        for d in self.data:
            values.append(d.real_value)

        for d in self.data:
            values.append(d.d_value)

        for d in self.data:
            values.append(d.p_value)

        values.append(self.P)

        table.ViewTable(values)
        print(round(np.sum(self.P), 2))

    def plot(self):
        # Figureを追加
        fig = plt.figure(figsize=(8, 8))

        def plot_time(t):
            # 3DAxesを追加
            ax = fig.add_subplot(111, projection="3d")

            # Axesのタイトルを設定
            ax.set_title("Helix", size=20)

            # 軸ラベルを設定
            ax.set_xlabel("x", size=14)
            ax.set_ylabel("y", size=14)
            ax.set_zlabel("z", size=14)

            ax.cla()
            ax.set_xlim(-500, 500)
            ax.set_ylim(0,1000)
            ax.set_zlim(-500, 500)

            xlim = ax.get_xlim()
            ylim = ax.get_ylim()

            ax.plot(np.array([0, 0]), np.array([0, self.line_len_a]), np.array([0, 0]), c="blue")
            ax.plot(np.array([0, self.line_len_b * math.cos(math.radians(self.theta_R))]), np.array([self.line_len_a, self.line_len_a + self.line_len_b * math.sin(math.radians(self.theta_R))]), np.array([0, 0]), c="blue")
            ax.plot(np.array([0, -self.line_len_c * math.cos(math.radians(self.theta_L))]), np.array([self.line_len_a, self.line_len_a + self.line_len_b * math.sin(math.radians(self.theta_R))]), np.array([0, 0]), c="blue")
        
            points = CalcCylinderNP(np.array([[0,0,0]]).T,np.array([[50,0,0]]).T,self.r)
            ax.plot(np.array(points)[:,0,0],np.array(points)[:,1,0],np.array(points)[:,2,0], c="green")


        #ani = animation.FuncAnimation(fig, plot_time, interval=10)
        plot_time(0)
        plt.show()


NSMP = NonStopMotionPlannning(20)
NSMP.run()
NSMP.plot()
