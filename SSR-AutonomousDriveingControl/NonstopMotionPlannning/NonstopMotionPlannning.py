import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from Utility import TableData
import random
import math

class Parameter:
    def __init__(self,DataMax,minValue,maxValue,DefinedIdx,DefinedValue):
        self.DataMax = DataMax
        self.MinValue = minValue
        self.MaxValue = maxValue
        self.DefinedIdx = DefinedIdx
        self.DefinedValue = DefinedValue

        self.InitValue = 0
        self.real_value = np.zeros(self.DataMax)
        self.d_value = np.zeros(self.DataMax)
        self.p_value = np.zeros(self.DataMax)

    def GetRand(self,min,max):
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
            self.p_value[i] = self.d_value[i] ** 2/100

    def SetRound(self):
        for i in range(self.DataMax):
            self.d_value[i] = round(self.d_value[i],2)
            self.real_value[i] = round(self.real_value[i],2)
            self.p_value[i] = round(self.p_value[i],2)



class NonStopMotionPlannning:
    def __init__(self,DataMax):
        self.DataMax = DataMax
        self.alpha = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)
        self.beta = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)
        self.gamma = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)
        self.omega = Parameter(self.DataMax,-20,20,int(self.DataMax / 2),30)

        self.t = np.zeros(self.DataMax)
        self.P = np.zeros(self.DataMax)
        self.data = [self.alpha,self.beta,self.gamma,self.omega]

    def SolveP(self):
        for i in range(self.DataMax):
            self.P[i] = 0
            for d in self.data:
                self.P[i]+=d.p_value[i]
            self.P[i] = round(self.P[i],2)

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
        print(round(np.sum(self.P),2))


NSMP = NonStopMotionPlannning(20)
NSMP.run()


