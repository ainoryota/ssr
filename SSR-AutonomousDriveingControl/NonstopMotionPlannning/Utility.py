# -*- coding: utf-8 -*-
from prettytable import PrettyTable
from prettytable import MSWORD_FRIENDLY
from itertools import zip_longest
from unicodedata import east_asian_width
from unicodedata import east_asian_width
import numpy as np
import math


class SimpleTable(object):
    """
    SimpleTable
    Print a simple table as follows:
    +--------------+----------+----------+
    | Header 1     | Header 2 | Header 3 |
    +--------------+----------+----------+
    | aaa          | bbb      | ccc      |
    | aaaaaaaaaaaa | bb       | ccccc    |
    | a            | b        |          |
    +--------------+----------+----------+
    """

    def __init__(self, header=None, rows=None):
        self.header = header or ()
        self.rows = rows or []

    def set_header(self, header):
        self.header = header

    def add_row(self, row):
        self.rows.append(row)

    def _calc_maxes(self):
        array = [self.header] + self.rows
        return [
            max(self._unicode_width(s) for s in ss)
            for ss in zip_longest(*array, fillvalue="")
        ]

    def _unicode_width(self, s, width={"F": 2, "H": 1, "W": 2, "Na": 1, "A": 2, "N": 1}):
        s = str(s)

        return sum(width[east_asian_width(c)] for c in s)

    def _get_printable_row(self, row):
        maxes = self._calc_maxes()
        # return '| ' + ' | '.join([('{0: <%d}' % m).format(r) for r, m in
        # zip_longest(row, maxes, fillvalue='')]) + ' |'
        return ("| " + " | ".join([
                    str(r) + " " * (m - self._unicode_width(r))
                    for r, m in zip_longest(row, maxes, fillvalue="")
                ]) + " |")

    def _get_printable_header(self):
        return self._get_printable_row(self.header)

    def _get_printable_border(self):
        maxes = self._calc_maxes()
        return "+-" + "-+-".join(["-" * m for m in maxes]) + "-+"

    def get_table(self):
        lines = []
        if self.header:
            lines.append(self._get_printable_border())
            lines.append(self._get_printable_header())
        lines.append(self._get_printable_border())
        for row in self.rows:
            lines.append(self._get_printable_row(row))
        lines.append(self._get_printable_border())
        return lines

    def print_table(self):
        lines = self.get_table()
        for line in lines:
            print(line)


class TableData:
    def __init__(self, header, axisName):
        # 引数を属性にセット
        self.header = [" "] + header
        self.axisName = axisName

    def ViewTable(self, data):
        table = SimpleTable(self.header)
        for i in range(len(data)):
            table.add_row([self.axisName[i]] + [str(a) for a in data[i]])

        # 表を表示
        table.print_table()


# Pの座標をO2を原点として傾きax+by=-zの平面に載っているとみなした座標系に座標変換する
def ConvertXYZCoodinate(O2, a, b, c, P):
    sigma = -1
    alpha = 1 / math.sqrt(a * a + c * c)
    beta = 1 / math.sqrt(a * a + b * b + c * c)

    x = np.array(P[0]) - O2[0]
    y = np.array(P[1]) - O2[1]
    z = np.array(P[2]) - O2[2]

    X = x * c * alpha * sigma - z * a * alpha * sigma
    Y = (x * alpha * beta * a * b + y * alpha * beta * (a * a + c * c) - z * alpha * beta * b * c)
    Z = x * a * beta * sigma + y * b * beta * sigma + z * c * beta * sigma
    return X, Y, Z


#P1とP2を軸にする半径rの円柱のプロット点群を返す
def CalcCylinderNP(P1,P2,r):
    result = []
    e1 = (P2 - P1) / np.linalg.norm(P2 - P1)
    for t in [n * np.linalg.norm(P2 - P1) / 100 for n in range(1,100)]:
        P_v = P1 + t * e1

        #何でもいいからP1P2に垂直な単位ベクトルv1を用意
        dP=P2-P1
        if(dP[0][0] == 0):
            v1 = np.array([[1,0,0]]).T
        elif(dP[1][0] == 0):
            v1 = np.array([[0,1,0]]).T
        elif(dP[2][0] == 0):
            v1 = np.array([[0,0,1]]).T
        else:
            try:
                v1 = np.array([[dP[0][0],dP[1][0],-(dP[0][0] ** 2 + dP[1][0] ** 2) / dP[2][0]]]).T
                v1/=np.linalg.norm(v1)
            except Exception as e:
                v1 = np.array([[-(dP[1][0] ** 2 + dP[2][0] ** 2) / dP[0][0],dP[1][0],dP[2][0]]]).T
                v1/=np.linalg.norm(v1)


        v2 = cross(v1,e1)
        for theta in [n * 2 * math.pi / 100 for n in range(100)]:
            result.append(P_v + r * (v1 * math.cos(theta) + v2 * math.sin(theta)))
    return result
    
def cross(v1, v2):
    v3 = [v1[1][0] * v2[2][0] - v1[2][0] * v2[1][0],
         v1[2][0] * v2[0][0] - v1[0][0] * v2[2][0],
         v1[0][0] * v2[1][0] - v1[1][0] * v2[0][0]]

    return np.array([v3]).T