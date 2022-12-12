# -*- coding: utf-8 -*-
from prettytable import PrettyTable
from prettytable import MSWORD_FRIENDLY
from itertools import zip_longest
from unicodedata import east_asian_width
from unicodedata import east_asian_width


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
        return [max(self._unicode_width(s) for s in ss) for ss in zip_longest(*array, fillvalue='')]

    def _unicode_width(self, s, width={'F': 2, 'H': 1, 'W': 2, 'Na': 1, 'A': 2, 'N': 1}):
        s = str(s)

        return sum(width[east_asian_width(c)] for c in s)

    def _get_printable_row(self, row):
        maxes = self._calc_maxes()
        #return '| ' + ' | '.join([('{0: <%d}' % m).format(r) for r, m in
        #zip_longest(row, maxes, fillvalue='')]) + ' |'
        return '| ' + ' | '.join([str(r) + ' ' * (m - self._unicode_width(r)) for r, m in zip_longest(row, maxes, fillvalue='')]) + ' |'

    def _get_printable_header(self):
        return self._get_printable_row(self.header)

    def _get_printable_border(self):
        maxes = self._calc_maxes()
        return '+-' + '-+-'.join(['-' * m for m in maxes]) + '-+'

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
    def __init__(self,header,axisName):
        # 引数を属性にセット
        self.header = [" "] + header
        self.axisName = axisName


    def ViewTable(self,data):
        table = SimpleTable(self.header)
        for i in range(len(data)):
            table.add_row([self.axisName[i]] + [str(a) for a in data[i]])

        # 表を表示
        table.print_table()

