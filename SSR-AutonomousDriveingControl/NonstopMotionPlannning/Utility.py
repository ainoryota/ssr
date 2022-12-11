from prettytable import PrettyTable


class TableData:
    def __init__(self,header,axisName):
        # 引数を属性にセット
        self.header = [" "]+header
        self.axisName = axisName


    def ViewTable(self,data):
        data_view = []
        for i in range(len(data)):
            data_view.append([self.axisName[i]] + data[i])

        # 表を作成
        table = PrettyTable(self.header)
        for row in data_view:
            table.add_row(row)

        # 表を表示
        print(table)

