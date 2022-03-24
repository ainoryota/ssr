import time
import os
import multiprocessing
from Utilty import processing_time
from multiprocessing import Process

@processing_time
def MultiProsess(funcList):
    for func in funcList:
        p = multiprocessing.Process(target=func)
        p.start()

def call_slow_request():
    print("start one")
    #time.sleep(3)
    print("end one")

def call_slow_request2():
    print("start two")
    #time.sleep(3)
    print("end two")

def call_loop_request():
    print("start 1sec")
    time.sleep(1)
    print("end 1sec")
    call_loop_request();

#MultiProsess([call_slow_request,call_slow_request2])

class MyProcessor(Process):

    def __init__(self, num):
        super().__init__()
        self.__num = num

    def fizz_buzz(self, num: int):
        result_list = []
        for i in range(1, num + 1):
            result = ''
            if i % 3 == 0:
                result += 'fizz'
            if i % 5 == 0:
                result += 'buzz'
            if not result:
                result = str(i)
            result_list.append(result)
        return result_list

    def run(self):
        self.fizz_buzz(self.__num)

def startMultiProcess(obj):
    obj.start();

if __name__ == '__main__':
    start = time.time()
    processes = []
    num_list = [22000000, 19000000, 25000000, 24500000, 21300000]
    for n in num_list:
        process = MyProcessor(n)
        process.start()
        processes.append(process)
    for p in processes:
        p.join()
    stop = time.time()
    print(f'multi process: {stop - start:.3f} seconds')