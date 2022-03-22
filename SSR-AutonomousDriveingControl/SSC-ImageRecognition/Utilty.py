import time
import threading
from functools import wraps




def processing_time(func):
    @wraps(func)
    def wrapper(*args, **keywords):
        st = time.time()  # 開始前の時間を記録
        result = func(*args, **keywords)  # 関数を実行
        print(f'time: {time.time() - st} s')  # 開始後の時間と開始前の時間の差を出力
        return result

    return wrapper

