# 测试多线程、多进程
import threading, time


def one_thread():
    global sum
    while True:
        print(sum)


sum = 0
t = threading.Thread(target=one_thread, name="one_thread")
t.start()
t.isAlive()
t.join()    # 等待直到线程终止，为了同步吗？
print("是否执行")

