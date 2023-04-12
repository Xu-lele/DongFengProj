# 测试python传参,通过传可变类型实现参数返回，不用全局变量即可实现
import threading, time


def change_value(a):
    time.sleep(4)
    a[0] = 3


if __name__ == "__main__":
    a = [1]
    t = threading.Thread(target=change_value, args=(a, ))
    # change_value(a)
    t.start()
    t.join()
    print(a)    # 可能会脏读

