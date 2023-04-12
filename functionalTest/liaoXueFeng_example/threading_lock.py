# 测试加锁实现,用local改造(只适用于同一函数的多线程，其他线程里并无法访问thread_local)
import time, threading

# 假定这是你的银行存款:
balance = 0


def change_it(n):
    # 先存后取，结果应该为0:
    thread_local.balance = thread_local.balance + n
    thread_local.balance = thread_local.balance - n


def run_thread(n):
    thread_local.balance = 0
    for i in range(2000000):
        change_it(n)
    print(thread_local.balance, threading.current_thread().name)


def use_local_test():
    while 1:
        print(thread_local.balance, threading.current_thread().name)
        time.sleep(0.5)


if __name__ == "__main__":
    thread_local = threading.local()

    t1 = threading.Thread(target=run_thread, args=(5,))
    t2 = threading.Thread(target=run_thread, args=(8,))
    t3 = threading.Thread(target=use_local_test)
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()
    # print(thread_local.balance)



