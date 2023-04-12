# 测试多进程. 处理好多进程和异常处理，保障数据记录。
# 掌握好多进程的输入输出，然后适时采用多进程
from multiprocessing import Process
import os


# 子进程代码
def run_proc(name):
    print('Run child process %s (%s)...' % (name, os.getpid()))
    return 1


if __name__ == "__main__":
    print("Parent process %s." % os.getpid())
    p = Process(target=run_proc, args=('test', ))
    print("Child process will start.")
    p.start()
    p.join()
    print('The end.')
    # 看下返回值