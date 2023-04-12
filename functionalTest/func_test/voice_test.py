# 测试发声
from playsound import playsound
import threading
import ctypes   # 使用ctypes结束线程需要设计线程是可以直接杀死的，而没有临界资源的占用
import inspect


def voice_test():
    while True:
        playsound('bomb_warnings.mp3')


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)
    return "success"

# while True:
#     playsound('buzzing.mp3')




# t1.join()

if __name__ == "__main__":
    t = threading.Thread(target=voice_test, name='test')
    t.start()

    t1 = threading.Thread(target=voice_test, name='test')
    t1.start()
    print(t.isAlive())
    print(stop_thread(t))
    print(stop_thread(t1))
    print('daozheli ')
    print('jjj')
