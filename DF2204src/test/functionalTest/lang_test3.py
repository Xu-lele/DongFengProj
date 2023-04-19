# 单例测试
import sys
import time

def test_out_output():
    print("this is out output.")

class Singleton(object):
    print("测试输出行数", sys._getframe().f_lineno)
    time.time()
    test_out_output()

    def __new__(cls, name):
        if not hasattr(cls, 'instance'):
            cls.instance = super().__new__(cls)
        return cls.instance

    def __init__(self, name):
        print("输出行数：", sys._getframe().f_lineno)
        self.name = name

    def test_output(self):
        print("不正常输出")

s1 = Singleton('Singleton1')
print(s1)
print(s1.name)
s2 = Singleton('Singleton2')
print(s2)
print(s2.name)
print(s1.name)
