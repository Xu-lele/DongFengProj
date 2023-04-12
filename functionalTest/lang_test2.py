# 测试单例模式
class Singleton(object):
    def __new__(cls, name):
        if not hasattr(cls, 'instance'):
            cls.instance = super().__new__(cls)
        return cls.instance

    def __init__(self, name):
        self.name = name


s1 = Singleton('Singleton1')
print(s1)
# => <__main__.Singleton object at 0x7efc1b006220>
print(s1.name)
# => Singleton1

s2 = Singleton('Singleton2')
print(s2)
# => <__main__.Singleton object at 0x7efc1b006220>
print(s2.name)
# => Singleton2
print(s1.name)
# => Singleton2
