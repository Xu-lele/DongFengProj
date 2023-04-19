# @Time    : 2021/9/26 下午8:46
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : list_test.py
# @Software: PyCharm
# 测试可变变量传参

list_0 = []
def change_value(list: list):
    a = list
    a.append(1)

change_value(list_0)
print(list_0)


