# @Time    : 2021/9/16 下午8:38
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : cubic_spline_interpolation_func.py
# @Software: PyCharm
# 测试三次b样条曲线曲线路径生成
import numpy as np
import bisect
from matplotlib import pyplot as plt


class Spline:
    """
    三次样条类
    """

    def __init__(self, x, y):
        self.a, self.b, self.c, self.d = [], [], [], []     # 这个学习一下

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.m = np.linalg.solve(A, B)
        self.c = self.m / 2.0

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        计算位置
        当t超过边界，返回None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
                 self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        计算算法第二步中的等号左侧的矩阵表达式A
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h):
        """
        计算算法第二步中的等号右侧的矩阵表达式B
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 6.0 * (self.a[i + 2] - self.a[i + 1]) / h[i + 1] - 6.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


def main():
    # x = [-4., -2, 0.0, 2, 4, 6, 10]
    # y = [1.2, 0.6, 0.0, 1.5, 3.8, 5.0, 3.0]

    x = [0, 0.1, 3, 10]
    y = [0, 0.5, 5, 3.0]

    spline = Spline(x, y)
    # rx = np.arange(-4.0, 10, 0.01)
    rx = np.arange(0, 10, 0.01)
    ry = [spline.calc(i) for i in rx]

    plt.plot(x, y, "og")
    plt.plot(rx, ry, "-r")
    plt.grid(True)
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
    # 测试下给多个变量赋值
    # a, b = 0, 1
    # try:
    #     assert a == 10
    # except AssertionError:
    #     print("有错误")
    # finally:
    #     print("执行该句话")

