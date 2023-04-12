# 生成直线路径
# (444502.86203611083, 4428582.758912923)
# (444508.74640294677, 4428476.713391734)
# n=213
import math

def func(x):
    y = ((x-x2)/(x1-x2))*(y1-y2)+y2
    return y

def distance(a, b, c, d):
    dis = math.sqrt((a-c)*(a-c)+(b-d)*(b-d))
    return dis

point0 = (444502.86203611083, 4428582.758912923)
point1 = (444508.74640294677, 4428476.713391734)
x1 = point0[0]
y1 = point0[1]
x2 = point1[0]
y2 = point1[1]

A = y1-y2
B = x2-x1
C = -(y1-y2)*x2+y2*(x1-x2)

print("A,B,C:", A, B, C)
print("")



print(func(1))
print(distance(x1, y1, x2, y2))
jiange = (x2-x1)/213

i_list = []
for i in range(213):
    y_produce = func(jiange*i+x1)
    i_list.append((jiange*i+x1, y_produce))

print(i_list)

# if __name__ == "__main__":
#


