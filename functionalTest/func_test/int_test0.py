# 测试方向盘取整
import numpy as np

c = np.arange(-5, 5, 0.1)    #生成0到1，不包括1 以0.1为间隔的序列
print(c)

c_round = []
for i in c:
    # print(round(i))
    c_round.append(round(i))

print(c_round)

with open("1.txt", "w") as file_open:
    length = len(c)
    for i in range(length):
        line_list = [c[i], c_round[i]]
        file_open.writelines(str(c[i])+","+str(c_round[i])+"\n")

    file_open.close()
