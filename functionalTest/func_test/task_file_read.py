# @Time    : 2021/9/12 下午3:40
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : task_file_read.py
# @Software: PyCharm
# 测试任务文件的读取
import os

ctrl_miss_path = os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)), "../../pathGenerate/ctrl_mission_planning"))

print("轨迹文件的路径是： " + ctrl_miss_path + " 运行失败请检查文件路径是否正确")

task_dict = {}
for root, dirs, files in os.walk(ctrl_miss_path):
    for file in files:
        print(file)
        file_underline = file.split("_")
        file_index = int(file_underline[0])
        task_dict.update({file_index: file})

print(task_dict)
print()
dict_len = len(task_dict)
for i in range(dict_len):
    # print(task_dict[i])
    # 在这里执行任务
    pass

