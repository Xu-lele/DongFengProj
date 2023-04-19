#获取相对与本文件的文件路径，以方便从本文件导航到另一个文件
import os
p_path = os.path.abspath(os.path.dirname(__file__))
print(p_path)
ctrl_miss_path = os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)),"../data/ctrl_mission_planning"))
print(ctrl_miss_path)

#添加环境变量 方便模组的跨目录导入
import sys
sys.path.append("/66666666666/666666666666")
print(sys.path)
