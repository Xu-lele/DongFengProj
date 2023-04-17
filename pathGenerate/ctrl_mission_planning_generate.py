# @Time    : 2021/9/22 下午8:19
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : ctrl_mission_planning_generate.py
# @Software: PyCharm
# @func    : 生成掉头轨迹代码
import math
import time

import numpy as np
from matplotlib import pyplot as plt
# from pyproj import Proj, transform
from pyproj import Transformer
from typing import Tuple
import ab_line_generate
import bezier_gen

transformer = Transformer.from_crs("epsg:32650", "epsg:4326")


def xy2ll(x, y):
    # WGS84 = Proj(init='EPSG:4326')  # WGS84
    # p = Proj(init="EPSG:32650")  # UTM 50N  这个可能会影响到除北京外的作业
    # lon, lat = transform(p, WGS84, x, y)
    lat, lon = transformer.transform(x, y)
    return lon, lat


def calcAngOfY_Axis(vector):
    """
        计算向量与Y轴的顺时针夹角，范围0~360°
    :param vector:
    :return: 返回两个向量的夹角
    """
    # endAng=0
    vector1 = (0, 1)  # Y axis
    vector2 = vector
    AB = [0, 0, vector1[0], vector1[1]]
    CD = [0, 0, vector2[0], vector2[1]]

    def angle(v1, v2):
        # 计算v1，v2两角的0~180度角度
        dx1 = v1[2] - v1[0]
        dy1 = v1[3] - v1[1]
        dx2 = v2[2] - v2[0]
        dy2 = v2[3] - v2[1]
        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle
        return included_angle

    ang1 = angle(AB, CD)
    # 浮点数应该可以比较大小，但是比较相等要用精度判断
    # if waypoint_x - x > 0:
    #     # 在右边，0~180
    #     endAng = ang1
    # else:
    #     # 在左边，180~360
    #     endAng = 360 - ang1
    if vector2[0] < 0:
        ang1 = 360 - ang1

    return ang1


# class Ring_Mission_Path_Generate(object):
#     """本类旨在实现套圈无人驾驶路径的生成，即生成全局路径的局部规划"""
#
#     def __init__(self, A, B, radius, beta):
#         self.A = A
#         self.B = B
#         self.len_AB = math.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2)
#         self.radius = radius
#         self.beta = beta
#         path_point_redundant = 0.01  # m
#         self.path_point_redundant = path_point_redundant
#
#         # 尝试生成曲线的离散化点，按固定distance来取
#         curve_list = [(0, 0)]
#         sliding_x_first = 0
#         sliding_x_end = 0
#         gap_dis = 0.5  # m,相邻点间隔
#
#         while sliding_x_end <= 1.5 * radius:
#             s_point_y = math.sqrt(radius * radius - (sliding_x_first - radius) ** 2)
#             e_point_y = math.sqrt(radius * radius - (sliding_x_end - radius) ** 2)
#             sliding_dis = math.sqrt((sliding_x_end - sliding_x_first) ** 2 + (e_point_y - s_point_y) ** 2)
#             if sliding_dis >= gap_dis:  # 这个想办法统一一下
#                 # 添加进list
#                 curve_list.append((sliding_x_end, e_point_y))
#                 sliding_x_first = sliding_x_end
#             sliding_x_end += 0.01
#         curve_list.append((3 / 2 * radius, math.sqrt(3 / 4 * radius * radius)))
#         print(curve_list)
#
#         p_0 = (0, 0)
#         p_neg_1 = (p_0[0] - path_point_redundant, p_0[1] - path_point_redundant)
#         p_neg_2 = (p_neg_1[0] - path_point_redundant, p_neg_1[1] - path_point_redundant)
#         p_1_0 = (radius / 4, math.sqrt(7) / 4 * radius)
#         p_1 = (radius / 2, math.sqrt(3 / 4 * radius * radius))
#         p_2 = (radius, radius)
#         p_3 = (3 / 2 * radius, math.sqrt(3 / 4 * radius * radius))
#         p_4 = (p_3[0] + path_point_redundant, p_3[1] + path_point_redundant)
#         p_5 = (p_4[0] + path_point_redundant, p_4[1] + path_point_redundant)
#         self.p_list = [p_neg_2, p_neg_1, p_0, p_1_0, p_1, p_2, p_3, p_4, p_5]
#         self.p_list = curve_list
#         self.p_list_left = []
#         for i in self.p_list:
#             self.p_list_left.append((-i[0], i[1]))
#         self.p_list_left_of_next_line = []
#         for i in self.p_list_left:
#             self.p_list_left_of_next_line.append((i[0] + radius, i[1]))
#         self.p_list_left_of_next_line.reverse()
#
#         self.p_list_right_of_next_line = []
#         for i in self.p_list:
#             self.p_list_right_of_next_line.append((i[0] - radius, i[1]))
#         self.p_list_right_of_next_line.reverse()
#
#     def curve_three_generate(self, floder_name: str, start_index: int, seq_dis: float, extending_direction_left: bool):
#         # 把生成的文件保存下来
#         # 生成utm坐标点序列
#         # utm_p_x_list = []
#         # utm_p_y_list = []
#
#         # 写第一段掉头曲线前进
#         with open(floder_name + "/" + str(start_index + 1) + "_掉头前进离开直线.txt", 'w') as file_open:
#             # 写表头
#             file_open.write("x,y,lon,lat\n")
#             if extending_direction_left:  # True左，False右
#                 mid_p_list = self.p_list_left  # 左侧离开直线
#             else:
#                 mid_p_list = self.p_list  # 右侧离开直线
#
#             for orig_p in mid_p_list:
#                 # 先旋转，再平移
#                 x_skim = orig_p[0]
#                 y_skim = orig_p[1]
#
#                 # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
#                 # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)
#
#                 # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
#                 # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]
#
#                 utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
#                 utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
#
#                 lon, lat = xy2ll(utm_p_x, utm_p_y)
#                 # utm_p_x_list.append(utm_p_x)
#                 # utm_p_y_list.append(utm_p_y)
#                 # file_open.write(str(lon)+","+str(lat)+"\n")
#                 file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
#
#             file_open.close()
#
#         # 写第二段即掉头直线后退
#         with open(floder_name + "/" + str(start_index + 2) + "_掉头直线后退.txt", 'w') as file_open:
#             # 写表头
#             file_open.write("x,y,lon,lat\n")
#             # 得到起止点
#             start_point = self.p_list[-1]
#             end_point = self.p_list_left_of_next_line[0]
#
#             start_point_neg_1 = (start_point[0] + 0.01, start_point[1])
#             start_point_neg_2 = (start_point_neg_1[0] + 0.01, start_point[1])
#             end_point_neg_1 = (end_point[0] - 0.01, end_point[1])
#             end_point_neg_2 = (end_point_neg_1[0] - 0.01, end_point[1])
#
#             if extending_direction_left:  # True左，False右
#                 start_point = self.p_list_left[-1]
#                 end_point = self.p_list_right_of_next_line[0]
#
#                 start_point_neg_1 = (start_point[0] - 0.01, start_point[1])
#                 start_point_neg_2 = (start_point_neg_1[0] - 0.01, start_point[1])
#                 end_point_neg_1 = (end_point[0] + 0.01, end_point[1])
#                 end_point_neg_2 = (end_point_neg_1[0] + 0.01, end_point[1])
#
#             ab_list = [start_point_neg_2, start_point_neg_1] + ab_line_generate_1_0.ab_line_seq(start_point, end_point,
#                                                                                                 seq_dis) \
#                       + [end_point_neg_1, end_point_neg_2]
#             # 直线生成与保存，需要生成多余点
#             for orig_p in ab_list:
#                 # 先旋转，再平移
#                 x_skim = orig_p[0]
#                 y_skim = orig_p[1]
#
#                 # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
#                 # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]
#
#                 utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
#                 utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
#                 lon, lat = xy2ll(utm_p_x, utm_p_y)
#                 # utm_p_x_list.append(utm_p_x)
#                 # utm_p_y_list.append(utm_p_y)
#                 # file_open.write(str(lon)+","+str(lat)+"\n")
#                 file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
#             file_open.close()
#
#         # 写第三段即掉头曲线前进
#         with open(floder_name + "/" + str(start_index + 3) + "_掉头前进靠近直线.txt", 'w') as file_open:
#             # 写表头
#             file_open.write("x,y,lon,lat\n")
#
#             if extending_direction_left:
#                 mid_p_list = self.p_list_right_of_next_line  # 左侧靠近直线
#             else:
#                 mid_p_list = self.p_list_left_of_next_line  # 左侧离开直线
#
#             for orig_p in mid_p_list:
#                 # 先旋转，再平移
#                 x_skim = orig_p[0]
#                 y_skim = orig_p[1]
#
#                 # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
#                 # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)
#
#                 # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
#                 # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]
#
#                 utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
#                 utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
#                 lon, lat = xy2ll(utm_p_x, utm_p_y)
#                 # utm_p_x_list.append(utm_p_x)
#                 # utm_p_y_list.append(utm_p_y)
#                 # file_open.write(str(lon)+","+str(lat)+"\n")
#                 file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
#
#             file_open.close()
#
#
# class Primary_Mission_Path_Generate(object):
#     """本类旨在实现方向盘控制下的路径生成，即生成n条指定幅宽的AB线"""
#
#     def __init__(self, A, B, radius, beta):
#         self.A = A
#         self.B = B
#         self.len_AB = math.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2)
#         self.radius = radius
#         self.beta = beta
#         path_point_redundant = 0.01  # m
#         self.path_point_redundant = path_point_redundant
#
#         # 尝试生成曲线的离散化点，按固定distance来取
#         curve_list = [(0, 0)]
#         sliding_x_first = 0
#         sliding_x_end = 0
#         gap_dis = 0.5  # m,相邻点间隔
#
#         while sliding_x_end <= 1.5 * radius:
#             s_point_y = math.sqrt(radius * radius - (sliding_x_first - radius) ** 2)
#             e_point_y = math.sqrt(radius * radius - (sliding_x_end - radius) ** 2)
#             sliding_dis = math.sqrt((sliding_x_end - sliding_x_first) ** 2 + (e_point_y - s_point_y) ** 2)
#             if sliding_dis >= gap_dis:  # 这个想办法统一一下
#                 # 添加进list
#                 curve_list.append((sliding_x_end, e_point_y))
#                 sliding_x_first = sliding_x_end
#             sliding_x_end += 0.01
#         curve_list.append((3 / 2 * radius, math.sqrt(3 / 4 * radius * radius)))
#         print(curve_list)
#
#         p_0 = (0, 0)
#         p_neg_1 = (p_0[0] - path_point_redundant, p_0[1] - path_point_redundant)
#         p_neg_2 = (p_neg_1[0] - path_point_redundant, p_neg_1[1] - path_point_redundant)
#         p_1_0 = (radius / 4, math.sqrt(7) / 4 * radius)
#         p_1 = (radius / 2, math.sqrt(3 / 4 * radius * radius))
#         p_2 = (radius, radius)
#         p_3 = (3 / 2 * radius, math.sqrt(3 / 4 * radius * radius))
#         p_4 = (p_3[0] + path_point_redundant, p_3[1] + path_point_redundant)
#         p_5 = (p_4[0] + path_point_redundant, p_4[1] + path_point_redundant)
#         self.p_list = [p_neg_2, p_neg_1, p_0, p_1_0, p_1, p_2, p_3, p_4, p_5]
#         self.p_list = curve_list
#         self.p_list_left = []
#         for i in self.p_list:
#             self.p_list_left.append((-i[0], i[1]))
#         self.p_list_left_of_next_line = []
#         for i in self.p_list_left:
#             self.p_list_left_of_next_line.append((i[0] + radius, i[1]))
#         self.p_list_left_of_next_line.reverse()
#
#         self.p_list_right_of_next_line = []
#         for i in self.p_list:
#             self.p_list_right_of_next_line.append((i[0] - radius, i[1]))
#         self.p_list_right_of_next_line.reverse()
#
#     def curve_three_generate(self, floder_name: str, start_index: int, seq_dis: float, extending_direction_left: bool):
#         # 把生成的文件保存下来
#         # 生成utm坐标点序列
#         # utm_p_x_list = []
#         # utm_p_y_list = []
#
#         # 写第一段掉头曲线前进
#         with open(floder_name + "/" + str(start_index + 1) + "_掉头前进离开直线.txt", 'w') as file_open:
#             # 写表头
#             file_open.write("x,y,lon,lat\n")
#             if extending_direction_left:  # True左，False右
#                 mid_p_list = self.p_list_left  # 左侧离开直线
#             else:
#                 mid_p_list = self.p_list  # 右侧离开直线
#
#             for orig_p in mid_p_list:
#                 # 先旋转，再平移
#                 x_skim = orig_p[0]
#                 y_skim = orig_p[1]
#
#                 # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
#                 # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)
#
#                 # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
#                 # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]
#
#                 utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
#                 utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
#
#                 lon, lat = xy2ll(utm_p_x, utm_p_y)
#                 # utm_p_x_list.append(utm_p_x)
#                 # utm_p_y_list.append(utm_p_y)
#                 # file_open.write(str(lon)+","+str(lat)+"\n")
#                 file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
#
#             file_open.close()
#
#         # 写第二段即掉头直线后退
#         with open(floder_name + "/" + str(start_index + 2) + "_掉头直线后退.txt", 'w') as file_open:
#             # 写表头
#             file_open.write("x,y,lon,lat\n")
#             # 得到起止点
#             start_point = self.p_list[-1]
#             end_point = self.p_list_left_of_next_line[0]
#
#             start_point_neg_1 = (start_point[0] + 0.01, start_point[1])
#             start_point_neg_2 = (start_point_neg_1[0] + 0.01, start_point[1])
#             end_point_neg_1 = (end_point[0] - 0.01, end_point[1])
#             end_point_neg_2 = (end_point_neg_1[0] - 0.01, end_point[1])
#
#             if extending_direction_left:  # True左，False右
#                 start_point = self.p_list_left[-1]
#                 end_point = self.p_list_right_of_next_line[0]
#
#                 start_point_neg_1 = (start_point[0] - 0.01, start_point[1])
#                 start_point_neg_2 = (start_point_neg_1[0] - 0.01, start_point[1])
#                 end_point_neg_1 = (end_point[0] + 0.01, end_point[1])
#                 end_point_neg_2 = (end_point_neg_1[0] + 0.01, end_point[1])
#
#             ab_list = [start_point_neg_2, start_point_neg_1] + ab_line_generate_1_0.ab_line_seq(start_point, end_point,
#                                                                                                 seq_dis) \
#                       + [end_point_neg_1, end_point_neg_2]
#             # 直线生成与保存，需要生成多余点
#             for orig_p in ab_list:
#                 # 先旋转，再平移
#                 x_skim = orig_p[0]
#                 y_skim = orig_p[1]
#
#                 # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
#                 # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]
#
#                 utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
#                 utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
#                 lon, lat = xy2ll(utm_p_x, utm_p_y)
#                 # utm_p_x_list.append(utm_p_x)
#                 # utm_p_y_list.append(utm_p_y)
#                 # file_open.write(str(lon)+","+str(lat)+"\n")
#                 file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
#             file_open.close()
#
#         # 写第三段即掉头曲线前进
#         with open(floder_name + "/" + str(start_index + 3) + "_掉头前进靠近直线.txt", 'w') as file_open:
#             # 写表头
#             file_open.write("x,y,lon,lat\n")
#
#             if extending_direction_left:
#                 mid_p_list = self.p_list_right_of_next_line  # 左侧靠近直线
#             else:
#                 mid_p_list = self.p_list_left_of_next_line  # 左侧离开直线
#
#             for orig_p in mid_p_list:
#                 # 先旋转，再平移
#                 x_skim = orig_p[0]
#                 y_skim = orig_p[1]
#
#                 # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
#                 # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)
#
#                 # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
#                 # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]
#
#                 utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
#                 utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
#                 lon, lat = xy2ll(utm_p_x, utm_p_y)
#                 # utm_p_x_list.append(utm_p_x)
#                 # utm_p_y_list.append(utm_p_y)
#                 # file_open.write(str(lon)+","+str(lat)+"\n")
#                 file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
#
#             file_open.close()
#
#     def lines_generate_and_save(self, floder_path: str, start_index: int, seq_dis: float,
#                                 start_extending_direction: str,
#                                 breadth: float, lines_num: int):
#         """
#             进行直线路径生成和保存，输入幅宽、条数、作业行生成方向
#         :param floder_path:保存文件路径
#         :param start_index:
#         :param seq_dis:相邻点距
#         :param start_extending_direction:作业行方向
#         :param breadth:幅宽
#         :param lines_num:条数
#         :return:
#         """
#         # 211004继续
#         # 生成n个A、B点


class Turn_Around_Mission_Path_Generate(object):
    def __init__(self, A, B, radius, beta, line_width, turn_type: str):
        """

        :param A:
        :param B:
        :param radius:
        :param beta:
        :param line_width:幅宽，为正
        """
        self.A = A
        self.B = B
        self.len_AB = math.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2)
        self.radius = radius
        self.beta = beta
        path_point_redundant = 0.01  # m
        self.path_point_redundant = path_point_redundant
        self.line_width = line_width

        # 尝试生成曲线的离散化点，按固定distance来取
        curve_list = [(0, 0)]
        sliding_x_first = 0
        sliding_x_end = 0
        gap_dis = 0.5  # m,相邻点间隔 20211017 debug 0.5改0.25

        round_shape = 0
        if turn_type == "loop":
            round_shape = 1
        if turn_type == "fishtail":
            round_shape = 1.5
        while sliding_x_end <= round_shape * radius:
            s_point_y = math.sqrt(radius * radius - (sliding_x_first - radius) ** 2)
            e_point_y = math.sqrt(radius * radius - (sliding_x_end - radius) ** 2)
            sliding_dis = math.sqrt((sliding_x_end - sliding_x_first) ** 2 + (e_point_y - s_point_y) ** 2)
            if sliding_dis >= gap_dis:  # 这个想办法统一一下
                # 添加进list
                curve_list.append((sliding_x_end, e_point_y))
                sliding_x_first = sliding_x_end
            sliding_x_end += 0.01
        if turn_type == "loop":
            curve_list.append((radius, radius))
        if turn_type == "fishtail":
            curve_list.append((3 / 2 * radius, math.sqrt(3 / 4 * radius * radius)))

        print(curve_list)

        p_0 = (0, 0)
        p_neg_1 = (p_0[0] - path_point_redundant, p_0[1] - path_point_redundant)
        p_neg_2 = (p_neg_1[0] - path_point_redundant, p_neg_1[1] - path_point_redundant)
        p_1_0 = (radius / 4, math.sqrt(7) / 4 * radius)
        p_1 = (radius / 2, math.sqrt(3 / 4 * radius * radius))
        p_2 = (radius, radius)
        p_3 = (3 / 2 * radius, math.sqrt(3 / 4 * radius * radius))
        p_4 = (p_3[0] + path_point_redundant, p_3[1] + path_point_redundant)
        p_5 = (p_4[0] + path_point_redundant, p_4[1] + path_point_redundant)
        self.p_list = [p_neg_2, p_neg_1, p_0, p_1_0, p_1, p_2, p_3, p_4, p_5]

        # 更改成贝塞尔曲线，两个方案切换,注意检查是否完全替换.贝塞不行20211019
        # 更改成 椭圆
        # curve_list = [(0, 0)]
        # a = 7.5
        # b = 6.3
        # x_list = [i for i in np.arange(0, a, 0.01)]
        # y_list = []
        # for i in x_list:
        #     y_point = math.sqrt((1 - (i - a) ** 2 / a ** 2) * b ** 2)
        #     # y0_point = math.sqrt(49-i**2)
        #     y_list.append(y_point)
        #
        # x_len = len(x_list)
        # start_point = [0, 0]
        # for i in range(x_len):
        #     end_point = [x_list[i], y_list[i]]
        #     sliding_dis = math.sqrt((end_point[1] - start_point[1]) ** 2 + (end_point[0] - start_point[0]) ** 2)
        #     if sliding_dis >= gap_dis:
        #         curve_list.append((x_list[i], y_list[i]))
        #         start_point = end_point
        #



        # x_list, y_list = bezier_gen.return_bz(x_list,y_list)
        # sliding_x_end = 0
        # x_len = len(x_list)
        # start_point = [0,0]
        # for i in range(x_len):
        #     #
        #     end_point = [x_list[i], y_list[i]]
        #     sliding_dis = math.sqrt((end_point[1]-start_point[1])**2+(end_point[0]-start_point[0])**2)
        #     if sliding_dis>=gap_dis:
        #         curve_list.append((x_list[i], y_list[i]))
        #         start_point = end_point

        # while sliding_x_end <= 7.1:
        #     # s_point_y = math.sqrt(radius * radius - (sliding_x_first - radius) ** 2)
        #     # e_point_y = math.sqrt(radius * radius - (sliding_x_end - radius) ** 2)
        #     # sliding_dis = math.sqrt((sliding_x_end - sliding_x_first) ** 2 + (e_point_y - s_point_y) ** 2)
        #     # if sliding_dis >= gap_dis:  # 这个想办法统一一下
        #     #     # 添加进list
        #     #     curve_list.append((sliding_x_end, e_point_y))
        #     #     sliding_x_first = sliding_x_end
        #     if
        #     sliding_x_end += 0.01

        self.p_list = curve_list  # 1号曲线，离开直线

        self.p_list_left = []
        for i in self.p_list:
            self.p_list_left.append((-i[0], i[1]))  # 2号曲线，离开直线

        # self.p_list_left_of_next_line = []
        # self.p_list_right_of_next_line = []

        self.p_list_left_of_next_line = []
        for i in self.p_list_left:
            self.p_list_left_of_next_line.append((i[0] + line_width, i[1]))
        self.p_list_left_of_next_line.reverse()  # 1_skim号曲线，靠近下一作业行

        self.p_list_right_of_next_line = []
        for i in self.p_list:
            self.p_list_right_of_next_line.append((i[0] - line_width, i[1]))
        self.p_list_right_of_next_line.reverse()  # 2_skim号曲线，靠近下一作业行

    def curve_three_generate(self, floder_name: str, start_index: int, seq_dis: float, extending_direction_left: bool):
        # 把生成的文件保存下来
        # 生成utm坐标点序列
        # utm_p_x_list = []
        # utm_p_y_list = []

        # if not extending_direction_left:    # 为右则取反
        #     self.line_width = -self.line_width
        #
        # for i in self.p_list_left:
        #     self.p_list_left_of_next_line.append((i[0] - self.line_width, i[1]))
        # self.p_list_left_of_next_line.reverse()  # 1_skim号曲线，靠近下一作业行
        #
        # for i in self.p_list:
        #     self.p_list_right_of_next_line.append((i[0] + self.line_width, i[1]))
        # self.p_list_right_of_next_line.reverse()  # 2_skim号曲线，靠近下一作业行

        # 写第一段掉头曲线前进
        with open(floder_name + "/" + str(start_index + 1) + "_掉头前进离开直线.txt", 'w') as file_open:
            # 写表头
            file_open.write("x,y,lon,lat\n")
            if extending_direction_left:  # True左，False右
                mid_p_list = self.p_list_left  # 左侧离开直线
            else:
                mid_p_list = self.p_list  # 右侧离开直线

            for orig_p in mid_p_list:
                # 先旋转，再平移
                x_skim = orig_p[0]
                y_skim = orig_p[1]

                # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
                # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)

                # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
                # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]

                utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
                utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]

                lon, lat = xy2ll(utm_p_x, utm_p_y)
                # utm_p_x_list.append(utm_p_x)
                # utm_p_y_list.append(utm_p_y)
                # file_open.write(str(lon)+","+str(lat)+"\n")
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")

            file_open.close()

        # 写第二段即掉头直线后退
        with open(floder_name + "/" + str(start_index + 2) + "_掉头直线后退.txt", 'w') as file_open:
            # 写表头
            file_open.write("x,y,lon,lat\n")
            # 得到起止点
            start_point = self.p_list[-1]
            end_point = self.p_list_left_of_next_line[0]

            start_point_neg_1 = (start_point[0] + 0.01, start_point[1])
            start_point_neg_2 = (start_point_neg_1[0] + 0.01, start_point[1])
            end_point_neg_1 = (end_point[0] - 0.01, end_point[1])
            end_point_neg_2 = (end_point_neg_1[0] - 0.01, end_point[1])

            if extending_direction_left:  # True左，False右
                start_point = self.p_list_left[-1]
                end_point = self.p_list_right_of_next_line[0]

                start_point_neg_1 = (start_point[0] - 0.01, start_point[1])
                start_point_neg_2 = (start_point_neg_1[0] - 0.01, start_point[1])
                end_point_neg_1 = (end_point[0] + 0.01, end_point[1])
                end_point_neg_2 = (end_point_neg_1[0] + 0.01, end_point[1])

            # ab_list = [start_point_neg_2, start_point_neg_1] + ab_line_generate_1_0.ab_line_seq(start_point, end_point,
            #                                                                                     seq_dis) \
            #           + [end_point_neg_1, end_point_neg_2]
            # 不生成多余点 debug 20211019
            ab_list = ab_line_generate_1_0.ab_line_seq(start_point, end_point, seq_dis)

            # 直线生成与保存，需要生成多余点
            for orig_p in ab_list:
                # 先旋转，再平移
                x_skim = orig_p[0]
                y_skim = orig_p[1]

                # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
                # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]

                utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
                utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
                lon, lat = xy2ll(utm_p_x, utm_p_y)
                # utm_p_x_list.append(utm_p_x)
                # utm_p_y_list.append(utm_p_y)
                # file_open.write(str(lon)+","+str(lat)+"\n")
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")
            file_open.close()

        # 写第三段即掉头曲线前进
        with open(floder_name + "/" + str(start_index + 3) + "_掉头前进靠近直线.txt", 'w') as file_open:
            # 写表头
            file_open.write("x,y,lon,lat\n")

            if extending_direction_left:
                mid_p_list = self.p_list_right_of_next_line  # 左侧靠近直线 2_skim号曲线，靠近下一作业行
            else:
                mid_p_list = self.p_list_left_of_next_line  # 左侧离开直线 1_skim号曲线，靠近下一作业行

            for orig_p in mid_p_list:
                # 先旋转，再平移
                x_skim = orig_p[0]
                y_skim = orig_p[1]

                # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
                # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)

                # utm_p_x = x_skim * math.cos(self.beta) + y_skim * math.sin(self.beta) + self.B[0]
                # utm_p_y = y_skim * math.cos(self.beta) - x_skim * math.sin(self.beta) + self.B[1]

                utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
                utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
                lon, lat = xy2ll(utm_p_x, utm_p_y)
                # utm_p_x_list.append(utm_p_x)
                # utm_p_y_list.append(utm_p_y)
                # file_open.write(str(lon)+","+str(lat)+"\n")
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")

            file_open.close()

    def curve_loop_generate(self, floder_name: str, start_index: int, seq_dis: float, extending_direction_left: bool,
                            across_line_num: int):
        """
            套圈任务生成
        :param floder_name:
        :param start_index:
        :param seq_dis:
        :param extending_direction_left:
        :param across_line_num:跨越条带数量
        :return:
        """
        # 加偏
        jia_pian = 0

        # 对1_skim,2_skim曲线更新
        p_list_left_of_next_line = []
        for i in self.p_list_left:
            p_list_left_of_next_line.append((i[0] + across_line_num * line_width+jia_pian, i[1]))
        p_list_left_of_next_line.reverse()  # 1_skim号曲线，靠近下一作业行

        p_list_right_of_next_line = []
        for i in self.p_list:
            p_list_right_of_next_line.append((i[0] - across_line_num * line_width-jia_pian, i[1]))
        p_list_right_of_next_line.reverse()  # 2_skim号曲线，靠近下一作业行

        # 写套圈曲线前进，通过三段拼接而来
        with open(floder_name + "/" + str(start_index + 1) + "_掉头前进离开直线.txt", 'w') as file_open:
            # 写表头，第一条曲线生成
            file_open.write("x,y,lon,lat\n")
            if extending_direction_left:  # True左，False右
                mid_p_list = self.p_list_left  # 左侧离开直线
            else:
                mid_p_list = self.p_list  # 右侧离开直线

            for orig_p in mid_p_list:
                # 先旋转，再平移
                x_skim = orig_p[0]
                y_skim = orig_p[1]

                utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
                utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]

                lon, lat = xy2ll(utm_p_x, utm_p_y)
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")

            # 第二段直线段生成
            # 得到起止点
            start_point = self.p_list[-1]
            end_point = p_list_left_of_next_line[0]

            start_point_neg_1 = (start_point[0] + 0.01, start_point[1])
            start_point_neg_2 = (start_point_neg_1[0] + 0.01, start_point[1])
            end_point_neg_1 = (end_point[0] - 0.01, end_point[1])
            end_point_neg_2 = (end_point_neg_1[0] - 0.01, end_point[1])

            if extending_direction_left:  # True左，False右
                start_point = self.p_list_left[-1]
                end_point = p_list_right_of_next_line[0]

                start_point_neg_1 = (start_point[0] - 0.01, start_point[1])
                start_point_neg_2 = (start_point_neg_1[0] - 0.01, start_point[1])
                end_point_neg_1 = (end_point[0] + 0.01, end_point[1])
                end_point_neg_2 = (end_point_neg_1[0] + 0.01, end_point[1])

            # ab_list = [start_point_neg_2, start_point_neg_1] + ab_line_generate_1_0.ab_line_seq(start_point,
            #                                                                                     end_point,
            #                                                                                     seq_dis) \
            #           + [end_point_neg_1, end_point_neg_2]
            # 不生成多余的点debug
            ab_list = ab_line_generate_1_0.ab_line_seq(start_point, end_point, seq_dis)
            # 直线生成与保存，需要生成多余点
            for orig_p in ab_list:
                # 先旋转，再平移
                x_skim = orig_p[0]
                y_skim = orig_p[1]
                utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
                utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
                lon, lat = xy2ll(utm_p_x, utm_p_y)
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")

            # 第三段曲线生成
            if extending_direction_left:
                mid_p_list = p_list_right_of_next_line  # 左侧靠近直线 2_skim号曲线，靠近下一作业行
            else:
                mid_p_list = p_list_left_of_next_line  # 左侧离开直线 1_skim号曲线，靠近下一作业行

            for orig_p in mid_p_list:
                # 先旋转，再平移
                x_skim = orig_p[0]
                y_skim = orig_p[1]
                utm_p_x = x_skim * math.cos(-self.beta) - y_skim * math.sin(-self.beta) + self.B[0]
                utm_p_y = y_skim * math.cos(-self.beta) + x_skim * math.sin(-self.beta) + self.B[1]
                lon, lat = xy2ll(utm_p_x, utm_p_y)
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")

            file_open.close()


def mission_path_points_produce(A: Tuple[float, float], B: Tuple[float, float], extending_direction: bool,
                                line_width: float, turn_radius: float,
                                folder_name: str, start_index: int, line_seq_dis: float, turn_type: str,
                                across_line_num: int):
    # 输入A,B两点，及线条延伸方向（AB向量左、右），幅宽、保存的文件夹，输出每个任务序列的路径点到文件里面,把经纬度也保存下来，方便检验
    # 先生成AB线路径，保存为起始序号
    ab_list = ab_line_generate_1_0.ab_line_seq(A, B, line_seq_dis)
    with open(folder_name + "/" + str(start_index) + '_直行作业.txt', 'w') as file_open:
        file_open.write("x,y,lon,lat\n")
        for i_ab_list in ab_list:
            i_x = i_ab_list[0]
            i_y = i_ab_list[1]
            i_lon, i_lat = xy2ll(i_x, i_y)
            file_open.write(str(i_x) + "," + str(i_y) + "," + str(i_lon) + "," + str(i_lat) + "\n")
        file_open.close()

    AB_vector = (B[0] - A[0], B[1] - A[1])
    beta_ridian = calcAngOfY_Axis(AB_vector) / 360 * 2 * math.pi
    mission_path = Turn_Around_Mission_Path_Generate(A, B, turn_radius, beta_ridian, line_width, turn_type)
    if turn_type == "fishtail":
        mission_path.curve_three_generate(folder_name, start_index, line_seq_dis, extending_direction)
    if turn_type == "loop":
        mission_path.curve_loop_generate(folder_name, start_index, line_seq_dis, extending_direction,
                                         across_line_num=across_line_num)
    else:
        print("turn type error")
    del mission_path


def geometric2utm(geo_point: tuple, datum_point: tuple, beta_ridian: float):
    """
        几何图形转utm坐标点
    :param geo_point:   根据想要的几何形状在直角坐标系中确定的点，通常在原点附近，根据原点和偏移来表示几何中的点
    :param datum_point: 变换基准点，指的是AB线中的B点，为UTM坐标系中的点
    :param beta_ridian: AB向量相对于正北方向的角度，0~360°
    :return:
    """
    x_skim = geo_point[0]
    y_skim = geo_point[1]
    utm_p_x = x_skim * math.cos(-beta_ridian) - y_skim * math.sin(-beta_ridian) + datum_point[0]
    utm_p_y = y_skim * math.cos(-beta_ridian) + x_skim * math.sin(-beta_ridian) + datum_point[1]
    utm_point = (utm_p_x, utm_p_y)
    return utm_point


def fishtail_auto_task_gen(A: tuple, B: tuple, extending_direction_left: bool, line_width: float, turn_radius: float,
                           folder_name: str, line_num: int):
    AB_distance = math.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2)
    AB_vector = (B[0] - A[0], B[1] - A[1])
    beta_ridian = calcAngOfY_Axis(AB_vector) / 360 * 2 * math.pi  # 旋转角度
    loc_line_width = line_width
    if extending_direction_left:
        loc_line_width = -line_width
    for i_line_num in range(line_num):
        # 当前所有ab线方向一致
        B_x = loc_line_width * i_line_num  # 第i_line_num条ab线
        A_x = loc_line_width * i_line_num
        A_ori = (A_x, -AB_distance)  # 把A点移到原点
        A_planning = geometric2utm(geo_point=A_ori, datum_point=B, beta_ridian=beta_ridian)

        B_ori = (B_x, 0)  # 把B点移到原点
        B_planning = geometric2utm(geo_point=B_ori, datum_point=B, beta_ridian=beta_ridian)

        if i_line_num % 2 != 0:  # 奇数倒置
            mid = B_planning
            B_planning = A_planning
            A_planning = mid

        mission_path_points_produce(A_planning, B_planning, extending_direction_left, line_width, turn_radius,
                                    folder_name, start_index=4 * i_line_num,
                                    line_seq_dis=0.5, turn_type="fishtail", across_line_num=0)
        extending_direction_left = bool(1 - extending_direction_left)


def loop_auto_task_gen(A: tuple, B: tuple, extending_direction_left: bool, line_width: float, turn_radius: float,
                       folder_name: str, line_num: int, ab_line_seq: list, group_seq_num: int):
    """

    :param A:
    :param B:
    :param extending_direction_left:
    :param line_width:
    :param turn_radius:
    :param folder_name:
    :param line_num: 当前假设行数刚好为一个套圈组
    :return:
    """

    # AB倒置
    # mid_AB = B
    # B = A
    # A = mid_AB

    AB_distance = math.sqrt((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2)
    AB_vector = (B[0] - A[0], B[1] - A[1])
    beta_ridian = calcAngOfY_Axis(AB_vector) / 360 * 2 * math.pi  # 旋转角度

    loc_line_width = line_width
    if extending_direction_left:
        loc_line_width = -line_width
    # 假设有[0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5]
    # ab_line_seq = [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]  # 默认向右
    ab_line_seq = ab_line_seq

    # 验证平移
    # ab_line_seq = [i + 11 for i in ab_line_seq]

    len_seq = len(ab_line_seq)
    for i in range(len_seq):
        if i == len_seq - 1:
            # 最后一项只用生成直线点序列
            break
            pass

        # 普通项需要生成直线和掉头区序列点
        # i, i+1分别为一组ab线的下标
        # 确定方向，向右的话就全部向右边
        loc_extending_direction_left = extending_direction_left
        # if not extending_direction_left:
        #     if ab_line_seq[i + 1] > ab_line_seq[i]:
        #         loc_extending_direction_left = False
        #     else:
        #         loc_extending_direction_left = True
        # else:
        #     if ab_line_seq[i + 1] > ab_line_seq[i]:
        #         loc_extending_direction_left = True
        #     else:
        #         loc_extending_direction_left = False

        # 生成几何AB点
        i_line_num = ab_line_seq[i]
        # print("i_line_num", i_line_num)
        # 加偏移
        delta_pian = -0
        B_x = loc_line_width * i_line_num  # 第i_line_num条ab线
        A_x = loc_line_width * i_line_num

        if i % 2 != 0:  # 奇数倒置
            # 右偏
            A_ori = (A_x-delta_pian, -AB_distance)  # 把A点移到原点
            A_planning = geometric2utm(geo_point=A_ori, datum_point=B, beta_ridian=beta_ridian)
            B_ori = (B_x-delta_pian, 0)  # 把B点移到原点
            B_planning = geometric2utm(geo_point=B_ori, datum_point=B, beta_ridian=beta_ridian)
        else:
            A_ori = (A_x + delta_pian, -AB_distance)  # 把A点移到原点
            A_planning = geometric2utm(geo_point=A_ori, datum_point=B, beta_ridian=beta_ridian)
            B_ori = (B_x + delta_pian, 0)  # 把B点移到原点
            B_planning = geometric2utm(geo_point=B_ori, datum_point=B, beta_ridian=beta_ridian)

        if i % 2 != 0:  # 奇数倒置
            mid = B_planning
            B_planning = A_planning
            A_planning = mid

        # debug 20211017 0.5改成0.25
        mission_path_points_produce(A_planning, B_planning, loc_extending_direction_left, line_width, turn_radius,
                                    folder_name, start_index=2 * i + group_seq_num * 2 * (len_seq - 1),
                                    line_seq_dis=0.5, turn_type="loop",
                                    across_line_num=abs(ab_line_seq[i + 1] - ab_line_seq[i]))

    # for i_line_num in range(line_num):
    #     # 当前所有ab线方向一致
    #     B_x = loc_line_width * i_line_num  # 第i_line_num条ab线
    #     A_x = loc_line_width * i_line_num
    #     A_ori = (A_x, -AB_distance)  # 把A点移到原点
    #     A_planning = geometric2utm(geo_point=A_ori, datum_point=B, beta_ridian=beta_ridian)
    #
    #     B_ori = (B_x, 0)  # 把B点移到原点
    #     B_planning = geometric2utm(geo_point=B_ori, datum_point=B, beta_ridian=beta_ridian)
    #
    #     if i_line_num % 2 != 0:  # 奇数倒置
    #         mid = B_planning
    #         B_planning = A_planning
    #         A_planning = mid
    #
    #     mission_path_points_produce(A_planning, B_planning, extending_direction_left, line_width, turn_radius,
    #                                 folder_name, start_index=4 * i_line_num,
    #                                 line_seq_dis=0.5)
    #     extending_direction_left = bool(1 - extending_direction_left)
    # pass


def loop_group_gen(A: tuple, B: tuple, extending_direction_left: bool, line_width: float, turn_radius: float,
                   folder_name: str, line_num: int, ab_line_seq, group_num: int):
    ab_line_seq = ab_line_seq
    ab_line_len = len(ab_line_seq)
    loc_extending_direction_left = extending_direction_left

    for i_group_num in range(group_num):
        # 平移
        ab_line_seq = [i + i_group_num * (ab_line_len - 1) for i in ab_line_seq]

        # 奇数倒置
        if i_group_num % 2 != 0:
            mid_AB = B
            B = A
            A = mid_AB

        loop_auto_task_gen(A, B, loc_extending_direction_left, line_width, turn_radius, folder_name, line_num,
                           ab_line_seq, group_seq_num=i_group_num)
        loc_extending_direction_left = bool(1 - loc_extending_direction_left)

    # loop_auto_task_gen(A, B, extending_direction_left, line_width, turn_radius, folder_name, line_num)


if __name__ == '__main__':
    # A = (488086.61527862446, 4466473.786561005)
    # B = (488087.6214051248, 4466504.71990286)
    # 人工驾驶
    A = (487790.427+1.5, 4466493.669)
    B = (487790.427+1.5, 4466117.106-7)

    A = (488088.29591353226,4466105.6460556965)
    B = (488172.0291863239,4466096.9603796685)
    A=(487857.2399236554,4466510.894873215)
    B= (487858.075342892,4466465.546697163)

    # 无人驾驶区域东侧AB点
    # A = (488016.646 - 1.5, 4466539.41-7)
    # B = (488016.646 - 1.5, 4466206.16-7)

    # # 无人驾驶区域西侧AB点
    # A = (487944.67+1.5, 4466539.41-7)
    # B = (487944.67+1.5, 4466206.16+7)
    #
    # # 辅助驾驶区东侧AB点
    # A = (487931.00-1.5, 4466539.41-7)
    # B = (487932.43-1.5,4466118.08)
    #
    # # 辅助驾驶区西侧AB点
    # A = (487861.39+1.5, 4466539.41-7)
    # B = (487861.38+1.5, 4466118.07)

    # 南边小试验田AB点
    # A = (488099.46370352106,4466119.951443543)
    # B = (488174.85143155715,4466111.5831486415)

    # B = (487944.7, 4466206)
    # A = (488016.6, 4466206)
    # B = (488235.0488366564, 4466139.434445432)
    # A = (488092.4020280437, 4466151.642338371)
    #
    # B = (488058.1817552213, 4466480.154091353)
    # A = (488051.6553336895, 4466274.486571828)
    # ab_list = ab_line_generate_1_0.ab_line_seq(A, B, 0.5)
    # print(ab_list)
    extending_direction_left = True  # True表示左，False表示右
    line_width = 3.0  # m 看看是不是有效幅宽
    turn_radius = 7.5  # m 转弯半径 6.3试试
    folder_name = "mission_path_generate_test"
    # mission_path_points_produce(A, B, extending_direction_left, line_width, turn_radius, folder_name, start_index=0, line_seq_dis=0.5)

    # 20211009准备写下掉头作业的
    # 生成line_num条ab线，全部规划出来
    start_time = time.time()
    line_num = 20  # 暂时无用20211019
    ab_line_seq = [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]  # 当前此值为常量，更改会有问题
    # ab_line_seq = [0, 12, 1, 13, 2, 14, 3, 15, 4, 16, 5, 17, 6, 18, 7, 19, 8, 20, 9, 21, 10, 22, 11, 23]    # 24个条带

    # fishtail_auto_task_gen(A, B, extending_direction_left, line_width, turn_radius, folder_name, line_num)
    # loop_auto_task_gen(A, B, extending_direction_left, line_width, turn_radius, folder_name, line_num, ab_line_seq,group_seq_num=0)
    loop_group_gen(A, B, extending_direction_left, line_width, turn_radius, folder_name, line_num, ab_line_seq,
                   group_num=2)

    end_time = time.time()
    print("the time cost is %f", end_time - start_time)
