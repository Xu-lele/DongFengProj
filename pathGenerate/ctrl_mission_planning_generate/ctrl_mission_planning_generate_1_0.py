# @Time    : 2021/10/4 下午6:00
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : ctrl_mission_planning_generate.py
# @Software: PyCharm
import math
import pathGenerate.ab_line_generate_1_0 as ab_line_generate_1_0
from pyproj import Proj, transform
from pyproj import Transformer
transformer = Transformer.from_crs("epsg:32650", "epsg:4326")

def xy2ll(x, y):
    # WGS84 = Proj(init='EPSG:4326')  # WGS84
    # p = Proj(init="EPSG:32650")  # UTM 50N  这个可能会影响到除北京外的作业
    # lon, lat = transform(p, WGS84, x, y)
    # return lon, lat
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


def lines_generate_and_save(A: tuple, B: tuple, folder_path: str, seq_dis: float,
                            start_extending_direction: str,
                            breadth: float, lines_num: int):
    """
        进行直线路径生成和保存，输入幅宽、条数、作业行生成方向
    :param folder_path:保存文件路径
    :param seq_dis:相邻点距
    :param start_extending_direction:作业行方向
    :param breadth:幅宽
    :param lines_num:条数
    :return:
    """
    # 生成n个A、B点作业任务
    AB_vector = (B[0] - A[0], B[1] - A[1])
    AB_distance = math.sqrt((B[0] - A[0])**2+(B[1] - A[1])**2)
    # AB_distance = 999
    beta_ridian = calcAngOfY_Axis(AB_vector) / 360 * 2 * math.pi  # 旋转角度
    if start_extending_direction == "left":
        breadth = -breadth

    for i in range(0, lines_num):
        # 每一行写一个文件
        with open(folder_path+str(i)+"_直行作业.txt", "w") as file_open:
            file_open.write("x,y,lon,lat\n")

            # 生成lines_num行,通过改变AB点实现
            B_x = breadth * i
            A_x = breadth * i
            A_ori = (A_x, -AB_distance)
            B_ori = (B_x, 0)
            # 根据两点和点距生成点序列
            ab_list = ab_line_generate_1_0.ab_line_seq(A_ori, B_ori, seq_dis)
            # 偶数不变，奇数倒置
            if i % 2 != 0:
                ab_list.reverse()

            for i_ab_list in ab_list:
                # 进行旋转平移得到实际作业点
                x_skim = i_ab_list[0]
                y_skim = i_ab_list[1]
                utm_p_x = x_skim * math.cos(-beta_ridian) - y_skim * math.sin(-beta_ridian) + B[0]
                utm_p_y = y_skim * math.cos(-beta_ridian) + x_skim * math.sin(-beta_ridian) + B[1]
                lon, lat = xy2ll(utm_p_x, utm_p_y)
                print(utm_p_x, utm_p_y, lon, lat)
                file_open.write(str(utm_p_x) + "," + str(utm_p_y) + "," + str(lon) + "," + str(lat) + "\n")

            file_open.close()

    # # 写入文件
    # with open(folder_path, "w") as file_open:
    #     # 写入表头
    #     file_open.write("x,y,lon,lat\n")
    #
    #
    #     file_open.close()

    # for i in range(0, lines_num + 1):
    #     if start_extending_direction == "left":
    #         breadth = -breadth
    #
    #     # 生成lines_num行,通过改变AB点实现
    #     B_x = B[0] + breadth * i
    #     A_x = A[0] + breadth * i
    #     A = (A_x, A[1])
    #     B = (B_x, B[1])
    #     # 根据两点和点距生成点序列
    #     ab_list = ab_line_generate_1_0.ab_line_seq(A, B, seq_dis)
    #     # 偶数不变，奇数倒置
    #     if i%2 != 0:
    #         ab_list.reverse()
    #
    #     for i_ab_list in ab_list:
    #         # 进行旋转平移得到实际作业点
    #         x_skim = i_ab_list[0]
    #         y_skim = i_ab_list[1]
    #         utm_p_x = x_skim * math.cos(-beta_ridian) - y_skim * math.sin(-beta_ridian) + B[0]
    #         utm_p_y = y_skim * math.cos(-beta_ridian) + x_skim * math.sin(-beta_ridian) + B[1]
    #         lon, lat = xy2ll(utm_p_x, utm_p_y)
    #         print(utm_p_x, utm_p_y, lon, lat)

    # # 根据两点和点距生成点序列
    # ab_list = ab_line_generate_1_0.ab_line_seq(A, B, seq_dis)
    # for i in ab_list:
    #     # 进行旋转平移得到实际作业点
    #     x_skim = i[0]
    #     y_skim = i[1]
    #     utm_p_x = x_skim * math.cos(-beta_ridian) - y_skim * math.sin(-beta_ridian) + B[0]
    #     utm_p_y = y_skim * math.cos(-beta_ridian) + x_skim * math.sin(-beta_ridian) + B[1]
    #     lon, lat = xy2ll(utm_p_x, utm_p_y)


if __name__ == '__main__':
    # 20211020生成辅助驾驶区域直线，在程序那里动手脚，运行两版不同的程序
    A = (487861.394, 4466529.41)
    B = (487861.377, 4466118.075)

    # 辅助驾驶区东侧AB点
    # A = (487931.00-1.5, 4466539.41-7)
    # B = (487932.43-1.5, 4466118.08)

    # 辅助驾驶区西侧AB点
    A = (487861.39+1.5, 4466539.41-7)
    B = (487861.38+1.5, 4466118.07)

    # 20211004经测试可用
    # A = (488099.46370352106, 4466119.951443543)
    # B = (488174.85143155715, 4466111.5831486415)
    # A = (488086.61527862446, 4466473.786561005)
    # B = (488087.6214051248, 4466504.71990286)
    # B = (488235.0488366564, 4466139.434445432)
    # A = (488092.4020280437, 4466151.642338371)

    # 北部开始，从左向右，暂定这个顺序
    lines_generate_and_save(A, B, "mission_path_generate_test1/", seq_dis=0.5, start_extending_direction="left", breadth=3.0, lines_num=40)
