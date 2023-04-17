# @Time    : 2021/9/10 上午8:38
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : exception_handling_func.py
# @Software: PyCharm
# 测试下异常处理
import component_main.component.calc_point2line as calc_p2l
import math

def calcAngOfY_Axis(vector):
    """
        计算向量与Y轴的顺时针夹角，范围0~360°
    :param vector:
    :return: 返回两个向量的夹角
    """
    #endAng=0
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
        ang1 = 360-ang1
    # print("小车指向目标点向量与Y轴夹角=", ang1)
    return ang1

# angle_1 = calcAngOfY_Axis((1, 1))   # 固定
# angle_2 = calcAngOfY_Axis((2, 2))
# print(angle_2 - angle_1)

def calcAngOf2Vectors(vector_line, vector_car):
    return calcAngOfY_Axis(vector_car)-calcAngOfY_Axis(vector_line)



x = 2
y = 3

path_x_y = [(1, 1), (2, 2), (3, 3), (5, 5)]
path_target_point_index = 2

# 计算实时误差然后传递出去，让显示线程不断访问误差并显示(以下为直线计算)
# 暂时统一直线曲线的计算方式，使用异常处理，异常时误差显示为999
try:
    linear_tracking_error = calc_p2l.get_distance_from_point_to_line([x, y], [path_x_y[path_target_point_index - 1][0], path_x_y[path_target_point_index - 1][1]], [path_x_y[path_target_point_index][0], path_x_y[path_target_point_index][1]])
    start_point2vehicle = (x - path_x_y[path_target_point_index - 1][0], y - path_x_y[path_target_point_index - 1][1])
    start2end = (path_x_y[path_target_point_index][0] - path_x_y[path_target_point_index - 1][0], path_x_y[path_target_point_index][1] - path_x_y[path_target_point_index - 1][1])
    angle_error = calcAngOf2Vectors(start2end, start_point2vehicle)
    if 180 > angle_error >= 0 or -360 <= angle_error < -180:
        print(calcAngOf2Vectors(start2end, start_point2vehicle))
        # 右边，为负
        linear_tracking_error = -linear_tracking_error

except:
    linear_tracking_error = 9999

print(linear_tracking_error)
