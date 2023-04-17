# 暂定：计算点到直线距离
import numpy as np

def get_distance_from_point_to_line(point, line_point1, line_point2):
    """
        note:对于两点坐标为同一点时,返回点与点的距离
    :param point:[x, y]
    :param line_point1:[start_x, start_y]
    :param line_point2:[end_x, end_y]
    :return:
    """
    if line_point1 == line_point2:
        point_array = np.array(point)
        point1_array = np.array(line_point1)
        return np.linalg.norm(point_array -point1_array )
    # 计算直线的三个参数
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
        (line_point2[0] - line_point1[0]) * line_point1[1]
    # 根据点到直线的距离公式计算距离
    distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A**2 + B**2))
    return distance


if __name__ == "__main__":
    point = [1, 1]
    start_p = [0, -1]
    end_p = [0, 1]
    distance = get_distance_from_point_to_line(point, start_p, end_p)
    print(distance)
