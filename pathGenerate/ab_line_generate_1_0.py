# 生成符合要求间隔的直线路径点序列
# (444502.86203611083, 4428582.758912923)
# (444508.74640294677, 4428476.713391734)
# n=213
import math

def ab_line_seq(a_point, b_point, seq_distance):
    """
        直线序列生成工具
    :param a_point:A点
    :param b_point:B点
    :param seq_distance:相邻两点间距,单位m
    :return:类型：list，内容：序列点集
    """
    # sep_distance = 0.5  # 相邻路径点距离，单位m
    # a_point = (444502.86203611083, 4428582.758912923)
    # b_point = (444508.74640294677, 4428476.713391734)
    x1 = a_point[0]
    y1 = a_point[1]
    x2 = b_point[0]
    y2 = b_point[1]
    # 确定分割点数n（整型）
    n = math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) / seq_distance
    n = n + 1
    n = int(n)
    # print(n)

    ab_path_list = []
    for i in range(n + 1):
        ab_path_list.append((x1 + (x2 - x1) / n * i, y1 + (y2 - y1) / n * i))

    # print(ab_path_list)  # 当前一个条带的点密度对于python list来说是一点问题没有的
    return ab_path_list


if __name__ == "__main__":
    # 这样分起终点一定没有问题的
    ab_list = ab_line_seq((444502.86203611083, 4428582.758912923), (444508.74640294677, 4428476.713391734), 0.5)
    print(ab_list)




