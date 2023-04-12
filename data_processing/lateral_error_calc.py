# 进行直线度计算
import numpy as np
import processing_component.calc_p2l as calc_p2l
import matplotlib.pyplot as plt


# 后期抽象成工厂类，使其符合开闭原则
def lateral_error_calc(path_x_y, realtime_x, realtime_y):
    """
        计算车辆跟踪实时误差，输入utm路径点，车辆realtime utm坐标
    :param path_x_y: list，取起终点
    :param realtime_x:
    :param realtime_y:
    :return:
    """
    distance = calc_p2l.get_distance_from_point_to_line(point, start_point, end_point)



if __name__ == "__main__":
    # 读取ab线文件，得到始终点
    ab_file_path = "/home/wen/PycharmProjects/dongFengProj_1.1/pathGenerate/ab_record/"+"ab_line_2021-09-02_0"
    start_point = []
    end_point = []
    with open(ab_file_path+"/a.txt", "r") as a_read_open:
        with open(ab_file_path+"/b.txt", "r") as b_read_open:
            a_point = a_read_open.readline()
            a_point = a_point.split(",")
            a_point_x = float(a_point[0])
            a_point_y = float(a_point[1])

            b_point = b_read_open.readline()
            b_point = b_point.split(",")
            b_point_x = float(b_point[0])
            b_point_y = float(b_point[1])

            start_point.append(a_point_x)
            start_point.append(a_point_y)
            end_point.append(b_point_x)
            end_point.append(b_point_y)

            b_read_open.close()
        a_read_open.close()


    start_point = [488104.92, 4466137.46]
    end_point = [488161.96, 4466131.13]
    # with open("0_直行作业.txt", 'r') as file_open:
    #     file_open.close()
    # 使用一般式表达，才能表达所有直线

    straight_line_points_path = r"/home/wen/PycharmProjects/dongFengProj_1.1/pathRecord/ctrlCAN/2021-10-22/行驶记录数据2021-10-22 09-46-09.csv"
    with open(straight_line_points_path, 'r') as file_open:
        # 去表头
        file_open.readline()
        first_line = file_open.readline()
        first_line_comma = first_line.split(',')
        filename = first_line_comma[29]
        with open(filename, "r") as filename_open:
            filename_open.readline()
            filename_open_list = filename_open.readlines()
            filename_comma_0 = filename_open_list[0].split(",")
            filename_comma_1 = filename_open_list[-1].split(',')
            start_point = [float(filename_comma_0[0]), float(filename_comma_0[1])]
            end_point = [float(filename_comma_1[0]), float(filename_comma_1[1])]

            filename_open.close()
        #29

        file_open.close()
    # 加载点序列
    points_num = 0
    lateral_error_sum = 0
    max_lateral_error = 0

    x_axis = []
    y_axis = []
    distance_list = []

    # 提供utm_x最值
    max = start_point[0]
    min = end_point[0]
    if max < min:
        temp = max
        max = min
        min = temp
    with open(straight_line_points_path, 'r') as line_file_open:
        line_file_open.readline()  # 去表头
        data_list = line_file_open.readlines()
        # 取x，y点
        for i in range(len(data_list)):  # 4，5
            data_list_comma = data_list[i].split(",")
            x = float(data_list_comma[4])
            y = float(data_list_comma[5])

            # 增加对非AB线内点的判断
            if x < min or x > max:
                # 不是AB线内的点，则舍去
                continue

            x_axis.append(x)
            y_axis.append(y)

            point = [x, y]
            # 计算横向误差
            distance = calc_p2l.get_distance_from_point_to_line(point, start_point, end_point)
            distance_list.append(distance)
            print(distance)

            # 计算0.2以下点的个数
            # if distance <= 1:
            #     points_num = points_num + 1
            #     lateral_error_sum = lateral_error_sum + distance

            points_num = points_num + 1
            lateral_error_sum = lateral_error_sum + distance
            if max_lateral_error < distance:
                max_lateral_error = distance

    line_file_open.close()

    print("points_num=", points_num)
    print("lateral_error_sum=", lateral_error_sum)
    print("mean_lateral_error=", lateral_error_sum / points_num)
    print("max_lateral_error=", max_lateral_error)

    plt.figure(1)
    ax1 = plt.subplot(2, 1, 1)
    ax2 = plt.subplot(2, 1, 2)
    plt.sca(ax1)

    plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], label="AB_line")
    plt.plot(x_axis, y_axis, label="actual_driving")
    plt.legend(loc='upper left', bbox_to_anchor=(0, 0.95))
    plt.grid(color="k", linestyle=":")
    plt.title("tractor_selfdriving")
    plt.xlabel("utm_x m")
    plt.ylabel("utm_y m")

    plt.sca(ax2)
    plt.plot(x_axis, distance_list, label="lateral_error")
    plt.legend(loc='upper left', bbox_to_anchor=(0, 0.95))
    plt.grid(color="k", linestyle=":")
    plt.title("tractor_selfdriving_lateral_error 4km/h")
    plt.xlabel("utm_x m")
    plt.ylabel("lateral_error m")

    plt.show()


