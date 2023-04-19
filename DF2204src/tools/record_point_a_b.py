# 打AB点，并生成点集保存
import os, time
import math
import sys
#将component库添加到环境变量里以便导入模组
sys.path.append(os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)),"../component")))

from pathlib import Path
import classOfIMU_CGI610

def check_mkdir(folder_path):
    # 检查文件夹是否存在，若不存在，则创建
    is_exists = os.path.exists(folder_path)
    if not is_exists:
        os.makedirs(folder_path)
    else:
        print("path 存在")


def ab_line_generate(a_point, b_point, seq_distance):
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
    seq_distance = 0.5
    folder_path = "../data/point_a_b"+time.strftime("%Y-%m-%d", time.localtime())
    # 判断文件是否存在，不存在则创建
    check_mkdir(folder_path)
    # 判断是否有A文件
    my_file = Path(folder_path + "/a.txt")
    imu = classOfIMU_CGI610.Imu()
    if my_file.exists():
        print("have")
        # 开始打B点
        # 读取惯导数据，判断42，存入b.txt
        while 1:
            # 不断读取imu，然后保存为csv
            inte_navi_info = imu.stateOfCar()
            # 判断42
            if inte_navi_info[22] != 42:
                print("inte_navi_info[22]", inte_navi_info[22])
                time.sleep(1)
                continue
            else:
                # 记下b点，
                utm_x = inte_navi_info[0]
                utm_y = inte_navi_info[1]
                b_point = [utm_x, utm_y]
                with open(folder_path+"/b.txt", "w") as b_file_open:
                    b_file_open.writelines([str(utm_x)+","+str(utm_y)])
                    b_file_open.close()

                # 对ab两点连线进行序列化，并存储为csv格式
                a_point = [0, 0]
                with open(folder_path+"/a.txt", "r") as a_file_open:
                    a_content = a_file_open.readline()
                    a_list = a_content.split(",")
                    a_point[0] = float(a_list[0])
                    a_point[1] = float(a_list[1])
                    a_file_open.close()
                ab_list = ab_line_generate(a_point, b_point, seq_distance)
                print(ab_list)
                ab_list_namesuffix = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
                with open(folder_path+"/ab_list"+ab_list_namesuffix+".txt", "w") as ab_list_file_open:
                    # 保存为csv格式，写入列名
                    ab_list_file_open.write("utm_x, utm_y\n")
                    for i_ab_list in ab_list:
                        ab_x = i_ab_list[0]
                        ab_y = i_ab_list[1]
                        ab_list_file_open.write(str(ab_x)+","+str(ab_y)+"\n")
                    # ab_list_file_open.writelines(str(ab_list))
                    ab_list_file_open.close()

                # 修改文件夹名字
                os.rename(folder_path, "ab_record/ab_line_"+ab_list_namesuffix)

                break

            # msg = [time.time(), cmd_steering, cmd_v, path_target_point_index] + path_generate_inte_navi_info


    else:
        # 没有a文件，记录作为a点
        # 读取惯导数据，判断42，存入b.txt
        while 1:
            # 不断读取imu，然后保存为csv
            inte_navi_info = imu.stateOfCar()
            print(inte_navi_info[13], inte_navi_info[14]) # lat, lon
            # 判断42
            if inte_navi_info[22] != 42:
                print("inte_navi_info[22]", inte_navi_info[22])
                time.sleep(1)
                continue
            else:
                # 记下b点，
                utm_x = inte_navi_info[0]
                utm_y = inte_navi_info[1]
                with open(folder_path + "/a.txt", "w") as b_file_open:
                    b_file_open.writelines([str(utm_x)+","+str(utm_y)])
                    b_file_open.close()
                break
