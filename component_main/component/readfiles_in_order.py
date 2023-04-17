# @Time    : 2021/9/11 上午9:47
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : readfiles_in_order.py
# @Software: PyCharm
# 进行文件遍历读取
import os


def is_task_legacy(rootDir):
    # 进行文件遍历
    latest_date = [0, 0, 0]
    for root, dirs, files in os.walk(rootDir):
        # root表示根目录，dirs表示根目录下的目录，files表示根目录下的所有文件
        # 后期进行复杂度的降低
        date_rung = root.split("/")
        year_month_day = date_rung[-1].split("-")
        if len(year_month_day) == 3:
            # 表明是数据记录文件夹
            # 找到最新的文件并读取进来
            year = int(year_month_day[0])
            month = int(year_month_day[1])
            day = int(year_month_day[2])
            if year > latest_date[0]:
                latest_date[0] = year
                latest_date[1] = month
                latest_date[2] = day
            elif year == latest_date[0]:
                if month > latest_date[1]:
                    latest_date[0] = year
                    latest_date[1] = month
                    latest_date[2] = day
                elif month == latest_date[0]:
                    if day > latest_date[2]:
                        latest_date[0] = year
                        latest_date[1] = month
                        latest_date[2] = day

        # print(year_month_day)
        # print(date_rung)
        # print("root=", type(root))
        # print("dirs=", dirs)
        # print("files=", files)
        # for file in files:
        #     print(os.path.join(root, file))
    print(latest_date)
    latest_month = ""
    latest_day = ""
    # 合成文件路径，进行文件读取
    if latest_date[1] < 10:
        # 加0
        latest_month = "0" + str(latest_date[1])
    if latest_date[2] < 10:
        latest_day = "0" + str(latest_date[2])
    latest_date_name = str(latest_date[0]) + "-" + latest_month + "-" + latest_day
    print("latest_date_name=", latest_date_name)

    # 在该文件夹下遍历，访问最新文件
    state_info = os.stat(rootDir + "/" + latest_date_name)
    # print(state_info.st_size)
    # print(state_info.st_atime)
    print(state_info.st_mtime)  # 最后一次修改时间，如果使用write函数写某个文件，会改变文件的这个时间
    # print(state_info.st_ctime)

    # 对行驶记录文件进行遍历，展示时间前后
    max = 0
    for root, dirs, files in os.walk(rootDir + "/" + latest_date_name):
        for file in files:
            if file[0] == "行":
                # print(file)
                state_info = os.stat(rootDir + "/" + latest_date_name + "/" + file)
                # print(file, "--", state_info.st_mtime)
                if state_info.st_mtime > max:
                    max = state_info.st_mtime

        for file in files:
            if file[0] == "行":
                # 打开并读取该文件
                state_info = os.stat(rootDir + "/" + latest_date_name + "/" + file)
                # print(file, "--", state_info.st_mtime)
                if state_info.st_mtime == max:
                    # 进行该文件的读取判断
                    with open(rootDir + "/" + latest_date_name + "/" + file, "r") as latest_file_open:
                        latest_file_open.readline()
                        context = latest_file_open.readlines()
                        line = context[-1]
                        line_comma = line.split(",")
                        index = line_comma[3]
                        print(line_comma[3])
                        x_y_file = line_comma[-6]       # 这里无法应对行驶时数据记录文件格式变化20211001
                        x_y_path = "/home/wen/PycharmProjects/dongFengProj_1.1/pathGenerate/ctrl_mission_planning/" + x_y_file     # 该文件夹路径要做更改
                        # 读取该文件，获取行数，判断length-1
                        with open(x_y_path, "r") as xy_file_open:
                            xy_file_open.readline()
                            if int(index) < len(xy_file_open.readlines()) - 1:  # 运行时增加了末尾两点，这里直接判断最后一个点
                                print("任务未完成，继续旧任务")
                                # 获取到未完成任务的文件和跟踪点index，传递给ctrl任务规划模块，然后下发给path_following执行。
                                # 未完成任务的文件和跟踪点index
                                return [x_y_path, int(index)]
                            else:
                                print("任务完成，请开启新任务")
                                # 返回1
                                return [0]

                            xy_file_open.close()

                        latest_file_open.close()

                    break
    # print(max)


if __name__ == '__main__':
    x_y_path = '0_直行作业'
    x_y_path_file_index = x_y_path.split('_')[0]
    print(x_y_path_file_index)

    # rootDir = "/home/wen/PycharmProjects/dongFengProj_1.1/pathRecord/ctrlCAN"
    # print(is_task_legacy(rootDir))
