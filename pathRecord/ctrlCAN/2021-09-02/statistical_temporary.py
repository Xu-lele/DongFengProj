# 进行华测组合导航状态码的统计
import os

# 遍历所有文件，读取状态码，统计所有可能的状态码
def traverse(root_dir):
    state_nums = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file[0] == "行":
                # 是需要的文件，进行读取遍历
                path_name = os.path.join(root, file)
                with open(path_name, "r") as file_open:
                    file_open.readline()
                    data_list = file_open.readlines()
                    for data in data_list:
                        data_list_comma = data.split(",")
                        # 获取状态码
                        # print(type(data_list_comma[26]))
                        state_num = data_list_comma[26]
                        if state_nums.count(state_num) == 0:
                            # 添加进去
                            state_nums.append(state_num)


                    file_open.close()
    return state_nums
            # print(file[0])

if __name__ == "__main__":
    files_root_path = os.getcwd()
    return_list = traverse(files_root_path)
    print(return_list)

    # # 测试列表查找元素
    # a_list = [1,2,3,4,5,6,7,5,6,6,7,8]
    # # list.index(obj)
    # b = a_list.count(6)
    # print(b)