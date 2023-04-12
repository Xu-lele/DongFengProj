# 对文件行数进行降采样,先实现再优化


if __name__ == "__main__":
    downsampling_rate = 10    # 降采样率
    origin_data_file_path = r"/home/wen/PycharmProjects/dongFengProj_1.1/pathRecord/ctrlCAN/2021-09-02/行驶记录数据2021-09-02 15:21:07.csv"
    result_data_file_prefix = "result_data/"
    result_data_file_name = "dowsampling_行驶记录数据2021-09-02 15:21:07.csv"
    with open(origin_data_file_path, "r") as origin_data_file_open:
        with open(result_data_file_prefix+result_data_file_name, "w") as result_data_file_open:
            data_list = origin_data_file_open.readlines()
            for i in range(len(data_list)):
                print(i)
                if i%downsampling_rate == 0:
                    print("保存")
                    result_data_file_open.write(data_list[i])
            result_data_file_open.close()
        origin_data_file_open.close()

