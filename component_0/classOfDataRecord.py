# 学习csv文件的读写
# 用一个类来实现，在循迹程序开始前打开文件，在循迹程序结束时关闭文件
import csv


class DataRecord:
    def __init__(self, csv_name):
        self.csvfile = open(csv_name, 'w')
        self.writer = csv.writer(self.csvfile, delimiter=',')
        # self.writer.writerow(['GPSWeek', 'GPSTime', 'imu_lon', 'imu_lat', 'headingAngle', 'x', 'y', 'v',
        #                       'cmd_steering', 'cmd_v', 'inte_navi_status', 'imu_altitude', 'imu_satellite_num', 'imu_warning']) # 写入表头

        self.writer.writerow(['PC_time_stamp', 'cmd_steering', 'cmd_v', 'path_target_point_index', 'utm_x', 'utm_y',
                              'GPSWeek', 'GPSTime', 'Heading', 'Pitch', 'Roll', 'gyro_x', 'gyro_y', 'gyro_z',
                              'acc_x', 'acc_y', 'acc_z', 'Latitude', 'Longitude', 'Altitude', 'Ve', 'Vn', 'Vu', 'V',
                              'NSV1', 'NSV2', 'Status', 'Age', 'Warming', "path_x_y_file", "target_speed", "Kp",
                              "threshold_distance", "tractor_info_dict", "realtime_lateral_error", "backward_flag",
                              "machine_operation_status", "task_index", "is_endtask","driving_mode",
                              "machine_operation_enabled"])  # 写入表头

    def __del__(self):
        self.csvfile.close()

    def data_record(self, msg):
        """
            需要记录的数据字段：pc记录时间戳，x坐标，y坐标，指令转角，指令车速，预瞄点序号（组合导航读取数据）
            'PC_time_stamp', 'utm_x', 'utm_y', 'cmd_steering', 'cmd_v', 'path_target_point_index',
                              'GPSWeek', 'GPSTime', 'Heading', 'Pitch', 'Roll', 'gyro_x', 'gyro_y', 'gyro_z',
                              'acc_x', 'acc_y', 'acc_z', 'Latitude', 'Longitude', 'Altitude', 'Ve', 'Vn', 'Vu', 'V',
                              'NSV1', 'NSV2', 'Status', 'Age', 'Warming',path_x_y_file, target_speed, Kp,
                              threshold_distance，tractor_info_dict, realtime_lateral_error, backward_flag,
                              machine_operation_status,task_index, is_endtask, driving_mode, machine_operation_enabled
        :return:
        """
        # msg=[GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v]
        # 测试是否可以写数值型
        # self.writer.writerow([GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v])
        self.writer.writerow(msg)
        self.csvfile.flush()    # 后期如果这里复杂度太高了可以减少缓存刷新次数


if __name__ == "__main__":
    record = DataRecord()
    # 10个参数存入
    a = []
    for i in range(10):
        print(i)
        a.append(i)

    for j in range(10):
        record.data_record(a)
    del record
