# 拖拉机状态信息保存
import csv
import os
import time

class TractorStatusDataRecord(object):
    def __init__(self, csv_name):
        self.csvfile = open(csv_name, 'w')
        self.writer = csv.writer(self.csvfile, delimiter=',')
        self.writer.writerow(['PC_time_stamp', '帧ID', '帧内容'])  # 写入表头

    def __del__(self):
        self.csvfile.close()

    def data_record(self, msg):
        """
            需要记录的数据字段：
            'PC_time_stamp', '帧ID', '帧内容'
        :return:
        """
        # 要实现写一行就保存一行
        start_time = time.time()
        self.writer.writerow(msg)   # 注意msg得为list类型
        self.csvfile.flush()
        end_time = time.time()
        print("the cost is ", end_time-start_time)

