# 程序一运行就开始记录小车的轨迹，以一定的时间间隔，这样来减少记录的点数aav
import time
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)),"../component")))
print(sys.path)

#上面已经增加了环境变量 所以报错不用管,程序没有错，是拼写检查的错误
import classOfIMU_CGI610
import classOfDataRecord

imu = classOfIMU_CGI610.Imu()
path = r'pathRecord/'
name = '记录下的要跟踪的轨迹点' + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + '.csv'
record = classOfDataRecord.DataRecord(name)

delta_t = 0.5   # 这个是录轨迹点的时间间隔，单位s

cmd_steering = 0
cmd_v = 0
path_target_point_index = 1
while 1:
    # 不断读取imu，然后保存为csv
    path_generate_inte_navi_info = imu.stateOfCar()
    msg = [time.time(), cmd_steering, cmd_v, path_target_point_index] + path_generate_inte_navi_info
    record.data_record(msg)
    time.sleep(delta_t)

del record
