"""
    本类实现CGI610组合导航实时信息的读取,后期可抽象为组合导航类
"""

import serial
import time
import component_main.component.ll2xy as ll2xy
import math

class Imu():
    """调用读取IMU的数据，并且将IMU传来的数据进行转换和格式化"""
    def __init__(self):
        """初始化端口，等待调用读取即可"""
        portx = "/dev/ttyUSB0"
        # bps = 115200  # 10hz时，5hz时对应波特率
        bps = 460800  # 100hz时对应波特率，这样应该就不会有数据延迟的问题 460800
        timex = 5
        self.ser = serial.Serial(portx, bps, timeout=timex)

        # 如果不加下面的一行命令会出现读取错误
        # 解决：读取错误应该是要抛去第一次读取的，跟读取的快慢没关系
        self.ser.readline()


    def stateOfCar(self):
        """"获取Imu传来的数据，并将数据解析分段，保存到各个参数中，最后的返回值是已经格式化好并转好格式的数据"""

        imuInfo = self.ser.readline().decode("gbk")

        # 调用一次，清空一次缓冲区,解决错误读取的问题
        self.ser.reset_input_buffer()
        self.ser.readline()  # 扔掉一条

        #输出读取到的IMU信息
        print('imuInfo=', imuInfo)

        # 对格式字符进行分割得到
        str = imuInfo.split(',')

        # print(str)

        # 'PC_time_stamp', 'utm_x', 'utm_y', 'cmd_steering', 'cmd_v', 'path_target_point_index',
        # 'GPSWeek', 'GPSTime', 'Heading', 'Pitch', 'Roll', 'gyro_x', 'gyro_y', 'gyro_z',
        # 'acc_x', 'acc_y', 'acc_z', 'Latitude', 'Longitude', 'Altitude', 'Ve', 'Vn', 'Vu', 'V',
        # 'NSV1', 'NSV2', 'Status', 'Age', 'Warming'

        # 以下为GPCHC所有的字段
        GPSWeek = int(str[1])
        GPSTime = float(str[2])
        Heading = float(str[3])  # headingAngle = Heading
        Pitch = float(str[4])
        Roll = float(str[5])
        gyro_x = float(str[6])
        gyro_y = float(str[7])
        gyro_z = float(str[8])
        acc_x = float(str[9])
        acc_y = float(str[10])
        acc_z = float(str[11])
        Latitude = float(str[12])
        Longitude = float(str[13])
        Altitude = float(str[14])
        Ve = float(str[15])
        Vn =float(str[16])
        Vu = float(str[17])
        V = float(str[18])
        NSV1 = int(str[19])
        NSV2 = int(str[20])
        Status = int(str[21])
        Age = int(str[22])
        Warming = str[23]
        Warming = Warming.split("*")
        Warming = float(Warming[0])

        # 经纬度转xy
        utm_x, utm_y = ll2xy.ll2xy(lat=Latitude, lon=Longitude)

        def geometric2utm(geo_point: tuple, datum_point: tuple, beta_ridian: float):
            """
                将由由经纬度转成的XY点再转成utm坐标点
            :param geo_point:   根据想要的几何形状在直角坐标系中确定的点，通常在原点附近，根据原点和偏移来表示几何中的点
            :param datum_point: 变换基准点，指的是AB线中的B点，为UTM坐标系中的点
            :param beta_ridian: AB向量相对于正北方向的角度，0~360°
            :return:
            """
            x_skim = geo_point[0]
            y_skim = geo_point[1]
            utm_p_x = x_skim * math.cos(-beta_ridian) - y_skim * math.sin(-beta_ridian) + datum_point[0]
            utm_p_y = y_skim * math.cos(-beta_ridian) + x_skim * math.sin(-beta_ridian) + datum_point[1]
            utm_point = (utm_p_x, utm_p_y)
            return utm_point

        B = (utm_x, utm_y)
        B_ori = (0 + 0.78, 0)  # 把B点移到原点
        B_planning = geometric2utm(geo_point=B_ori, datum_point=B, beta_ridian=math.radians(Heading))
        utm_x = B_planning[0]
        utm_y = B_planning[1]

        return [utm_x, utm_y, GPSWeek, GPSTime, Heading, Pitch, Roll, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, Latitude, Longitude,
                Altitude, Ve, Vn, Vu, V, NSV1, NSV2, Status, Age, Warming]



if __name__ == "__main__":
    # 进行人工驾驶时开启
    imu = Imu()
    while True:
        t1 = time.time()
        print("imu.stateOfCar()[8]", type(imu.stateOfCar()))  # 测试下正常情况
        t2 = time.time()
        print("Showing the states costs " + str(t2 - t1) + " s at this circle")
        time.sleep(0.1)
