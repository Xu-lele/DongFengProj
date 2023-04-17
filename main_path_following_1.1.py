"""当前模块进行路径跟踪功能开发，上层的任务规划产生具体的路径点序列。"""
import random
import threading
import time
import component_main.classOfIMU_CGI610 as classOfIMU_CGI610
import component_main.classOfDF2204 as classOfDongFengSF2204
# import classOfYunLe
import math
import component_main.steering_ctrl as steering_ctrl
import component_main.classOfDataRecord as classOfDataRecord
# import classOfCollect_info

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from functools import partial

# import component_main.helloworld as helloworld
import component_ui.driving_information_board as driving_information_board
import os
import component_main.classOfTractorInfoRead as classOfTractorInfoRead
import component_main.component.calc_point2line as calc_p2l
import component_ui.ui_module_main_path_following_1_1 as ui_module
import component_main.component.readfiles_in_order as readfiles_in_order
import component_main.severTest1 as perception_process_ser

# debug
imu = classOfIMU_CGI610.Imu()
tractor = classOfDongFengSF2204.Tractor("send")
vcu_cmd = classOfDongFengSF2204.VCUCmd(tractor)
tractor_info_read = classOfTractorInfoRead.TractorInfoRead()


def isArrive(vehicle_x_y, path_x_y, i, threshold_distance):
    # 计算小车与当前目标点的距离
    goalPoint = path_x_y[i]
    distance = math.sqrt((vehicle_x_y[0] - goalPoint[0]) ** 2 + (vehicle_x_y[1] - goalPoint[1]) ** 2)
    print("离路径点" + str(i) + "的距离为：" + str(distance))

    # 增加一个记录功能
    net_goal_point = path_x_y[i + 1]  # 对路径增加一个多余的点即可
    distance2net_point = math.sqrt(
        (vehicle_x_y[0] - net_goal_point[0]) ** 2 + (vehicle_x_y[1] - net_goal_point[1]) ** 2)

    if (distance < threshold_distance) or (
            distance2net_point < distance):  # 说明两点的距离要大于0.8m，直线情况下无所谓，但是转弯情况下就有可能跟踪不上路径。可能会出现转圈的情况
        # 此时认为小车到达了
        return True
    else:
        return False

    # 可以启用另一种判别，即到下一点的距离比到当前点的距离小。
    # 即满足小于误差，或者上述条件


def is_arrive(vehicle_x_y, path_x_y, i):
    """
        纯追踪判断小车是否逻辑到达目标点，方便切换目标点用
    :param vehicle_x_y:
    :param path_x_y:
    :param i:
    :return:
    """
    global v
    # 计算小车与当前目标点的距离
    goal_point = path_x_y[i]
    distance = math.sqrt((vehicle_x_y[0] - goal_point[0]) ** 2 + (vehicle_x_y[1] - goal_point[1]) ** 2)
    print("离路径点" + str(i) + "的距离为：" + str(distance))

    # 计算与下一个跟踪点的距离
    next_goal_point = path_x_y[i + 1]
    distance2net_point = math.sqrt(
        (vehicle_x_y[0] - next_goal_point[0]) ** 2 + (vehicle_x_y[1] - next_goal_point[1]) ** 2)

    kv = v
    # 给kv设置上下限
    if kv < 1:
        kv = 1
    if kv > 3.0555:  # 11km/h
        kv = 3.0555

    if (distance < kv) or (distance2net_point < distance):  # 用纯追踪判断，这里k设置为1
        # 此时认为小车到点了
        return True
    else:
        return False


def PIDCtrl():
    pass


def calcAngOfTwoVector(vector1, vector2):
    """
        这里向量用元组来实现，已经是计算好的以原点为起点的向量
    :param vector1:
    :param vector2:
    :return: 返回两个向量的夹角，范围0~180°
    """
    # endAng=0
    AB = [0, 0, vector1[0], vector1[1]]
    CD = [0, 0, vector2[0], vector2[1]]

    def angle(v1, v2):
        # 计算v1，v2两角的0-180度角度
        dx1 = v1[2] - v1[0]
        dy1 = v1[3] - v1[1]
        dx2 = v2[2] - v2[0]
        dy2 = v2[3] - v2[1]
        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle
        return included_angle

    ang1 = angle(AB, CD)
    # 浮点数应该可以比较大小，但是比较相等要用精度判断
    # if waypoint_x - x > 0:
    #     # 在右边，0~180
    #     endAng = ang1
    # else:
    #     # 在左边，180~360
    #     endAng = 360 - ang1
    print("AB和CD的夹角")
    print(ang1)
    return ang1


def calcAngErrorOfCarAndPoint(vector1, vector2):
    """
        2021/1/12 未完成，完成功能后删除
        实现目标航向和小车航向的偏差计算
    :param vector1:目标航向
    :param vector2:实际航向
    :return: 返回两个向量的夹角，范围从目标航向逆时针0~180°，从目标航向顺时针0~-180°
    """
    # endAng=0
    AB = [0, 0, vector1[0], vector1[1]]
    CD = [0, 0, vector2[0], vector2[1]]

    def angle(v1, v2):
        # 计算v1，v2两角的0-180度角度
        dx1 = v1[2] - v1[0]
        dy1 = v1[3] - v1[1]
        dx2 = v2[2] - v2[0]
        dy2 = v2[3] - v2[1]
        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle
        return included_angle

    ang1 = angle(AB, CD)
    # 浮点数应该可以比较大小，但是比较相等要用精度判断
    # if waypoint_x - x > 0:
    #     # 在右边，0~180
    #     endAng = ang1
    # else:
    #     # 在左边，180~360
    #     endAng = 360 - ang1
    print("AB和CD的夹角")
    print(ang1)
    return ang1


def calcAngOfY_Axis(vector):
    """
        计算向量与Y轴的顺时针夹角，范围0~360°
    :param vector:
    :return: 返回两个向量的夹角
    """
    # endAng=0
    vector1 = (0, 1)  # Y axis
    vector2 = vector
    AB = [0, 0, vector1[0], vector1[1]]
    CD = [0, 0, vector2[0], vector2[1]]

    def angle(v1, v2):
        # 计算v1，v2两角的0~180度角度
        dx1 = v1[2] - v1[0]
        dy1 = v1[3] - v1[1]
        dx2 = v2[2] - v2[0]
        dy2 = v2[3] - v2[1]
        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle
        return included_angle

    ang1 = angle(AB, CD)
    # 浮点数应该可以比较大小，但是比较相等要用精度判断
    # if waypoint_x - x > 0:
    #     # 在右边，0~180
    #     endAng = ang1
    # else:
    #     # 在左边，180~360
    #     endAng = 360 - ang1
    if vector2[0] < 0:
        ang1 = 360 - ang1
    print("小车指向目标点向量与Y轴夹角=", ang1)
    return ang1


def calcAngOf2Vectors(vector_car, vector_line):
    """
        计算起始点指向车辆向量与轨迹线向量的差，返回值提供给横向误差判断正负用。
    :param vector_car:
    :param vector_line:
    :return:
    """
    return calcAngOfY_Axis(vector_car) - calcAngOfY_Axis(vector_line)


def PControl(target, current, Kp):
    """
        P控制器
    :param target:
    :param current:
    :return:
    """
    a = Kp * (target - current)

    return a


class WorkOperation(object):
    # 本类暂时只针对播种作业
    def __init__(self, is_operation_flag, realtime_lateral_error):
        """进行初始化作业,在这里放一个主循环，监测is_operation_flag的变化，根据其变化来"""
        self.iswork_flag = is_operation_flag  # 标志位要传递到path_following里面去。(列表类型)
        self.realtime_lateral_error = realtime_lateral_error

        # task_root_dir = "/home/wen/PycharmProjects/dongFengProj_1.1/pathRecord/ctrlCAN"

        # 加载任务文件，监测标志位，准备开始作业
        # task_path = "/home/wen/PycharmProjects/dongFengProj_1.1/pathGenerate/ctrl_mission_planning/"
        # 遍历所有文件，存为列表

        # with open(task_path, "r") as task_files_open:
        #
        # pass

    def is_task_legacy_and_execute(self):
        # 该函数还可监测是否有遗留任务，若想直接开启新任务，则调用task_execute即可
        # 该函数被调用，然后是主线程阻塞在该函数里
        # 加载之前的任务，监测是否任务完成，若完成进行新任务，若未完成进行续作
        # 读取最后一个文件，判断任务是否结束
        legacy_return = readfiles_in_order.is_task_legacy("/home/wen/PycharmProjects/dongFengProj_1.1/pathRecord/ctrlCAN")
        x_y_path = "0_"
        x_y_index = 0
        if len(legacy_return) == 2:
            # 有遗留任务
            x_y_path = legacy_return[0]  # 格式类似于：0_直行作业.txt
            x_y_index = legacy_return[1]
        # 输入path的序号、index就应该接着往下执行，调用同一个执行任务的函数才对
        x_y_path_file_index = int(x_y_path.split('_')[0])
        self.task_execute(x_y_path_file_index, x_y_index, driving_mode=2, machine_operation_enabled=1)  # 没有遗留任务，则传参为0，0

    def task_execute(self, x_y_path_file_index, xy_index, driving_mode, machine_operation_enabled):
        # 增加flag判断
        while self.iswork_flag[0] == 0:
            time.sleep(0.2)

        # 获取中断路径点
        x_y_path_file_index = is_operation_flag[4]
        xy_index = is_operation_flag[5]

        # 开启任务，读取任务文件，判断is_work,然后执行。若是新任务，则两个参数均为0
        ctrl_miss_path = "/home/wen/PycharmProjects/dongFengProj_1.1/pathGenerate/ctrl_mission_planning"
        task_dict = {}
        for root, dirs, files in os.walk(ctrl_miss_path):
            for file in files:
                print(file)
                file_underline = file.split("_")
                file_index = int(file_underline[0])
                task_dict.update({file_index: file})

        print(task_dict)
        dict_len = len(task_dict)
        for i in range(dict_len):
            # print(task_dict[i])
            # 在这里执行任务
            if i < x_y_path_file_index:
                # 已完成的任务，跳过
                continue
            # 开始执行任务,进行path_following参数的初始化
            # is_operation_flag = [0]  # 使用列表，方便参数传递（保证只有一个对象修改完成即可）。为1表示开启，为0表示不开启
            # path_file = "pathGenerate/ab_record/ab_line_2021-09-02_1/ab_list.txt"  # 在此处更换路径文件
            path_file = ctrl_miss_path + "/" + task_dict[i]
            path_x_y = path_load(path_file, x_pos=0, y_pos=1)
            # path_x_y.reverse()
            target_speed = 3  # 目标速度，cmd_v是计算出来的指令速度，为控制量
            # path_target_point_index = 0  # 后期记录断点，方便续作
            Kp = 1  # 拖拉机转角p控制器参数，测试时适当调整
            threshold_distance = 3  # 拖拉机判断是否到达预瞄点的距离阈值，拖拉机前后轮距3.1m，测试时候适当调整
            # 判断文件名，然后确定是否后退
            file_underline = task_dict[i].split("_")
            backward_flag = 0  # 默认后退标志为0，表示前进模式
            machine_operation_status = "raise"  # 机具状态（当前针对播种作业）
            if file_underline[1] == "掉头直线后退.txt":
                backward_flag = 1
                target_speed = 3  # 掉头后退速度
            if file_underline[1] == "直行作业.txt":
                machine_operation_status = "down"
                target_speed = 7  # 直线作业速度
            if file_underline[1] == "掉头前进离开直线.txt" or file_underline[1] == "掉头前进靠近直线.txt":
                # backward_flag = 1
                target_speed = 3  # 掉头曲线速度

            # realtime_lateral_error = [9999]  # 实时横向误差，提供给界面显示误差用

            task_index = i
            is_endtask = 0
            if i == dict_len - 1 or driving_mode == 0:  # 手动模式时也设置为1
                is_endtask = 1

            path_following(target_speed, path_x_y, xy_index, Kp, threshold_distance=threshold_distance,
                           path_x_y_file=path_file, realtime_lateral_error=self.realtime_lateral_error,
                           backward_flag=backward_flag,
                           machine_operation_status=machine_operation_status, task_index=task_index,
                           is_endtask=is_endtask,
                           iswork_flag=self.iswork_flag, driving_mode=driving_mode,
                           machine_operation_enabled=machine_operation_enabled)

        pass

    # def is_task_legacy(self):
    #     # 检查是否有遗留任务，若有，返回文件名和index。若无返回0
    #     readfiles_in_order.is_task_legacy()
    #     pass

    # 作业路径直线（曲线）跟踪，机具全部放下
    # def work_line_tracking_operation(self):
    #     pass
    #     # 直线跟踪函数与机具控制函数在这里集成
    #
    # # 掉头前进跟踪路径，机具全部抬起
    # def turn_line_tracking_operation(self, direction_flag):
    #     pass
    #
    # # 有中断就保存中断点的信息


def convert(ui):
    ui.pushButton.setEnabled(False)
    print("print one.")
    text_input = ui.lineEdit_input.text()
    result = float(text_input) * 6.7
    ui.lineEdit_output.setText(str(result))


def click_success(ui):
    print("test")
    ui.pushButton.setEnabled(True)


def ui_window(ui_lateral_error, is_operation_flag):
    """UI展示，用线程实现。显示必要的状态参数，设置程序参数等，进行人机交互"""
    app = QApplication(sys.argv)  # zz开启一个windows主线程？
    MainWindow = QMainWindow()
    ui = driving_information_board.Ui_MainWindow()  # 加载自定义的UI
    ui.setupUi(MainWindow)  # 设置为主窗口？

    # 误差显示函数
    def lateral_error_dispaly(ui, lateral_error_list):
        refresh_rate = 10  # Hz,刷新率后期进行调整
        while True:
            ui.label_lateral_error.setText("左正右负" + str(lateral_error_list[0]))
            time.sleep(1 / refresh_rate)

    # 开启误差显示线程
    display_t = threading.Thread(target=lateral_error_dispaly, args=(ui, ui_lateral_error))
    display_t.start()
    ui.pushButton_start.clicked.connect(partial(ui_module.pushButton_start_clicked, ui, is_operation_flag))
    ui.pushButton_interrupt.clicked.connect(partial(ui_module.pushButton_interrupt_clicked, ui, is_operation_flag))
    ui.pushButton_load.clicked.connect(partial(ui_module.pushButton_load_clicked, ui, is_operation_flag))
    ui.pushButton_ori_start.clicked.connect(partial(ui_module.pushButton_ori_start_clicked, ui, is_operation_flag))

    MainWindow.show()
    # ui.pushButton_serial.clicked.connect(click_success)
    # ui.pushButton_pcan.clicked.connect(partial(convert, ui))
    # print(app)
    # ui.webView_map.load(QtCore.QUrl("http://baidu.com"))
    sys.exit(app.exec_())


def check_mkdir():
    # 检查文件夹是否存在，若不存在，则创建
    is_exists = os.path.exists("pathRecord/ctrlCAN/" + time.strftime("%Y-%m-%d", time.localtime()))
    if not is_exists:
        os.makedirs("pathRecord/ctrlCAN/" + time.strftime("%Y-%m-%d", time.localtime()))
    else:
        print("path 存在")


def path_following(target_speed, path_x_y, path_target_point_index, Kp, threshold_distance, path_x_y_file,
                   realtime_lateral_error, backward_flag, machine_operation_status, task_index, is_endtask,
                   iswork_flag, driving_mode, machine_operation_enabled):  # 20211001增加驾驶模式，机具控制使能
    """
        路径跟踪函数。后期添加机具控制代码
    :param target_speed:
    :param path_x_y:
    :param path_target_point_index:
    :param Kp:
    :param threshold_distance:
    :param path_x_y_file:
    :param realtime_lateral_error:
    :param backward_flag:默认为0，前进模式
    :param machine_operation_status:
    :param task_index:
    :param is_endtask:
    :param iswork_flag:判断是否为1，为0则停车
    :param driving_mode:驾驶模式，分为0：手动模式，1：扭矩请求模式，2：自动模式
    :param machine_operation_enabled:机具控制使能标志，0：未使能，1：使能
    :return:
    """
    # imu = classOfIMU_CGI610.Imu()
    # tractor = classOfDongFengSF2204.Tractor("send")
    # vcu_cmd = classOfDongFengSF2204.VCUCmd(tractor)
    global imu
    global vcu_cmd

    # 机具控制函数
    def machine_ctrl(machine_operation_enabled):
        # 添加机具控制，及其使能
        loc_machine_operation_enabled = machine_operation_enabled
        if loc_machine_operation_enabled == 1:
            if machine_operation_status == "raise":
                # 机具提升命令发送
                # pto断开
                vcu_cmd.send_pto_msg(0)
                time.sleep(0.1)  # debug
                vcu_cmd.send_hoist_msg("rising", hoist_ploughing_depth_set=1000, hoist_height_limit_set=220)
                pass
            elif machine_operation_status == "down":
                # 机具下降命令发送
                vcu_cmd.send_hoist_msg("falling", hoist_ploughing_depth_set=0, hoist_height_limit_set=220)
                time.sleep(0.1)  # debug
                vcu_cmd.send_pto_msg(1)
                pass

    # tractor_info_read = classOfTractorInfoRead.TractorInfoRead()
    global tractor_info_read
    # 判断按日记录文件夹是否存在，不存在则创建
    check_mkdir()
    path = r"pathRecord/ctrlCAN/" + time.strftime("%Y-%m-%d", time.localtime()) + '/'
    record = classOfDataRecord.DataRecord(
        path + "行驶记录数据" + time.strftime('%Y-%m-%d %H-%M-%S', time.localtime(time.time())) + ".csv")
    cmd_steering = 0  # 初始化指令转角为0
    cmd_v = 0  # 初始化指令速度为0
    Kd = 0.5
    Ki = 0
    sum_error = 0
    loc_threshold_distance = threshold_distance
    cmd_gear = "空挡"
    if backward_flag == 1:
        cmd_gear = "后退低档"
    else:
        cmd_gear = "前进低档"
    Lf = 1.0  # 前视距离
    front_error = 0

    # 20210927增加尾部两点，保证车子能到路径末尾点（20211014这个逻辑上有点问题，只用在最后一个直线任务上进行就好了，判断是否最后一个任务）
    tail_threshold_distance = 2  # m
    if is_endtask == 1:
        tail_two_points = [(0, tail_threshold_distance), (0, 2 * tail_threshold_distance)]
        tail_A = path_x_y[-2]
        tail_B = path_x_y[-1]
        AB_vector = (tail_B[0] - tail_A[0], tail_B[1] - tail_A[1])
        beta = calcAngOfY_Axis(AB_vector)
        for orig_p in tail_two_points:
            # 先旋转，再平移
            x_skim = orig_p[0]
            y_skim = orig_p[1]

            # utm_p_x = b_1 * math.cos(beta) + b_2 * math.sin(beta) + c * math.cos(beta) + d * math.sin(beta)
            # utm_p_y = b_2 * math.cos(beta) - b_1 * math.sin(beta) + d * math.cos(beta) - c * math.sin(beta)

            utm_p_x = x_skim * math.cos(beta) + y_skim * math.sin(beta) + tail_B[0]
            utm_p_y = y_skim * math.cos(beta) - x_skim * math.sin(beta) + tail_B[1]
            path_x_y.append((utm_p_x, utm_p_y))

    #  初始化第一个跟踪的路径点
    length = len(path_x_y)

    record_time_start = time.time()  # 数据记录时间戳开始
    steering_wheel_failure_timestamp = time.time()  # 方向盘故障时间戳
    while path_target_point_index <= length - 2:  # 当跟踪的最后一个点时候，结束跟踪，小车制动。因为每步只跟踪一个点，所以i的范围可以从0~length-1,设为len-2保险
        starttime = time.time()
        # 增加运行标志判断，如果不在工作，则发停车指令（速度为0，方向盘固定角度。暂时先这种，这样的话停止就不会记录数据了。）
        if iswork_flag[0] == 0:
            while True:
                if iswork_flag[0] == 1:
                    break
                else:
                    # 停车指令
                    vcu_cmd.send_motion_ctrl_msg(0, "空挡", 0, 0, 1)
                    # 断开pto
                    vcu_cmd.send_pto_msg(0)
                    print("iswork_flag为1，正在停车")
                    time.sleep(0.2)

        # time.sleep(0.3)

        """全部处理完再合成指令发送"""
        waypoint = path_x_y[path_target_point_index]

        # GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, inte_navi_status, imu_altitude, imu_satellite_num, imu_warning = imu.stateOfCar()  # 执行一次采样一次。这里用imu.stateOfCar实现
        inte_navi_info = imu.stateOfCar()  # 执行一次采样一次。这里用imu.stateOfCar实现
        tractor_info_dict = tractor_info_read.vehicle_state_info_return()
        x = inte_navi_info[0]
        y = inte_navi_info[1]

        # 计算实时误差然后传递出去，让显示线程不断访问误差并显示(以下为直线计算)
        # 暂时统一直线曲线的计算方式，使用异常处理，异常时误差显示为999
        try:
            linear_tracking_error = calc_p2l.get_distance_from_point_to_line([x, y],
                                                                             [path_x_y[path_target_point_index - 1][0],
                                                                              path_x_y[path_target_point_index - 1][1]],
                                                                             [path_x_y[path_target_point_index][0],
                                                                              path_x_y[path_target_point_index][1]])
            start_point2vehicle = (
                x - path_x_y[path_target_point_index - 1][0], y - path_x_y[path_target_point_index - 1][1])
            start2end = (path_x_y[path_target_point_index][0] - path_x_y[path_target_point_index - 1][0],
                         path_x_y[path_target_point_index][1] - path_x_y[path_target_point_index - 1][1])
            angle_error = calcAngOf2Vectors(start_point2vehicle, start2end)
            if 180 > angle_error >= 0 or -360 <= angle_error < -180:
                # 右边，为负
                linear_tracking_error = -linear_tracking_error
        except:
            linear_tracking_error = 9998

        # linear_tracking_error = calc_p2l.get_distance_from_point_to_line([x, y], [path_x_y[0][0], path_x_y[0][1]], [path_x_y[-1][0], path_x_y[-1][1]])
        # 计算左右

        realtime_lateral_error[0] = linear_tracking_error
        headingAngle = inte_navi_info[4]

        # 20210820新增组合导航状态判断，并记录日志
        # 判断inte_navi status,不是42则停车等待。
        while inte_navi_info[22] != 42 or tractor_info_dict["方向盘故障"] == 10:  # 20210929debug !=0 改成 ==10
            # 发送停车指令
            vcu_cmd.send_motion_ctrl_msg(driving_mode, "空挡", 0, 0, 1)

            if inte_navi_info[22] != 42:
                print("惯导状态不正常，停车中。状态码：", inte_navi_info[22])
                # time.sleep(10)
            if tractor_info_dict["方向盘故障"] == 10:  # 20210929debug,把故障码改了1->10
                steering_wheel_failure_now = time.time()
                print("方向盘故障，清理中。状态码：", tractor_info_dict["方向盘故障"])
                # 计划清一下故障，但是两次故障时间<1秒视为真正的故障,不再清理等待修复，打印真故障(需要记录下故障时候的数据才能设计清理策略)
                # 将本次出错的时间戳与上次进行相比，若还在安全时间内则基础清除，否则阻塞执行
                if steering_wheel_failure_now - steering_wheel_failure_timestamp > 1:
                    # 刹车
                    vcu_cmd.send_motion_ctrl_msg(driving_mode, "空挡", 0, 0, 1)
                    while True:
                        tractor_info_dict = tractor_info_read.vehicle_state_info_return()
                        # print(tractor_info_dict)
                        print("方向盘真正故障了，等待修复...状态码", tractor_info_dict["方向盘故障"])
                        time.sleep(0.2)
                vcu_cmd.steering_wheel_fault_removal()
                # 保存本次出错的时间戳
                steering_wheel_failure_timestamp = steering_wheel_failure_now
                # 另一种判断是判断故障累计时间>1s,测试过后再说

            time.sleep(0.2)
            inte_navi_info = imu.stateOfCar()
            tractor_info_dict = tractor_info_read.vehicle_state_info_return()

        # print(headingAngle, x, y, v)
        vehicle_x_y = (x, y)

        # 20211014更新loc_threshold_distance，不行的话再做修改
        if tractor_info_dict["车速"] < 4.0:
            loc_threshold_distance = 1
        else:
            loc_threshold_distance = 1 / 3 * (tractor_info_dict["车速"] - 1)

        # 计算小车是否到达跟踪点
        if not isArrive(vehicle_x_y, path_x_y, path_target_point_index, threshold_distance=loc_threshold_distance):
            # TODO 进行横向跟踪控制，将控制指令的产生用函数来实现
            # 判断小车航向角是否满足要求:

            # 先判断向量的象限，再计算目标夹角

            # 计算小车指向目标点的向量
            car2goalPoint = (waypoint[0] - x, waypoint[1] - y)

            goalHeadingAng = calcAngOfY_Axis(car2goalPoint)
            # etc = goalAng - headingAngle

            # 转向控制pid实现，需要调参
            # target_headingAngle=0 #小车指向目标点航向角
            # 20211013debug 引入微分控制
            # cmd_steering = steering_ctrl.new_pcontrol(goalHeadingAng, headingAngle, Kp, Ki=0, Kd=0.5)

            cmd_steering, front_error, sum_error = steering_ctrl.PControl(goalHeadingAng, headingAngle,
                                                                          Kp, front_error=front_error, Kd=Kd, Ki=Ki,
                                                                          sum_error=sum_error)  # 这里应该是需要转换角度到相同的起始部位。headingAngle是正北顺时针，goalAng方向是目标方向，是小车指向目标点的方向。
            # 对倒车进行判断
            if backward_flag == 1:
                # 倒车模式
                if cmd_steering > 0:
                    cmd_steering = cmd_steering - 180
                    cmd_steering = -cmd_steering
                else:
                    cmd_steering = cmd_steering + 180
                    cmd_steering = -cmd_steering

            # 对cmd_steering进行整型化。看看这样效果会不会好一些20210907
            # 20211013 尝试放大小区间，大区间比例不变。
            # rand
            rand_num = random.randint(-1, 1)
            cmd_steering = cmd_steering + rand_num

            cmd_steering = round(cmd_steering)  # debug

            # cmd_steering = steering_ctrl.pure_pursuit(goalHeadingAng, headingAngle, Lf)   # 纯追踪。这个按照公式另外设置下
            print('goalHeadingAng-headingAngle=', goalHeadingAng - headingAngle)
            # cmd_steering = headingAngle+cmd_steering
            # cmd_steering = cmd_steering

            # (小车情况)限定在小车接收范围内，+—120°以内，精度0.1。并且误差在60°以上时就方向盘打满.在60°以下进行缩放
            # （拖拉机）计算航向角误差在40°外则打满，不在则取系数为1，暂时先这样，后期再调整。
            if cmd_steering > 40:  # 直接打满40°
                cmd_steering = 40
            elif cmd_steering < -40:
                cmd_steering = -40
            else:
                cmd_steering = cmd_steering * 1
            # 以上，符合条件的转角命令生成完毕

        else:
            # 更新跟踪点为下一个点
            path_target_point_index = path_target_point_index + 1

        # cmd_v filter
        cmd_v = target_speed

        # 2021-10-20 进行速度规划,根据cmd_steering增速，当前默认掉头3km/h，不可改变

        jie_ju_x = 15  # 降到3
        jie_ju_y = 1
        zhixian_k = -1 / jie_ju_x
        zhixian_b = jie_ju_y
        y_point = zhixian_k * cmd_steering + zhixian_b
        cmd_v_delta = target_speed - 3
        cmd_v = 3 + cmd_v_delta * y_point

        cmd_v = target_speed

        if cmd_v <= 0.0:
            cmd_v = 0.0

        # 增加跟踪快要结束时候的减速阶段
        # 20211014 给5s时间给人反应，根据车速计算尾部安全距离
        tail_safe_distance = 10  # m

        # if tractor_info_dict["车速"] < 4.0:
        #     tail_safe_distance = 5  # 尾部安全距离，/m，暂定5m
        # else:
        #     tail_safe_distance = tractor_info_dict["车速"]/3.6*5

        def tail_distance_calc(path_x_y, index0, index1):
            x0 = path_x_y[index0][0]
            y0 = path_x_y[index0][1]
            x1 = path_x_y[index1][0]
            y1 = path_x_y[index1][1]
            return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))

        if tail_distance_calc(path_x_y, path_target_point_index, length - 1) < tail_safe_distance and target_speed != 0:
            cmd_v = 3.0  # debug 转弯默认3km/h，不然会冲突

        if tail_distance_calc(path_x_y, path_target_point_index, 0) < 10 and target_speed != 0:
            cmd_v = 3.0  # debug 转弯默认3km/h，不然会冲突
        if path_target_point_index <= 4:  # 速度一直为4直到跟踪到序号5点
            cmd_v = 3.0
        # 如果是直行作业，则判断4以后控制机具。否则立即控制
        if machine_operation_status == "down":
            if path_target_point_index >= 4 and math.sqrt(
                    (vehicle_x_y[0] - path_x_y[0][0]) ** 2 + (vehicle_x_y[1] - path_x_y[0][1]) ** 2) > 5:
                # 放机具，进行控制
                machine_ctrl(machine_operation_enabled)
                # 更改Kp
                Kp = 1
        elif machine_operation_status == "raise":
            machine_ctrl(machine_operation_enabled)

        # 进行iswork_flag的检查
        # 20210929debug临时 驾驶模式
        temp_driving_mode = driving_mode
        # if iswork_flag[0] == 0 or iswork_flag[1] == 1:  # 作业未使能或者有障碍物，车速降为0，并切换人工驾驶模式
        #     cmd_v = 0.0
        #     temp_driving_mode = 0  # 为0，手动模式
        #     print("iswork_flag为0，降速到0，但是档位没变")

        # cmd_v = 0   # 20210918debug

        # debug测试阶段（暂时先这样） 发送指令前检查下是否大角度，是大角度检查机具是否提升。掉头时会抬起机具，所以大角度也没问题
        max_turn_rad = 15
        if cmd_steering > max_turn_rad:
            # 大角度，检查机具提升状态
            tractor_info_dict = tractor_info_read.vehicle_state_info_return()
            if tractor_info_dict["提升器高度"] < 150:
                while True:
                    print("大角度转弯，提升器位置不够,检查提升器设置和角度阈值设置")
                    # 刹车
                    vcu_cmd.send_motion_ctrl_msg(0, "空挡", 0, 0, 1)
                    time.sleep(1)

        # cmd_v = 7.0     # debug
        vcu_cmd.send_motion_ctrl_msg(temp_driving_mode, cmd_gear, cmd_steering, cmd_v, 0)  # 每个控制周期关键性的命令

        # 记录状态。需要记录的数据字段：GPSweek，GPStime，经度，纬度，航向角，x,y,小车发送的指令（转角，速度）.一共九个。
        # 需求：再增加一个字段，存跟踪的path_target_point_index
        # 'PC_time_stamp', 'cmd_steering', 'cmd_v', 'path_target_point_index', 'utm_x', 'utm_y',
        # 'GPSWeek', 'GPSTime', 'Heading', 'Pitch', 'Roll', 'gyro_x', 'gyro_y', 'gyro_z',
        # 'acc_x', 'acc_y', 'acc_z', 'Latitude', 'Longitude', 'Altitude', 'Ve', 'Vn', 'Vu', 'V',
        # 'NSV1', 'NSV2', 'Status', 'Age', 'Warming'
        # msg = [time.time(), GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v]
        # 生成msg
        msg = [time.time(), cmd_steering, cmd_v, path_target_point_index] + inte_navi_info + \
              [path_x_y_file, target_speed, Kp, loc_threshold_distance] + \
              [tractor_info_dict, linear_tracking_error, backward_flag, machine_operation_status, task_index,
               is_endtask, driving_mode,
               machine_operation_enabled, Kd, Ki]
        record.data_record(msg)

        # 20211020增加作业信息传递,中断记录下来
        iswork_flag[2] = task_index
        iswork_flag[3] = path_target_point_index

        # 增加按固定时间间隔保存数据，大于固定时间，就保存一次，再重开
        record_delta_time = 60 * 60  # （用了flush就一定会被保存，当前的机制就没必要了）每隔1hour保存一个文件。当前实验阶段，适合每行驶一段完整的任务才保存一次，这样方便对比，避免后期数据拼接的麻烦，使用了flush数据就一定会存下来。
        record_time_end = time.time()
        if record_time_end - record_time_start > record_delta_time:
            del record
            record = classOfDataRecord.DataRecord(
                path + "行驶记录数据" + time.strftime('%Y-%m-%d %H-%M-%S', time.localtime(time.time())) + ".csv")
            record_time_start = record_time_end

        endtime = time.time()
        print("mpns_循环开销" + str(endtime - starttime) + "s")

    # 若任务标志为最后一个任务，最后需要停下来，制动（或者切换任务）
    del record

    # 增加停车切换
    # vcu_cmd.send_motion_ctrl_msg(driving_mode, "前进低档", 0, 0, 1)
    # time.sleep(5)

    if is_endtask == 1:
        vcu_cmd.send_motion_ctrl_msg(driving_mode, "空挡", 0, 0, 1)
        time.sleep(0.2)
        vcu_cmd.send_motion_ctrl_msg(0, "空挡", 0, 0, 1)

        while 1:
            print('死循环等待')
            time.sleep(1)


def path_load(path_file, x_pos, y_pos):
    path_x_y = []  # 初始化路径段
    path_doc = path_file

    def path_read(path_doc, x_pos, y_pos, path_x_y):
        with open(path_doc, 'r') as fileopen:
            fileopen.readline()  # 去表头
            content = fileopen.readlines()
            for msg_line in content:
                # 得到经纬度，然后转换成path_x_y
                msg_line_list = msg_line.split(',')
                # ciee_test.csv
                y = float(msg_line_list[y_pos])
                x = float(msg_line_list[x_pos])  # weiDu
                path_x_y.append((x, y))
            fileopen.close()
        print('点加载完成')

    path_read(path_doc, x_pos, y_pos, path_x_y)  # 记录后再循迹

    # 临时加载20210629，直线路径点
    # path_x_y = [(44502.86203611083, 4428582.758912923), (444502.8896622462, 4428582.261046626), (444502.9172883816, 4428581.763180329), (444502.944914517, 4428581.265314032), (444502.9725406524, 4428580.767447736), (444503.0001667877, 4428580.26958144), (444503.0277929231, 4428579.771715143), (444503.0554190585, 4428579.273848847), (444503.08304519387, 4428578.775982549), (444503.11067132925, 4428578.278116253), (444503.13829746464, 4428577.780249956), (444503.1659236, 4428577.282383659), (444503.1935497354, 4428576.784517363), (444503.2211758708, 4428576.286651066), (444503.2488020061, 4428575.78878477), (444503.2764281415, 4428575.290918473), (444503.3040542769, 4428574.793052176), (444503.3316804123, 4428574.29518588), (444503.3593065477, 4428573.797319583), (444503.38693268306, 4428573.299453286), (444503.41455881845, 4428572.801586989), (444503.44218495383, 4428572.303720692), (444503.4698110892, 4428571.805854396), (444503.49743722455, 4428571.3079881), (444503.52506335994, 4428570.810121804), (444503.5526894953, 4428570.312255506), (444503.5803156307, 4428569.81438921), (444503.6079417661, 4428569.316522913), (444503.6355679015, 4428568.818656616), (444503.66319403687, 4428568.32079032), (444503.69082017225, 4428567.822924023), (444503.71844630764, 4428567.325057725), (444503.74607244297, 4428566.82719143), (444503.77369857836, 4428566.329325133), (444503.80132471374, 4428565.831458837), (444503.8289508491, 4428565.33359254), (444503.8565769845, 4428564.835726243), (444503.8842031199, 4428564.337859946), (444503.9118292553, 4428563.839993649), (444503.9394553907, 4428563.342127353), (444503.967081526, 4428562.844261057), (444503.9947076614, 4428562.3463947605), (444504.0223337968, 4428561.848528463), (444504.04995993216, 4428561.350662166), (444504.07758606755, 4428560.85279587), (444504.10521220294, 4428560.354929573), (444504.1328383383, 4428559.857063277), (444504.1604644737, 4428559.35919698), (444504.1880906091, 4428558.861330682), (444504.2157167444, 4428558.363464387), (444504.2433428798, 4428557.86559809), (444504.2709690152, 4428557.367731794), (444504.2985951506, 4428556.869865497), (444504.32622128597, 4428556.3719991995), (444504.35384742136, 4428555.874132903), (444504.38147355674, 4428555.376266606), (444504.40909969213, 4428554.87840031), (444504.4367258275, 4428554.380534013), (444504.46435196284, 4428553.882667717), (444504.49197809823, 4428553.38480142), (444504.5196042336, 4428552.886935123), (444504.547230369, 4428552.389068827), (444504.5748565044, 4428551.89120253), (444504.6024826398, 4428551.393336233), (444504.63010877516, 4428550.895469937), (444504.65773491055, 4428550.397603639), (444504.68536104594, 4428549.899737343), (444504.71298718126, 4428549.401871047), (444504.74061331665, 4428548.90400475), (444504.76823945204, 4428548.406138454), (444504.7958655874, 4428547.9082721565), (444504.8234917228, 4428547.41040586), (444504.8511178582, 4428546.912539563), (444504.8787439936, 4428546.414673266), (444504.90637012897, 4428545.91680697), (444504.93399626436, 4428545.418940673), (444504.9616223997, 4428544.921074377), (444504.9892485351, 4428544.42320808), (444505.01687467046, 4428543.925341784), (444505.04450080585, 4428543.427475487), (444505.07212694123, 4428542.92960919), (444505.0997530766, 4428542.4317428935), (444505.127379212, 4428541.933876596), (444505.1550053474, 4428541.436010299), (444505.1826314828, 4428540.938144003), (444505.2102576181, 4428540.440277707), (444505.2378837535, 4428539.942411411), (444505.2655098889, 4428539.4445451135), (444505.29313602427, 4428538.946678817), (444505.32076215965, 4428538.44881252), (444505.34838829504, 4428537.950946223), (444505.3760144304, 4428537.453079927), (444505.4036405658, 4428536.95521363), (444505.4312667012, 4428536.4573473325), (444505.4588928365, 4428535.959481037), (444505.4865189719, 4428535.46161474), (444505.5141451073, 4428534.963748444), (444505.5417712427, 4428534.465882147), (444505.5693973781, 4428533.9680158505), (444505.59702351346, 4428533.470149553), (444505.62464964885, 4428532.972283256), (444505.65227578423, 4428532.47441696), (444505.6799019196, 4428531.976550663), (444505.70752805495, 4428531.478684368), (444505.73515419033, 4428530.9808180705), (444505.7627803257, 4428530.482951773), (444505.7904064611, 4428529.985085477), (444505.8180325965, 4428529.48721918), (444505.8456587319, 4428528.989352884), (444505.87328486727, 4428528.491486587), (444505.90091100265, 4428527.9936202895), (444505.928537138, 4428527.495753994), (444505.95616327337, 4428526.997887697), (444505.98378940875, 4428526.500021401), (444506.01141554414, 4428526.002155104), (444506.0390416795, 4428525.504288807), (444506.0666678149, 4428525.00642251), (444506.0942939503, 4428524.508556213), (444506.1219200857, 4428524.010689917), (444506.1495462211, 4428523.51282362), (444506.1771723564, 4428523.014957325), (444506.2047984918, 4428522.517091027), (444506.2324246272, 4428522.01922473), (444506.26005076256, 4428521.521358434), (444506.28767689795, 4428521.023492137), (444506.31530303333, 4428520.52562584), (444506.3429291687, 4428520.027759544), (444506.3705553041, 4428519.5298932465), (444506.3981814395, 4428519.03202695), (444506.4258075748, 4428518.534160654), (444506.4534337102, 4428518.036294358), (444506.4810598456, 4428517.538428061), (444506.508685981, 4428517.040561764), (444506.53631211637, 4428516.542695467), (444506.56393825175, 4428516.04482917), (444506.59156438714, 4428515.546962873), (444506.6191905225, 4428515.049096577), (444506.6468166579, 4428514.55123028), (444506.67444279324, 4428514.053363984), (444506.70206892863, 4428513.555497687), (444506.729695064, 4428513.057631391), (444506.7573211994, 4428512.559765094), (444506.7849473348, 4428512.061898797), (444506.8125734702, 4428511.564032501), (444506.84019960556, 4428511.0661662035), (444506.86782574095, 4428510.568299907), (444506.89545187633, 4428510.07043361), (444506.92307801166, 4428509.572567314), (444506.95070414705, 4428509.074701018), (444506.97833028244, 4428508.576834721), (444507.0059564178, 4428508.078968424), (444507.0335825532, 4428507.581102127), (444507.0612086886, 4428507.08323583), (444507.088834824, 4428506.585369534), (444507.11646095937, 4428506.087503237), (444507.14408709476, 4428505.5896369405), (444507.1717132301, 4428505.091770644), (444507.19933936547, 4428504.593904347), (444507.22696550086, 4428504.096038051), (444507.25459163624, 4428503.598171754), (444507.28221777163, 4428503.100305458), (444507.309843907, 4428502.60243916), (444507.3374700424, 4428502.104572863), (444507.3650961778, 4428501.606706567), (444507.3927223132, 4428501.10884027), (444507.4203484485, 4428500.610973975), (444507.4479745839, 4428500.113107678), (444507.4756007193, 4428499.61524138), (444507.50322685466, 4428499.117375084), (444507.53085299005, 4428498.619508787), (444507.55847912544, 4428498.121642491), (444507.5861052608, 4428497.623776194), (444507.6137313962, 4428497.125909897), (444507.6413575316, 4428496.6280436), (444507.6689836669, 4428496.130177304), (444507.6966098023, 4428495.632311008), (444507.7242359377, 4428495.134444711), (444507.7518620731, 4428494.636578414), (444507.77948820847, 4428494.138712117), (444507.80711434386, 4428493.64084582), (444507.83474047924, 4428493.142979524), (444507.86236661463, 4428492.645113227), (444507.88999274996, 4428492.147246932), (444507.91761888535, 4428491.649380635), (444507.94524502073, 4428491.151514337), (444507.9728711561, 4428490.653648041), (444508.0004972915, 4428490.155781744), (444508.0281234269, 4428489.657915447), (444508.0557495623, 4428489.160049151), (444508.08337569766, 4428488.662182854), (444508.11100183305, 4428488.164316557), (444508.1386279684, 4428487.666450261), (444508.16625410377, 4428487.168583965), (444508.19388023915, 4428486.670717668), (444508.22150637454, 4428486.172851371), (444508.2491325099, 4428485.674985074), (444508.2767586453, 4428485.177118777), (444508.3043847807, 4428484.679252481), (444508.3320109161, 4428484.181386184), (444508.3596370515, 4428483.683519887), (444508.3872631868, 4428483.1856535915), (444508.4148893222, 4428482.687787294), (444508.4425154576, 4428482.189920998), (444508.47014159296, 4428481.692054701), (444508.49776772835, 4428481.194188404), (444508.52539386373, 4428480.696322108), (444508.5530199991, 4428480.198455811), (444508.5806461345, 4428479.700589514), (444508.6082722699, 4428479.202723217), (444508.6358984052, 4428478.704856921), (444508.6635245406, 4428478.206990625), (444508.691150676, 4428477.709124328), (444508.7187768114, 4428477.211258031)]
    # path_x_y = [(0, 0), (10, 10)]
    # path_x_y.reverse()

    """读取经纬度并转换成UTM的功能"""
    # path_x_y = []
    # with open('hardwareInitShell_Doc/ciee_test.csv', 'r') as fileopen:
    #     fileopen.readline()  # 去表头
    #     fileopen.readline()  # 去表头
    #     content = fileopen.readlines()
    #
    #     for msg_line in content:
    #         # 得到经纬度，然后转换成path_x_y
    #         msg_line_list = msg_line.split(',')
    #         # ciee_test.csv
    #         msg_lon = float(msg_line_list[1])
    #         msg_lat = float(msg_line_list[0])  # weiDu
    #         # caochang_test.csv
    #         # msg_lon = float(msg_line_list[1])
    #         # msg_lat = float(msg_line_list[2])  # weiDu
    #         x, y = ll2xy.ll2xy(lat=msg_lat, lon=msg_lon)
    #         path_x_y.append((x, y))
    #
    #     fileopen.close()

    # 增加功能：从csv文件中加载要跟踪的路径。这里加载路径非常耗时,要启动很久。
    # path_x_y = []
    # with open('记录下的要跟踪的轨迹点2021-01-22 15:12:36.csv', 'r') as fileopen:
    #     fileopen.readline()  # 去表头
    #     content = fileopen.readlines()
    #
    #     for msg_line in content:
    #         # 得到经纬度，然后转换成path_x_y
    #         msg_line_list = msg_line.split(',')
    #         msg_lon = float(msg_line_list[2])
    #         msg_lat = float(msg_line_list[3])
    #         x, y = ll2xy.ll2xy(lat=msg_lat, lon=msg_lon)
    #         path_x_y.append((x, y))
    #
    #     fileopen.close()

    return path_x_y


if __name__ == "__main__":
    # 标志位0：作业标志位，1：障碍物标志位，2：当前作业任务序号int，3当前跟踪点序号int,(4,5传出去）
    is_operation_flag = [0, 0, 0, 0, 0, 0]  # 使用列表，方便参数传递（保证只有一个对象修改完成即可）。为1表示开启，为0表示不开启。
    # 开启线程接收感知消息
    perce_t = threading.Thread(target=perception_process_ser.s_server, args=(is_operation_flag,))
    perce_t.start()
    realtime_lateral_error = [9999]  # 实时横向误差，提供给界面显示误差用

    # # path_file = 'hardwareInitShell_Doc/记录下的要跟踪的轨迹点2021-09-02 11:38:30.csv'  # 等待切换新的格式
    # # path_file = "pathGenerate/fromWangKe/utmpoints_46.csv"
    path_file = "pathGenerate/ab_record/ab_line_2021-09-02_1/ab_list.txt"  # 在此处更换路径文件
    path_x_y = path_load(path_file, x_pos=0, y_pos=1)
    # # path_x_y = [(488102.76871065865, 4466142.772785255), (488103.2673534956, 4466142.75874181), (488103.76599633245, 4466142.744698365), (488104.2646391694, 4466142.73065492), (488104.7632820063, 4466142.716611476), (488105.2619248432, 4466142.702568031), (488105.7605676801, 4466142.688524586), (488106.25921051705, 4466142.674481141), (488106.7578533539, 4466142.660437696), (488107.25649619085, 4466142.646394251), (488107.7551390278, 4466142.632350806), (488108.25378186465, 4466142.618307361), (488108.7524247016, 4466142.604263917), (488109.2510675385, 4466142.590220472), (488109.7497103754, 4466142.576177027), (488110.2483532123, 4466142.562133582), (488110.7469960492, 4466142.548090138), (488111.2456388861, 4466142.534046693), (488111.74428172305, 4466142.520003248), (488112.2429245599, 4466142.505959803), (488112.74156739685, 4466142.4919163585), (488113.2402102338, 4466142.477872914), (488113.73885307065, 4466142.463829469), (488114.2374959076, 4466142.449786024), (488114.7361387445, 4466142.4357425785), (488115.2347815814, 4466142.421699134), (488115.7334244183, 4466142.407655689), (488116.23206725525, 4466142.393612244), (488116.7307100921, 4466142.379568799), (488117.22935292905, 4466142.365525355), (488117.727995766, 4466142.35148191), (488118.22663860285, 4466142.337438465), (488118.7252814398, 4466142.32339502), (488119.2239242767, 4466142.309351576), (488119.7225671136, 4466142.295308131), (488120.2212099505, 4466142.281264686), (488120.71985278744, 4466142.267221241), (488121.2184956243, 4466142.2531777965), (488121.71713846124, 4466142.239134352), (488122.2157812982, 4466142.225090906), (488122.71442413505, 4466142.211047461), (488123.213066972, 4466142.1970040165), (488123.7117098089, 4466142.182960572), (488124.2103526458, 4466142.168917127), (488124.7089954827, 4466142.154873682), (488125.20763831964, 4466142.140830237), (488125.7062811565, 4466142.126786793), (488126.20492399344, 4466142.112743348), (488126.7035668303, 4466142.098699903), (488127.20220966724, 4466142.084656458), (488127.7008525042, 4466142.070613014), (488128.19949534105, 4466142.056569569), (488128.698138178, 4466142.042526124), (488129.1967810149, 4466142.028482679), (488129.6954238518, 4466142.014439234), (488130.1940666887, 4466142.000395789), (488130.69270952564, 4466141.986352344), (488131.1913523625, 4466141.972308899), (488131.68999519944, 4466141.9582654545), (488132.1886380364, 4466141.94422201), (488132.68728087324, 4466141.930178565), (488133.1859237102, 4466141.91613512), (488133.6845665471, 4466141.902091675), (488134.183209384, 4466141.888048231), (488134.6818522209, 4466141.874004786), (488135.18049505784, 4466141.859961341), (488135.6791378947, 4466141.845917896), (488136.17778073164, 4466141.831874452), (488136.67642356857, 4466141.817831007), (488137.17506640544, 4466141.803787562), (488137.6737092424, 4466141.789744116), (488138.1723520793, 4466141.775700672), (488138.6709949162, 4466141.761657227), (488139.1696377531, 4466141.747613782), (488139.66828059003, 4466141.733570337), (488140.1669234269, 4466141.7195268925), (488140.66556626384, 4466141.705483448), (488141.1642091007, 4466141.691440003), (488141.66285193764, 4466141.677396558), (488142.16149477457, 4466141.663353113), (488142.66013761144, 4466141.649309669), (488143.1587804484, 4466141.635266224), (488143.6574232853, 4466141.621222779), (488144.1560661222, 4466141.607179334), (488144.6547089591, 4466141.59313589), (488145.15335179603, 4466141.579092444), (488145.6519946329, 4466141.565048999), (488146.15063746984, 4466141.551005554), (488146.64928030677, 4466141.53696211), (488147.14792314364, 4466141.522918665), (488147.64656598057, 4466141.50887522), (488148.1452088175, 4466141.494831775), (488148.6438516544, 4466141.480788331), (488149.1424944913, 4466141.466744886), (488149.64113732823, 4466141.452701441), (488150.1397801651, 4466141.438657996), (488150.63842300203, 4466141.4246145515), (488151.13706583896, 4466141.410571107), (488151.63570867584, 4466141.396527662), (488152.13435151277, 4466141.382484217), (488152.6329943497, 4466141.3684407715), (488153.13163718657, 4466141.354397327), (488153.6302800235, 4466141.340353882), (488154.12892286043, 4466141.326310437), (488154.6275656973, 4466141.312266992), (488155.12620853423, 4466141.298223548), (488155.62485137116, 4466141.284180103), (488156.12349420803, 4466141.270136658), (488156.62213704496, 4466141.256093213), (488157.12077988184, 4466141.242049769), (488157.61942271877, 4466141.228006324), (488158.1180655557, 4466141.213962879), (488158.61670839257, 4466141.199919434), (488159.1153512295, 4466141.1858759895), (488159.6139940664, 4466141.171832545), (488160.1126369033, 4466141.1577891), (488160.61127974023, 4466141.143745654), (488161.10992257716, 4466141.1297022095), (488161.60856541403, 4466141.115658765), (488162.10720825096, 4466141.10161532), (488162.6058510879, 4466141.087571875), (488163.10449392477, 4466141.07352843), (488163.6031367617, 4466141.059484986), (488164.1017795986, 4466141.045441541), (488164.6004224355, 4466141.031398096), (488165.0990652724, 4466141.017354651), (488165.59770810936, 4466141.003311207), (488166.09635094623, 4466140.989267762), (488166.59499378316, 4466140.975224317), (488167.0936366201, 4466140.961180872), (488167.59227945696, 4466140.9471374275), (488168.0909222939, 4466140.933093982), (488168.5895651308, 4466140.919050537), (488169.0882079677, 4466140.905007092), (488169.5868508046, 4466140.8909636475), (488170.08549364156, 4466140.876920203), (488170.5841364784, 4466140.862876758), (488171.08277931536, 4466140.848833313), (488171.5814221523, 4466140.834789868), (488172.08006498916, 4466140.820746424), (488172.5787078261, 4466140.806702979), (488173.07735066296, 4466140.792659534), (488173.5759934999, 4466140.778616089), (488174.0746363368, 4466140.764572645), (488174.5732791737, 4466140.7505292), (488175.0719220106, 4466140.736485755), (488175.57056484756, 4466140.722442309), (488176.0692076844, 4466140.708398865), (488176.56785052136, 4466140.69435542), (488177.0664933583, 4466140.680311975), (488177.56513619516, 4466140.66626853), (488178.0637790321, 4466140.6522250855), (488178.562421869, 4466140.638181641), (488179.0610647059, 4466140.624138196), (488179.5597075428, 4466140.610094751), (488180.05835037975, 4466140.596051306), (488180.5569932166, 4466140.582007862), (488181.05563605356, 4466140.567964417), (488181.5542788905, 4466140.553920972), (488182.05292172736, 4466140.539877527), (488182.5515645643, 4466140.525834083), (488183.0502074012, 4466140.511790638), (488183.5488502381, 4466140.497747192), (488184.047493075, 4466140.483703747), (488184.54613591195, 4466140.469660303), (488185.0447787488, 4466140.455616858), (488185.54342158575, 4466140.441573413), (488186.0420644227, 4466140.427529968), (488186.54070725956, 4466140.413486524), (488187.0393500965, 4466140.399443079), (488187.53799293336, 4466140.385399634), (488188.0366357703, 4466140.371356189), (488188.5352786072, 4466140.3573127445), (488189.0339214441, 4466140.3432693), (488189.532564281, 4466140.329225855), (488190.03120711795, 4466140.31518241), (488190.5298499548, 4466140.301138965), (488191.02849279175, 4466140.28709552), (488191.5271356287, 4466140.273052075), (488192.02577846556, 4466140.25900863), (488192.5244213025, 4466140.244965185), (488193.0230641394, 4466140.230921741), (488193.5217069763, 4466140.216878296), (488194.0203498132, 4466140.202834851), (488194.51899265015, 4466140.188791406), (488195.017635487, 4466140.174747962), (488195.51627832395, 4466140.160704517), (488196.0149211609, 4466140.146661072), (488196.51356399775, 4466140.132617627), (488197.0122068347, 4466140.1185741825), (488197.5108496716, 4466140.104530738), (488198.0094925085, 4466140.090487293), (488198.5081353454, 4466140.076443847), (488199.00677818235, 4466140.0624004025), (488199.5054210192, 4466140.048356958), (488200.00406385615, 4466140.034313513), (488200.5027066931, 4466140.020270068), (488201.00134952995, 4466140.006226623), (488201.4999923669, 4466139.992183179), (488201.9986352038, 4466139.978139734), (488202.4972780407, 4466139.964096289), (488202.9959208776, 4466139.950052844), (488203.4945637145, 4466139.9360094), (488203.9932065514, 4466139.921965955), (488204.49184938835, 4466139.90792251), (488204.9904922252, 4466139.893879065), (488205.48913506215, 4466139.8798356205), (488205.9877778991, 4466139.865792176), (488206.48642073595, 4466139.85174873), (488206.9850635729, 4466139.837705285), (488207.4837064098, 4466139.8236618405), (488207.9823492467, 4466139.809618396), (488208.4809920836, 4466139.795574951), (488208.97963492054, 4466139.781531506), (488209.4782777574, 4466139.767488061), (488209.97692059434, 4466139.753444617)]
    path_x_y = [(488235.0488366564, 4466139.434445432), (488234.5518094487, 4466139.4769816445),
                (488234.05478224094, 4466139.519517857), (488233.55775503325, 4466139.56205407),
                (488233.06072782556, 4466139.604590282), (488232.5637006178, 4466139.647126494),
                (488232.0666734101, 4466139.689662706), (488231.56964620244, 4466139.732198918),
                (488231.0726189947, 4466139.774735131), (488230.575591787, 4466139.817271343),
                (488230.0785645793, 4466139.859807556), (488229.58153737156, 4466139.902343768),
                (488229.08451016387, 4466139.944879981), (488228.5874829562, 4466139.987416193),
                (488228.09045574843, 4466140.029952405), (488227.59342854074, 4466140.072488617),
                (488227.09640133305, 4466140.115024829), (488226.5993741253, 4466140.157561042),
                (488226.1023469176, 4466140.2000972545), (488225.6053197099, 4466140.242633467),
                (488225.1082925022, 4466140.285169679), (488224.6112652945, 4466140.327705891),
                (488224.11423808674, 4466140.370242104), (488223.61721087905, 4466140.412778316),
                (488223.12018367136, 4466140.455314528), (488222.6231564636, 4466140.49785074),
                (488222.1261292559, 4466140.540386953), (488221.62910204823, 4466140.5829231655),
                (488221.1320748405, 4466140.625459378), (488220.6350476328, 4466140.66799559),
                (488220.1380204251, 4466140.710531802), (488219.64099321736, 4466140.753068015),
                (488219.14396600967, 4466140.795604227), (488218.646938802, 4466140.838140439),
                (488218.1499115942, 4466140.880676651), (488217.65288438654, 4466140.9232128635),
                (488217.15585717885, 4466140.965749077), (488216.6588299711, 4466141.008285289),
                (488216.1618027634, 4466141.050821501), (488215.6647755557, 4466141.093357713),
                (488215.167748348, 4466141.135893926), (488214.6707211403, 4466141.178430138),
                (488214.1736939326, 4466141.22096635), (488213.67666672484, 4466141.263502562),
                (488213.17963951716, 4466141.306038775), (488212.68261230947, 4466141.348574988),
                (488212.1855851017, 4466141.3911112), (488211.688557894, 4466141.433647412),
                (488211.19153068634, 4466141.476183624), (488210.6945034786, 4466141.518719836),
                (488210.1974762709, 4466141.561256049), (488209.7004490632, 4466141.603792261),
                (488209.20342185546, 4466141.646328473), (488208.7063946478, 4466141.688864686),
                (488208.2093674401, 4466141.731400898), (488207.71234023233, 4466141.773937111),
                (488207.21531302464, 4466141.816473323), (488206.71828581695, 4466141.859009535),
                (488206.2212586092, 4466141.901545747), (488205.7242314015, 4466141.94408196),
                (488205.2272041938, 4466141.986618172), (488204.7301769861, 4466142.0291543845),
                (488204.2331497784, 4466142.071690597), (488203.73612257064, 4466142.114226809),
                (488203.23909536295, 4466142.156763022), (488202.74206815526, 4466142.199299234),
                (488202.2450409475, 4466142.241835446), (488201.7480137398, 4466142.284371658),
                (488201.25098653213, 4466142.32690787), (488200.7539593244, 4466142.369444083),
                (488200.2569321167, 4466142.411980296), (488199.759904909, 4466142.454516508),
                (488199.26287770126, 4466142.49705272), (488198.76585049357, 4466142.539588933),
                (488198.2688232859, 4466142.582125145), (488197.77179607813, 4466142.624661357),
                (488197.27476887044, 4466142.667197569), (488196.77774166275, 4466142.709733781),
                (488196.280714455, 4466142.7522699945), (488195.7836872473, 4466142.794806207),
                (488195.2866600396, 4466142.837342419), (488194.7896328319, 4466142.879878631),
                (488194.2926056242, 4466142.922414843), (488193.7955784165, 4466142.964951056),
                (488193.29855120875, 4466143.007487268), (488192.80152400106, 4466143.05002348),
                (488192.30449679337, 4466143.092559692), (488191.8074695856, 4466143.135095905),
                (488191.31044237793, 4466143.177632118), (488190.81341517024, 4466143.22016833),
                (488190.3163879625, 4466143.262704542), (488189.8193607548, 4466143.305240754),
                (488189.3223335471, 4466143.347776967), (488188.82530633936, 4466143.390313179),
                (488188.3282791317, 4466143.432849391), (488187.831251924, 4466143.4753856035),
                (488187.33422471624, 4466143.517921816), (488186.83719750855, 4466143.560458029),
                (488186.34017030086, 4466143.602994241), (488185.8431430931, 4466143.645530453),
                (488185.3461158854, 4466143.688066665), (488184.8490886777, 4466143.730602877),
                (488184.35206147, 4466143.77313909), (488183.8550342623, 4466143.815675302),
                (488183.35800705454, 4466143.858211515), (488182.86097984685, 4466143.900747727),
                (488182.36395263916, 4466143.94328394), (488181.8669254314, 4466143.985820152),
                (488181.3698982237, 4466144.028356364), (488180.87287101604, 4466144.070892576),
                (488180.3758438083, 4466144.113428788), (488179.8788166006, 4466144.155965001),
                (488179.3817893929, 4466144.198501213), (488178.88476218516, 4466144.241037426),
                (488178.38773497747, 4466144.283573638), (488177.8907077698, 4466144.32610985),
                (488177.39368056203, 4466144.368646063), (488176.89665335434, 4466144.411182275),
                (488176.39962614665, 4466144.453718487), (488175.9025989389, 4466144.496254699),
                (488175.4055717312, 4466144.538790912), (488174.9085445235, 4466144.5813271245),
                (488174.4115173158, 4466144.623863337), (488173.9144901081, 4466144.666399549),
                (488173.4174629004, 4466144.708935761), (488172.92043569265, 4466144.751471974),
                (488172.42340848496, 4466144.794008186), (488171.92638127727, 4466144.836544398),
                (488171.4293540695, 4466144.87908061), (488170.93232686183, 4466144.9216168225),
                (488170.43529965414, 4466144.964153036), (488169.9382724464, 4466145.006689248),
                (488169.4412452387, 4466145.04922546), (488168.944218031, 4466145.091761672),
                (488168.44719082327, 4466145.134297884), (488167.9501636156, 4466145.176834097),
                (488167.4531364079, 4466145.219370309), (488166.95610920014, 4466145.261906521),
                (488166.45908199245, 4466145.3044427335), (488165.96205478476, 4466145.346978947),
                (488165.465027577, 4466145.389515159), (488164.9680003693, 4466145.432051371),
                (488164.47097316163, 4466145.474587583), (488163.9739459539, 4466145.517123795),
                (488163.4769187462, 4466145.559660008), (488162.97989153845, 4466145.60219622),
                (488162.48286433076, 4466145.644732432), (488161.98583712307, 4466145.687268645),
                (488161.4888099153, 4466145.729804857), (488160.9917827076, 4466145.77234107),
                (488160.49475549994, 4466145.814877282), (488159.9977282922, 4466145.857413494),
                (488159.5007010845, 4466145.899949706), (488159.0036738768, 4466145.942485919),
                (488158.50664666906, 4466145.985022131), (488158.0096194614, 4466146.0275583435),
                (488157.5125922537, 4466146.070094556), (488157.01556504593, 4466146.112630768),
                (488156.51853783824, 4466146.155166981), (488156.02151063055, 4466146.197703193),
                (488155.5244834228, 4466146.240239405), (488155.0274562151, 4466146.282775617),
                (488154.5304290074, 4466146.325311829), (488154.0334017997, 4466146.367848042),
                (488153.536374592, 4466146.4103842545), (488153.0393473843, 4466146.452920467),
                (488152.54232017655, 4466146.495456679), (488152.04529296886, 4466146.537992891),
                (488151.5482657612, 4466146.580529104), (488151.0512385534, 4466146.623065316),
                (488150.55421134573, 4466146.665601528), (488150.05718413804, 4466146.70813774),
                (488149.5601569303, 4466146.750673953), (488149.0631297226, 4466146.793210166),
                (488148.5661025149, 4466146.835746378), (488148.06907530717, 4466146.87828259),
                (488147.5720480995, 4466146.920818802), (488147.0750208918, 4466146.963355015),
                (488146.57799368404, 4466147.005891227), (488146.08096647635, 4466147.048427439),
                (488145.58393926866, 4466147.090963651), (488145.0869120609, 4466147.133499864),
                (488144.5898848532, 4466147.176036077), (488144.09285764553, 4466147.218572289),
                (488143.5958304378, 4466147.261108501), (488143.0988032301, 4466147.303644713),
                (488142.60177602235, 4466147.346180926), (488142.10474881466, 4466147.388717138),
                (488141.60772160697, 4466147.43125335), (488141.1106943992, 4466147.4737895625),
                (488140.61366719153, 4466147.516325775), (488140.11663998384, 4466147.558861988),
                (488139.6196127761, 4466147.6013982), (488139.1225855684, 4466147.643934412),
                (488138.6255583607, 4466147.686470624), (488138.12853115296, 4466147.729006836),
                (488137.6315039453, 4466147.771543049), (488137.1344767376, 4466147.814079261),
                (488136.63744952984, 4466147.8566154735), (488136.14042232215, 4466147.899151686),
                (488135.64339511446, 4466147.941687899), (488135.1463679067, 4466147.984224111),
                (488134.649340699, 4466148.026760323), (488134.15231349133, 4466148.069296535),
                (488133.6552862836, 4466148.111832747), (488133.1582590759, 4466148.15436896),
                (488132.6612318682, 4466148.196905172), (488132.16420466045, 4466148.239441385),
                (488131.66717745276, 4466148.281977597), (488131.1701502451, 4466148.324513809),
                (488130.6731230373, 4466148.367050022), (488130.17609582964, 4466148.409586234),
                (488129.67906862195, 4466148.452122446), (488129.1820414142, 4466148.494658658),
                (488128.6850142065, 4466148.53719487), (488128.1879869988, 4466148.5797310835),
                (488127.69095979107, 4466148.622267296), (488127.1939325834, 4466148.664803508),
                (488126.6969053757, 4466148.70733972), (488126.19987816794, 4466148.749875933),
                (488125.70285096025, 4466148.792412145), (488125.20582375256, 4466148.834948357),
                (488124.7087965448, 4466148.877484569), (488124.2117693371, 4466148.920020781),
                (488123.71474212944, 4466148.9625569945), (488123.2177149217, 4466149.005093207),
                (488122.720687714, 4466149.047629419), (488122.22366050625, 4466149.090165631),
                (488121.72663329856, 4466149.132701843), (488121.22960609087, 4466149.175238056),
                (488120.7325788831, 4466149.217774268), (488120.23555167543, 4466149.26031048),
                (488119.73852446774, 4466149.3028466925), (488119.24149726, 4466149.345382906),
                (488118.7444700523, 4466149.387919118), (488118.2474428446, 4466149.43045533),
                (488117.75041563687, 4466149.472991542), (488117.2533884292, 4466149.515527754),
                (488116.7563612215, 4466149.558063967), (488116.25933401374, 4466149.600600179),
                (488115.76230680605, 4466149.643136391), (488115.26527959836, 4466149.685672604),
                (488114.7682523906, 4466149.728208816), (488114.2712251829, 4466149.770745029),
                (488113.77419797523, 4466149.813281241), (488113.2771707675, 4466149.855817453),
                (488112.7801435598, 4466149.898353665), (488112.2831163521, 4466149.940889877),
                (488111.78608914436, 4466149.98342609), (488111.28906193667, 4466150.025962302),
                (488110.792034729, 4466150.068498515), (488110.2950075212, 4466150.111034727),
                (488109.79798031354, 4466150.15357094), (488109.30095310585, 4466150.196107152),
                (488108.8039258981, 4466150.238643364), (488108.3068986904, 4466150.281179576),
                (488107.8098714827, 4466150.323715788), (488107.312844275, 4466150.366252001),
                (488106.8158170673, 4466150.4087882135), (488106.3187898596, 4466150.451324426),
                (488105.82176265185, 4466150.493860638), (488105.32473544416, 4466150.53639685),
                (488104.82770823647, 4466150.578933063), (488104.3306810287, 4466150.621469275),
                (488103.833653821, 4466150.664005487), (488103.33662661334, 4466150.706541699),
                (488102.8395994056, 4466150.749077912), (488102.3425721979, 4466150.791614125),
                (488101.84554499015, 4466150.834150337), (488101.34851778246, 4466150.876686549),
                (488100.8514905748, 4466150.919222761), (488100.354463367, 4466150.961758974),
                (488099.85743615933, 4466151.004295186), (488099.36040895164, 4466151.046831398),
                (488098.8633817439, 4466151.08936761), (488098.3663545362, 4466151.1319038225),
                (488097.8693273285, 4466151.174440036), (488097.37230012077, 4466151.216976248),
                (488096.8752729131, 4466151.25951246), (488096.3782457054, 4466151.302048672),
                (488095.88121849764, 4466151.344584885), (488095.38419128995, 4466151.387121097),
                (488094.88716408226, 4466151.429657309), (488094.3901368745, 4466151.472193521),
                (488093.8931096668, 4466151.514729734), (488093.39608245913, 4466151.557265947),
                (488092.8990552514, 4466151.599802159), (488092.4020280437, 4466151.642338371)]
    path_x_y = [(488087.6214051248, 4466504.71990286), (488087.605177278, 4466504.220977992),
                (488087.5889494312, 4466503.722053123), (488087.57272158447, 4466503.2231282545),
                (488087.5564937377, 4466502.724203385), (488087.5402658909, 4466502.225278517),
                (488087.5240380441, 4466501.726353648), (488087.5078101973, 4466501.22742878),
                (488087.4915823506, 4466500.728503911), (488087.4753545038, 4466500.229579043),
                (488087.459126657, 4466499.7306541735), (488087.4428988102, 4466499.231729305),
                (488087.4266709634, 4466498.732804436), (488087.4104431167, 4466498.233879568),
                (488087.3942152699, 4466497.734954699), (488087.3779874231, 4466497.236029831),
                (488087.3617595763, 4466496.737104962), (488087.3455317295, 4466496.2381800935),
                (488087.3293038827, 4466495.739255224), (488087.313076036, 4466495.240330356),
                (488087.2968481892, 4466494.741405487), (488087.2806203424, 4466494.242480619),
                (488087.2643924956, 4466493.74355575), (488087.24816464883, 4466493.244630882),
                (488087.2319368021, 4466492.7457060125), (488087.2157089553, 4466492.246781144),
                (488087.1994811085, 4466491.747856275), (488087.1832532617, 4466491.248931407),
                (488087.16702541494, 4466490.750006538), (488087.1507975682, 4466490.25108167),
                (488087.1345697214, 4466489.752156801), (488087.1183418746, 4466489.253231932),
                (488087.10211402783, 4466488.754307064), (488087.08588618104, 4466488.255382195),
                (488087.0696583343, 4466487.756457327), (488087.0534304875, 4466487.257532458),
                (488087.0372026407, 4466486.75860759), (488087.02097479394, 4466486.2596827205),
                (488087.00474694715, 4466485.760757852), (488086.9885191004, 4466485.261832983),
                (488086.9722912536, 4466484.762908115), (488086.95606340683, 4466484.263983246),
                (488086.93983556004, 4466483.765058378), (488086.92360771325, 4466483.266133509),
                (488086.9073798665, 4466482.76720864), (488086.8911520197, 4466482.268283771),
                (488086.87492417294, 4466481.769358903), (488086.85869632615, 4466481.270434034),
                (488086.84246847936, 4466480.771509166), (488086.82624063257, 4466480.272584297),
                (488086.81001278583, 4466479.773659429), (488086.79378493904, 4466479.2747345595),
                (488086.77755709225, 4466478.775809691), (488086.76132924546, 4466478.276884822),
                (488086.7451013987, 4466477.777959954), (488086.72887355194, 4466477.279035085),
                (488086.71264570515, 4466476.780110217), (488086.69641785836, 4466476.281185348),
                (488086.68019001157, 4466475.782260479), (488086.6639621648, 4466475.28333561),
                (488086.64773431804, 4466474.784410742), (488086.63150647125, 4466474.285485873),
                (488086.61527862446, 4466473.786561005)]
    path_x_y.reverse()
    # path_x_y.reverse()

    target_speed = 5  # 目标速度，cmd_v是计算出来的指令速度，为控制量
    path_target_point_index = 0  # 后期记录断点，方便续作
    Kp = 1  # 拖拉机转角p控制器参数，测试时适当调整
    threshold_distance = 3  # 拖拉机判断是否到达预瞄点的距离阈值，拖拉机前后轮距3.1m，测试时候适当调整
    backward_flag = 0  # 默认后退标志为0，表示前进模式
    machine_operation_status = "down"  # 机具状态（当前针对播种作业）
    task_index = 0
    is_endtask = 1
    driving_mode = 2  # 0,1,2分别对应手动，扭矩请求，自动模式
    machine_operation_enabled = 1  # 默认为1，开启机具控制

    # 开启一个线程去计算误差，显示出来
    ui_window_t = threading.Thread(target=ui_window, args=(realtime_lateral_error, is_operation_flag))
    ui_window_t.start()

    # for i in range(100000000):
    #     realtime_lateral_error[0] = i

    # 单次调试时使用
    # path_following(target_speed, path_x_y, path_target_point_index, Kp, threshold_distance=threshold_distance,
    #                path_x_y_file=path_file, realtime_lateral_error=realtime_lateral_error, backward_flag=backward_flag,
    #                machine_operation_status=machine_operation_status, task_index=task_index, is_endtask=is_endtask,
    #                iswork_flag=is_operation_flag, driving_mode=driving_mode, machine_operation_enabled=machine_operation_enabled)

    # 进行任务执行
    task_execute = WorkOperation(is_operation_flag=is_operation_flag, realtime_lateral_error=realtime_lateral_error)
    # task_execute.is_task_legacy_and_execute()     # 需要监测未完成时候再用
    x_y_path_file_index = 0
    task_point_ind = 0
    task_execute.task_execute(x_y_path_file_index, task_point_ind, driving_mode, machine_operation_enabled)

    # 开启一个线程进行HMI,这个适合用线程

    # 开启一个线程，进行故障信号的检测

    # 室外路面建议2公里以上
    # target_speed = 0  # 单位为km/h
    # imu = classOfIMU_CGI610.Imu()
    # # tractor = classOfYunLe.Car(1)
    # tractor = classOfDongFengSF2204.Tractor("send")
    # vcu_cmd = classOfDongFengSF2204.VCUCmd(tractor)
    # path = r'pathRecord/'
    # record = classOfDataRecord.DataRecord(path + "行驶时记录数据" +
    #                                       time.strftime('%Y-%m-%d %H-%M-%S', time.localtime(time.time())) +
    #                                       ".csv")
    # cmd_steering = 0
    # cmd_v = 0.5
    # Lf = 1.0  # look-ahead distance
    #
    # #  初始化第一个跟踪的路径点
    # i = 0
    # length = len(path_x_y)
    #
    # # 测试一下运行一次时间开销:150ms左右
    # while i <= length-2:  # 当跟踪的最后一个点时候，结束跟踪，小车制动。因为每步只跟踪一个点，所以i的范围可以从0~length-1,设为len-2保险
    #     starttime = time.time()
    #     """全部处理完再合成指令发送"""
    #     waypoint = path_x_y[i]
    #
    #     GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, inte_navi_status, imu_altitude, imu_satellite_num, imu_warning = imu.stateOfCar()  # 执行一次采样一次。这里用imu.stateOfCar实现
    #
    #     # 20210820新增组合导航状态判断，并记录日志
    #     # 判断inte_navi status,不是42则停车等待。
    #     # while inte_navi_status != 42:
    #     #     # 发送停车指令
    #     #     vcu_cmd.send_motion_ctrl_msg(2, "空挡", 0.0, 0, 1)
    #     #     time.sleep(0.2)
    #     #     inte_navi_status = imu.stateOfCar()[8]
    #
    #
    #     print(headingAngle, x, y, v)
    #     vehicle_x_y = (x, y)
    #     # 计算小车是否到达跟踪点
    #     if isArrive(vehicle_x_y, path_x_y, i) == False:
    #         # TODO 进行横向跟踪控制，将控制指令的产生用函数来实现
    #         # 判断小车航向角是否满足要求:
    #
    #         # 先判断向量的象限，再计算目标夹角
    #
    #         # 计算小车指向目标点的向量
    #         car2goalPoint = (waypoint[0]-x, waypoint[1]-y)
    #
    #         goalHeadingAng = calcAngOfY_Axis(car2goalPoint)
    #         #etc = goalAng - headingAngle
    #
    #
    #         # 转向控制pid实现，需要调参
    #         #target_headingAngle=0 #小车指向目标点航向角
    #         cmd_steering = steering_ctrl.PControl(goalHeadingAng, headingAngle, 1)   #这里应该是需要转换角度到相同的起始部位。heading是正北顺时针，goalAng方向是目标方向，是小车指向目标点的方向。
    #         # cmd_steering = steering_ctrl.pure_pursuit(goalHeadingAng, headingAngle, Lf)
    #         print('goalHeadingAng-headingAngle=', goalHeadingAng-headingAngle)
    #         #cmd_steering = headingAngle+cmd_steering
    #         #cmd_steering = cmd_steering
    #
    #         # (小车情况)限定在小车接收范围内，+—120°以内，精度0.1。并且误差在60°以上时就方向盘打满.在60°以下进行缩放
    #         # （拖拉机）计算航向角误差在40°外则打满，不在则取系数为1，暂时先这样，后期再调整。
    #         if cmd_steering > 40:   # 直接打满40°
    #             cmd_steering = 40.0
    #         elif cmd_steering < -40:
    #             cmd_steering = -40.0
    #         else:
    #             cmd_steering = cmd_steering*1
    #         # 以上，符合条件的转角命令生成完毕
    #
    #     else:
    #         # 更新跟踪点为下一个点
    #         i = i + 1
    #
    #     # cmd_v filter
    #     if cmd_v <= 0.0:
    #         cmd_v = 0.0
    #     print('cmd_steering=', cmd_steering, 'cmd_v', cmd_v)
    #     target_speed = 0             # 20210627临时
    #     vcu_cmd.send_motion_ctrl_msg(2, "前进高档", cmd_steering, target_speed, 0)
    #
    #     # 记录状态。需要记录的数据字段：GPSweek，GPStime，经度，纬度，航向角，x,y,小车发送的指令（转角，速度）.一共九个。
    #     # 需求：再增加一个字段，存跟踪的i
    #     # GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v
    #     msg = [GPSWeek, GPSTime, imu_lon, imu_lat, headingAngle, x, y, v, cmd_steering, cmd_v]
    #     record.data_record(msg)
    #
    #     endtime = time.time()
    #     print("mpns_循环花销"+str(endtime-starttime)+"s")
    #     # 按步上传数据
    #     # collect_info.Info_tran(gps_time=time.time(), gps_status=inte_navi_status, gps_way=1, latitude=imu_lat,
    #     #                        longitude=imu_lon, altitude=imu_altitude, speed=v, satellite_num=imu_satellite_num,
    #     #                        warning=imu_warning)
    #     # collect_info.Info_tran(1,2,3,4,5,6,7,8,9)  # debug
    #
    # # collect_info.Info_close()
    #
    # # 最后需要停下来，制动
    # vcu_cmd.send_motion_ctrl_msg(2, "空挡", 0.0, 0, 1)
    # del record
    #
    # while 1:
    #     print('死循环等待')
    #     time.sleep(1)
