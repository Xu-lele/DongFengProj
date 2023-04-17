# dbc文件解析类
# 进行dbc的解析
# 四个消息字段：发送消息message('NAVI', 0x18ff911c, True, 8, None)、message('NAVI2', 0x18ff921c, True, 8, None)、
# message('NAVI3', 0x18ff931c, True, 8, None)，接收消息message('VC6', 0x18ffa127, True, 8, None)
# 要实现消息的解析和发送

import cantools
import os
import can
import time
import numpy as np
import threading

class CANMsgTrans(object):
    db = 0

    def __init__(self, dbc_name):
        self.dbc_name = dbc_name
        dbc_path = os.path.join(os.getcwd(), dbc_name)
        self.db = cantools.database.load_file(dbc_path)
        # print(self.db)

    def can_msg_produce(self, msg_name, msg_list):
        msg = self.db.get_message_by_name(msg_name)
        # 消息发送初始化
        msg_data = {}
        j = 0
        for i in msg.signal_tree:
            msg_data[i] = msg_list[j]
            j = j + 1
        print(msg_data)
        data = msg.encode(msg_data)
        # message = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)
        msg_frame_id = msg.frame_id
        if msg.frame_id == 0x21C:
            msg_frame_id = 0x18FF921C
        if msg.frame_id == 0x11C:
            msg_frame_id = 0x18FF911C
        if msg.frame_id == 0x31C:
            msg_frame_id = 0x18FF931C
        message = can.Message(arbitration_id=msg_frame_id, data=data, is_extended_id=True)
        return message


class Tractor(object):
    __NAVI_msg_list = [0.0, 0, 0, 2, 0, 0]
    __NAVI2_msg_list = [3, 0, 250, 250, 250]
    # __NAVI3_msg_list = [0, 0, 0, 0]    # 第一版协议
    __NAVI3_msg_list = [0, 0, 0, 0, 0, 0, 0, 0]  # debug 20210930 已修改完成
    __VC6_msg_list = [0, 0, 0, 0, 0, 0]

    dbc_config_path = os.path.abspath(os.path.dirname(__file__))
    dbc_config_path = dbc_config_path + "/config/dongFeng2204_2.dbc"

    __can_msg_trans = CANMsgTrans(dbc_config_path)

    # __msg = can_msg_trans.can_msg_produce("NAVI", self.__NAVI_msg_list)

    def __init__(self, flag):
        # 新增权限赋予
        # print(os.system("./component_main/modprobe_peak_usb_forDongFeng2204"))  # 这样会不会使得成为单例模式？
        # print(os.system("./component_main/modprobe_vcan"))    # 测试时候使用

        self.can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=250000)
        if flag == "send":  # 当前是发送消息的功能
            self.task_NAVI = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NAVI", self.__NAVI_msg_list), period=0.1, duration=None,
                store_task=True)  # 一个task值对应一个can.Message.arbitration_id
            # task.modify_data(msg)

            self.task_NAVI2 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NAVI2", self.__NAVI2_msg_list), period=0.1,
                duration=None, store_task=True)
            print('debug navi2')
            self.task_NAVI3 = self.can_bus.send_periodic(
                self.__can_msg_trans.can_msg_produce("NAVI3", self.__NAVI3_msg_list), period=0.1,
                duration=None, store_task=True)
            # self.task_VC6 = self.can_bus.send_periodic(self.__can_msg_trans.can_msg_produce("VC6", self.__VC6_msg_list),
            #                                            period=0.1,
            #                                            duration=None, store_task=True)
        else:  # 当前是接受消息的功能
            pass

    def send_msg(self, msg_name, cmd_list):
        # 把can_msg的生成放进来
        can_msg = self.__can_msg_trans.can_msg_produce(msg_name, cmd_list)
        if msg_name == "NAVI":
            self.task_NAVI.modify_data(can_msg)
        if msg_name == "NAVI2":
            self.task_NAVI2.modify_data(can_msg)
        if msg_name == "NAVI3":
            self.task_NAVI3.modify_data(can_msg)
        # if msg_name == "VC6":
        #     self.task_VC6.modify_data(can_msg)

    def recv_msg_full_dbc_id(self):
        # 等待接收，直到收到信息才会返回。返回dbc里的两个反馈，其他的不反馈。
        get_data = self.can_bus.recv()
        while 1:
            if get_data.arbitration_id == 0x18FFA127:  # vcan会丢弃前面的，完整的是0x18FFA127,这个是扩展帧的原因
                # print("收到一条信息")
                # motion_info = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
                motion_info = self.__can_msg_trans.db.decode_message(0x18FFA127,
                                                                     get_data.data)  # eg:{'gearnum': 14, 'IO_vVeh': 51.1328125, 'Rx_valuePositionSensorFrmEHR': 68, 'Rx_valueSumofDraftFrmEHR': 85, 'Rx_staErr1': 0, 'Rx_staErr2': 1, 'Rx_rawwheelangle': 19}
                # 增加明文信息
                motion_dict = {"档位": motion_info["gearnum"], "车速": motion_info["IO_vVeh"],
                               "提升器高度": motion_info["Rx_valuePositionSensorFrmEHR"],
                               "提升器合力": motion_info["Rx_valueSumofDraftFrmEHR"],
                               "车不动原因": motion_info["Rx_staErr1"], "前轮转角": motion_info["Rx_rawwheelangle"]}
                # print(motion_dict)
                return [0x18FFA127, motion_dict]
            elif get_data.arbitration_id == 0x18FFA227:
                security_info = self.__can_msg_trans.db.decode_message(0x18FFA227,
                                                                       get_data.data)  # eg:{'Rx_staBreak': 17, 'Rx_trqEngActFrmEMS': -91, 'Rx_staWheelmotorError': 51, 'Rx_rawSteerlPos': 21828}
                security_dict = {"刹车状态": security_info["Rx_staBreak"], "发动机扭矩百分比": security_info["Rx_trqEngActFrmEMS"],
                                 "方向盘故障": security_info["Rx_staWheelmotorError"],
                                 "方向盘位置传感器值": security_info["Rx_rawSteerlPos"]}
                # print(security_dict)
                return [0x18FFA227, security_dict]
            else:
                # print("收到其他arbitration_id：", get_data.arbitration_id)
                get_data = self.can_bus.recv()
                # return [get_data.arbitration_id, get_data]

    def recv_msg_full(self):
        get_data = self.can_bus.recv()
        return get_data
        # while 1:
        #     if get_data.arbitration_id == 0x18FEF200:   # vcan会丢弃前面的，完整的是0x18FFA127,这个是扩展帧的原因
        #         print("收到一条信息")
        #         #VC6_Info = self.__can_msg_trans.db.decode_message(get_data.arbitration_id, get_data.data)
        #
        #         #VC6_Info = self.__can_msg_trans.db.decode_message(0x18FFA127, get_data.data)
        #         #print(VC6_Info)
        #         # time.sleep(0.1)
        #         print(get_data)
        #         return 0
        #     else:
        #         print("没收到，继续收，arbitration_id为：", get_data.arbitration_id)
        #         get_data = self.can_bus.recv()

    def offline_msg_parsing_(self, ID, data):
        plaintext = self.__can_msg_trans.db.decode_message(ID, data)
        return plaintext

    def offline_msg_parsing_plaintext(self, ID, data):
        plaintext = self.__can_msg_trans.db.decode_message(ID, data)
        if ID == 0x18ff911c:  # 底盘控制，{'Rx_vReqFrmNAVI': 0.0, 'Rx_numGrReqFrmNAVI': -3, 'Rx_cmdHitchFrmNAVI': 0, 'Rx_staFrmNAVI': 0, 'Rx_cmdStartEngFrmNAVI': 0, 'Rx_cmdStarPTOFrmNAVI': 0}
            # 先换名字，后改值
            plaintext['车速kmph'] = plaintext.pop("Rx_vReqFrmNAVI")
            plaintext['档位'] = plaintext.pop("Rx_numGrReqFrmNAVI")
            if plaintext['档位'] == 0:
                plaintext['档位'] = '空挡'
            elif plaintext['档位'] == 1:
                plaintext['档位'] = '前进挡'
            elif plaintext['档位'] == -1:
                plaintext['档位'] = '后退档'
            else:
                plaintext['档位'] = '未定义'

            plaintext['提升器'] = plaintext.pop("Rx_cmdHitchFrmNAVI")
            if plaintext['提升器'] == 0xf0:
                plaintext['提升器'] = '提升'
            elif plaintext['提升器'] == 0x0f:
                plaintext['提升器'] = '下降'
            else:
                plaintext['提升器'] = '不动作'

            plaintext['手自模式'] = plaintext.pop("Rx_staFrmNAVI")
            if plaintext['手自模式'] == 0:
                plaintext['手自模式'] = '手动模式'
            elif plaintext['手自模式'] == 2:
                plaintext['手自模式'] = '自动模式'
            else:
                plaintext['手自模式'] = '未定义'

            plaintext['点火熄火'] = plaintext.pop("Rx_cmdStartEngFrmNAVI")
            if plaintext['点火熄火'] == 0x0f:
                plaintext['点火熄火'] = '点火'
            elif plaintext['点火熄火'] == 0xf0:
                plaintext['点火熄火'] = '熄火'
            else:
                plaintext['点火熄火'] = '不动作'

            plaintext['PTO'] = plaintext.pop("Rx_cmdStarPTOFrmNAVI")
            if plaintext['PTO'] == 1:
                plaintext['PTO'] = '结合'
            elif plaintext['PTO'] == 0:
                plaintext['PTO'] = '断开'
            else:
                plaintext['PTO'] = '无动作'

            # 改值

        if ID == 0x18ff921c:  # 作业设置，{'Rx_vVehWrkSpdSetFrmNAVI2': 0.0, 'Rx_valueDeepSetFrmNAVI2': 0, 'Rx_valueHighLmtSetFrmNAVI2': 230, 'Rx_valueDropSpeedSetFrmNAVI2': 200, 'Rx_valueModeSetFrmNAVI2': 250}
            plaintext['选档车速'] = plaintext.pop("Rx_vVehWrkSpdSetFrmNAVI2")
            plaintext['耕深设置'] = plaintext.pop("Rx_valueDeepSetFrmNAVI2")
            plaintext['限高设置'] = plaintext.pop("Rx_valueHighLmtSetFrmNAVI2")
            plaintext['下降速度设置'] = plaintext.pop("Rx_valueDropSpeedSetFrmNAVI2")
            plaintext['力位控制'] = plaintext.pop("Rx_valueModeSetFrmNAVI2")

        if ID == 0x18ff931c:  # 液压设置，{'Rx_rawValve1SetFrmNAVI3': 0, 'Rx_rawValve2SetFrmNAVI3': 0, 'Rx_rawValve3SetFrmNAVI3': 0, 'Rx_rawValve4SetFrmNAVI3': 0}
            plaintext['后液压输出1设置'] = plaintext.pop("Rx_rawValve1SetFrmNAVI3") + 100
            plaintext['后液压输出2设置'] = plaintext.pop("Rx_rawValve2SetFrmNAVI3") + 100
            plaintext['后液压输出3设置'] = plaintext.pop("Rx_rawValve3SetFrmNAVI3") + 100
            plaintext['后液压输出4设置'] = plaintext.pop("Rx_rawValve4SetFrmNAVI3") + 100

        if ID == 0x18ffa127:  # 状态反馈，{'gearnum': 1, 'IO_vVeh': 0.0, 'Rx_valuePositionSensorFrmEHR': 217, 'Rx_valueSumofDraftFrmEHR': 137, 'Rx_staOfCaliStaFrmEHR1': 0, 'Rx_staOfCaliStaFrmEHR2': 1}
            plaintext['当前档位'] = plaintext.pop("gearnum")
            if plaintext['当前档位'] == 1:
                plaintext['当前档位'] = '前进挡'
            elif plaintext['当前档位'] == 0:
                plaintext['当前档位'] = '空挡'
            elif plaintext['当前档位'] == -1:
                plaintext['当前档位'] = '倒挡'

            plaintext['当前车速kmph'] = plaintext.pop("IO_vVeh")
            plaintext['当前提升器高度'] = plaintext.pop("Rx_valuePositionSensorFrmEHR")
            plaintext['当前提升器合力'] = plaintext.pop("Rx_valueSumofDraftFrmEHR")
            plaintext['车不按指令动作原因'] = plaintext.pop("Rx_staOfCaliStaFrmEHR1")
            if plaintext['车不按指令动作原因'] == 0:
                plaintext['车不按指令动作原因'] = '无故障'
            elif plaintext['车不按指令动作原因'] == 1:
                plaintext['车不按指令动作原因'] = '有故障'

            plaintext['车停止原因'] = plaintext.pop("Rx_staOfCaliStaFrmEHR2")
            if plaintext['车停止原因'] == 0:
                plaintext['车停止原因'] = '无障碍物'
            elif plaintext['车停止原因'] == 1:
                plaintext['车停止原因'] = '有障碍物'

        return plaintext
        # if ID == 0x18FEF200:    # 底盘控制指令
        #     plaintext = self.__can_msg_trans.db.decode_message(ID, data)
        #     return plaintext
        # if ID ==


class VCUCmd(object):
    """
        本类进行拖拉机整车（横纵向运动、提升器、PTO、液压输出）控制指令的封装，按特定功能来设计函数。
    """
    # 类成员变量
    __NAVI_cmd_list = [0.0, 0, 0, 2, 0, 0]
    __NAVI2_cmd_list = [3, 0, 250, 250, 250]
    __NAVI3_cmd_list = [0, 0, 0, 0, 0, 0, 0, 0]

    # 按实现不同功能设计类方法
    def __init__(self, tractor):
        """
            进行变量的初始化
        """
        self.tractor = tractor

    def send_motion_ctrl_msg(self, Drive_Mode_Req, ShiftLevel_Req, Steering_Wheel_Angle, Target_Speed, Brk_En):
        """
            发送车辆横纵向控制指令。后期把参数换成英文。
        :param Drive_Mode_Req:0:手动模式、1：扭矩受导航请求控制、2：自动模式
        :param ShiftLevel_Req:字符串类型，分为“前进高档”、“前进低档”、“后退高档”、“后退低档”、“空挡”
        :param Steering_Wheel_Angle:
        :param Target_Speed:
        :param Brk_En:
        :return:
        """
        # 首先检查驾驶模式
        if Drive_Mode_Req == 0:  # 手动模式
            self.__NAVI_cmd_list[3] = 0
            self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)
            # return "手动模式" # debug20211001注释掉本句，想在手动驾驶的时候进行方向盘的控制

        # TODO:检查下有车速情况下发空挡是什么反应
        # 刹车的优先级最高放在这里，刹车后就返回(刹车转换成速度为0、不管转角)
        if Brk_En == 1:
            self.__NAVI_cmd_list[0] = 0  # 修改车速(后期看是否要挂空挡)
            # 发送指令直接返回
            self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)
            return "程序刹车制动"

        # 修改变量序列
        self.__NAVI_cmd_list[0] = Target_Speed  # 修改车速

        if ShiftLevel_Req == "前进高档":  # 测试下是否高效
            self.__NAVI_cmd_list[1] = 1  # 修改档位(前进1、空挡0、后退1)
            # self.__NAVI2_cmd_list[2]
            self.__NAVI2_cmd_list[0] = 11  # 高低档（高档11，低档3）
        elif ShiftLevel_Req == "前进低档":
            self.__NAVI_cmd_list[1] = 1
            self.__NAVI2_cmd_list[0] = 3
        elif ShiftLevel_Req == "后退高档":
            self.__NAVI_cmd_list[1] = -1
            self.__NAVI2_cmd_list[0] = 11
        elif ShiftLevel_Req == "后退低档":
            self.__NAVI_cmd_list[1] = -1
            self.__NAVI2_cmd_list[0] = 3
        else:
            # “空挡”
            self.__NAVI_cmd_list[1] = 0

        # self.__NAVI_cmd_list[2]   # 修改提升器指令
        self.__NAVI_cmd_list[3] = Drive_Mode_Req  # 修改导航模式(0:手动模式、1：扭矩受导航请求控制、2：自动模式)
        # self.__NAVI_cmd_list[4]   # 修改点火熄火信号
        # self.__NAVI_cmd_list[5]   # 修改PTO指令

        # 前轮转角(后期读取前轮实际转角进行误差消除)
        self.__NAVI3_cmd_list[4] = Steering_Wheel_Angle  # 前轮目标转角

        # 发送修改的变量
        print('self.__NAVI_cmd_list', self.__NAVI_cmd_list)
        self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)

        self.tractor.send_msg("NAVI2", self.__NAVI2_cmd_list)
        self.tractor.send_msg("NAVI3", self.__NAVI3_cmd_list)
        return "程序控制指令发送成功"

    def send_hoist_msg(self, hoist_cmd="rising", hoist_ploughing_depth_set=1000, hoist_height_limit_set=220,
                       hoist_drop_speed_set=50, hoist_model_set=250):  # debug 修改最高高度250 到220
        """
            控制提升器的高度、升降等。参数列表：提升器指令、耕地深度设定旋钮、限高旋钮、下降速度设定、模式设定。 (反馈值：提升器位置（高度）、提升器合力)
        :param hoist_cmd:"rising"、“falling”、“no_action”
        :param hoist_ploughing_depth_set:
        :param hoist_height_limit_set:
        :param hoist_drop_speed_set:
        :param hoist_model_set:
        :return:
        """
        # 提升器指令
        if hoist_cmd == "rising":
            self.__NAVI_cmd_list[2] = 0xf0
        elif hoist_cmd == "falling":
            self.__NAVI_cmd_list[2] = 0x0f
        else:
            self.__NAVI_cmd_list[2] = 0  # 无动作

        self.__NAVI2_cmd_list[1] = hoist_ploughing_depth_set
        self.__NAVI2_cmd_list[2] = hoist_height_limit_set
        self.__NAVI2_cmd_list[3] = hoist_drop_speed_set
        self.__NAVI2_cmd_list[4] = hoist_model_set

        # 发送命令
        self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)
        self.tractor.send_msg("NAVI2", self.__NAVI2_cmd_list)
        # self.tractor.send_msg("NAVI3", self.__NAVI3_cmd_list)

    def steering_wheel_fault_removal(self):
        """发送方向盘故障清除指令，这里应该有一个持续时间的问题，如果测试不行则多发几次"""
        # 改好发送再改过来
        self.__NAVI3_cmd_list[5] = 1
        self.tractor.send_msg("NAVI3", self.__NAVI3_cmd_list)
        self.__NAVI3_cmd_list[5] = 0

    def send_pto_msg(self, pto_connected=0):
        """控制PTO"""
        self.__NAVI_cmd_list[5] = pto_connected  # 修改PTO指令
        self.tractor.send_msg("NAVI", self.__NAVI_cmd_list)

    def send_hydraulic_msg(self):
        """控制液压输出"""


class SafetyGuarantee(object):
    """提供安全保障功能"""

    # 读取刹车状态、方向盘故障信息，然后切换成手动驾驶。使用多线程的方式实现。后期完成
    def __init__(self, tractor_recv):
        self.__tractor_recv = tractor_recv
        # self.brake_flag = brake_flag     # 默认为0不刹车
        global brake_flag

    def monitor_brake(self):
        # 暂时先这样
        while 1:
            recv = self.__tractor_recv.recv_msg_full_dbc_id()
            if recv[0] == 0x18FFA227 and recv[1]["刹车状态"] == 1:
                # 有刹车
                print("在刹车")
                brake_flag = 1
                # 执行速度为0、抬机具，δ时间后切换成空挡，判断切换成空挡后换成手动模式
                return brake_flag


if __name__ == "__main__":
    NAVI_cmd_list = [3.0, 1, 0, 2, 0, 0]  # v=3,前进挡，测试横纵向连通性
    # NAVI_cmd_list = [0.0, 0, 0, 2, 0, 0]
    NAVI2_cmd_list = [3, 0, 250, 250, 250]
    NAVI3_cmd_list = [0, 0, 0, 0, 0, 0, 0, 0]

    # 20210705 更新:测试横向跟踪性能
    tractor = Tractor("send")
    time.sleep(3)
    tractor.send_msg("NAVI", NAVI_cmd_list)
    NAVI3_cmd_list[4] = -40
    tractor.send_msg("NAVI3", NAVI3_cmd_list)
    while True:
        time.sleep(1)
    # tractor.send_msg("NAVI3", NAVI3_cmd_list)
    # tractor = classOfDongFengSF2204.Tractor("send")
    # vcu_cmd = VCUCmd(tractor)

    # machine_operation_enabled=1
    # machine_operation_status="raise"
    # #machine_operation_status = "down"
    #
    # # 添加机具控制，及其使能
    # if machine_operation_enabled == 1:
    #     if machine_operation_status == "raise":
    #         # 机具提升命令发送
    #         # pto断开
    #         vcu_cmd.send_pto_msg(0)
    #         time.sleep(0.1)  # debug
    #         vcu_cmd.send_hoist_msg("rising", hoist_ploughing_depth_set=1000, hoist_height_limit_set=220)
    #         pass
    #     elif machine_operation_status == "down":
    #         # 机具下降命令发送
    #         vcu_cmd.send_hoist_msg("falling", hoist_ploughing_depth_set=760, hoist_height_limit_set=220)
    #         time.sleep(0.1)  # debug
    #         #vcu_cmd.send_pto_msg(1)
    #         pass

    # 单独测试转角
    # NAVI3_cmd_list[4] = 40
    # NAVI3_cmd_list[4] = -40
    # tractor.send_msg("NAVI3", NAVI3_cmd_list)
    # while True:
    #     time.sleep(1)
    Hz = 0.2
    list_for = [0, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40, -40, 40,
                -40, 40, -40, 40, -40, 40]
    for i in list_for:
        print("i=", i)
        print(time.time())
        # tractor.sendMsg(1, 1, float(i), 0.0, 0)
        NAVI3_cmd_list[4] = int(i)
        tractor.send_msg("NAVI3", NAVI3_cmd_list)
        time.sleep(1 / Hz)

    while True:
        time.sleep(1)
    # 响应频率
    Hz = 0.5
    angle_resolution = 40
    threshold_min = -40
    threshold_max = 40
    while True:
        for i in np.arange(0, threshold_max + 0.1, angle_resolution):
            print("i=", i)
            print(time.time())
            # tractor.sendMsg(1, 1, float(i), 0.0, 0)
            NAVI3_cmd_list[4] = int(i)
            tractor.send_msg("NAVI3", NAVI3_cmd_list)
            time.sleep(1 / Hz)

        j_list = []
        for j in np.arange(threshold_min, threshold_max + 0.1, angle_resolution):
            j_list.append(j)
            # j_list.remove(0)

        j_list.reverse()
        for j in j_list:
            print("j=", j)
            print(time.time())
            # tractor.sendMsg(1, 1, float(j), 0.0, 0)
            NAVI3_cmd_list[4] = int(j)
            print('j=', j)
            tractor.send_msg("NAVI3", NAVI3_cmd_list)
            time.sleep(1 / Hz)

        for k in np.arange(threshold_min, 0, angle_resolution):
            print("k=", k)
            # tractor.sendMsg(1, 1, float(k), 0.0, 0)
            NAVI3_cmd_list[4] = int(k)
            print("k=", k)
            print(time.time())
            tractor.send_msg("NAVI3", NAVI3_cmd_list)
            time.sleep(1 / Hz)

    #
    # can_msg_trans = CANMsgTrans("dongFengSF2204.dbc")
    # NAVI_msg = can_msg_trans.can_msg_produce("NAVI", NAVI_msg_list)
    # print(NAVI_msg)
    # can_bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
    # task_NAVI = can_bus.send_periodic(NAVI_msg, period=0.1, duration=None, store_task=True)  # 一个task值对应一个can.Message.arbitration_id
    # #task.modify_data(msg)
    # task_NAVI2 = can_bus.send_periodic(can_msg_trans.can_msg_produce("NAVI2", NAVI2_msg_list), period=0.1,
    #                                    duration=None, store_task=True)
    # task_NAVI3 = can_bus.send_periodic(can_msg_trans.can_msg_produce("NAVI3", NAVI3_msg_list), period=0.1,
    #                                    duration=None, store_task=True)
    # task_VC6 = can_bus.send_periodic(can_msg_trans.can_msg_produce("VC6", VC6_msg_list), period=0.1,
    #                                    duration=None, store_task=True)

    tractor = Tractor(0)
    time.sleep(5)
    # tractor.send_msg("NAVI", NAVI_msg)
    # for i in range(10):
    #     NAVI_msg_list[0] = i
    #     time.sleep(0.1)
    #     NAVI_msg = can_msg_trans.can_msg_produce("NAVI", NAVI_msg_list)
    #     tractor.send_msg("NAVI", NAVI_msg)

    while True:
        # time.sleep(1)
        for i in range(10):
            pass
            # NAVI_cmd_list[0] = i
            # NAVI2_cmd_list[0] = i
            # NAVI3_cmd_list[0] = i
            # VC6_cmd_list[1] = i
            #
            # time.sleep(0.1)
            # tractor.send_msg("NAVI", NAVI_cmd_list)
            # tractor.send_msg("NAVI2", NAVI2_cmd_list)
            # tractor.send_msg("NAVI3", NAVI3_cmd_list)
            # tractor.send_msg("VC6", VC6_cmd_list)

    # 最后想实现的功能，一个函数和参数，调用相关的指令。
    # tractor.send_msg("NAVI",[])
