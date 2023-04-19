# 拖拉机信息更新类，无延迟读取拖拉机信息，并提供函数返回所需的状态
import threading
import time
import classOfDF2204


class TractorInfoRead(object):
    def __init__(self):
        self.tractor_recv = classOfDF2204.Tractor("recv")
        # 尝试使用字典
        self.tractor_info = {"档位": "initializing", "车速": "initializing", "提升器高度": "initializing",
                             "提升器合力": "initializing", "车不动原因": "initializing",
                             "前轮转角": "initializing",
                             "刹车状态": "initializing", "发动机扭矩百分比": "initializing",
                             "方向盘故障": "initializing",
                             "方向盘位置传感器值": "initializing"}  # 将10个车辆自身状态爱参数保存为一个dict
        # 读取并反馈出车辆的所有dbc状态参数，方便使用和存储在行驶记录表里面
        # 开启一个线程不断去读取信息，并修改成员变量

        t = threading.Thread(target=self.tractor_info_read_cycle)
        t.start()
        # t.join()

    def vehicle_state_info_return(self):
        # 返回车辆状态字典
        return self.tractor_info

    def tractor_info_read_cycle(self):
        while True:
            msg_list = self.tractor_recv.recv_msg_full_dbc_id()
            if msg_list[0] == 0x18FFA127:
                # print(type(msg_list[1]))
                # 更新成员变量
                self.tractor_info["档位"] = msg_list[1]["档位"]
                self.tractor_info["车速"] = msg_list[1]["车速"]
                self.tractor_info["提升器高度"] = msg_list[1]["提升器高度"]
                self.tractor_info["提升器合力"] = msg_list[1]["提升器合力"]
                self.tractor_info["车不动原因"] = msg_list[1]["车不动原因"]
                self.tractor_info["前轮转角"] = msg_list[1]["前轮转角"]
                # self.tractor_info["gear"] = msg_list[1]
                # print(self.tractor_info)

                # print("收到运动反馈")
                # ui.label_motion.setText(str(msg_list[1]))
                # ui.label_motion.repaint()
            else:
                print(msg_list[1])
                self.tractor_info["刹车状态"] = msg_list[1]["刹车状态"]
                self.tractor_info["发动机扭矩百分比"] = msg_list[1]["发动机扭矩百分比"]
                self.tractor_info["方向盘故障"] = msg_list[1]["方向盘故障"]
                self.tractor_info["方向盘位置传感器值"] = msg_list[1]["方向盘位置传感器值"]
                # print(self.tractor_info)
                # ui.label_security.setText(str(msg_list[1]))
                # ui.label_security.repaint()


if __name__ == '__main__':
    # 先测试线程是否正常开启
    tractor_info = TractorInfoRead()
    while True:
        a = tractor_info.vehicle_state_info_return()
        time.sleep(0.5)
        print(a)
