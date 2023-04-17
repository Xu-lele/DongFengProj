# 进行车辆所有信息的界面展示
import sys, os
import time

from PyQt5.QtWidgets import QApplication, QMainWindow
from functools import partial
from PyQt5 import QtCore
import component_ui.status_info_display as status_info_display
import component_main.classOfDF2204 as classOfDF2204
import threading
import component_main.classOfTractorStatusDataRecord as classOfTractorStatusDataRecord
import ctypes   # 使用ctypes结束线程需要设计线程是可以直接杀死的，而没有临界资源的占用
import inspect



def check_mkdir(path_prefix):
    """
        检查文件夹是否存在，若不存在，则创建
    :param path_prefix:路径前缀，string型。eg:"pathRecord/ctrlCAN/"
    :return:
    """

    is_exists = os.path.exists(path_prefix + time.strftime("%Y-%m-%d", time.localtime()))
    if not is_exists:
        os.makedirs(path_prefix + time.strftime("%Y-%m-%d", time.localtime()))
    else:
        print("path 存在")


def button_serial_clicked(ui):
    if os.system("component_main/chmod_batch_file/sudo_chmod_777") == 0:
        ui.pushButton_serial.setText("serial初始化成功")
    else:
        ui.pushButton_serial.setText("serial初始化失败")


def button_pcan_clicked(ui):
    if os.system("component_main/chmod_batch_file/modprobe_peak_usb_forDongFeng2204") == 0:
        ui.pushButton_pcan.setText("pcan初始化成功")
    else:
        ui.pushButton_pcan.setText("pcan初始化失败")


def button_vcan_clicked(ui):
    if os.system("component_main/chmod_batch_file/modprobe_vcan") == 0:
        ui.pushButton_vcan.setText("vcan初始化成功")
    else:
        ui.pushButton_vcan.setText("vcan初始化失败")


def vehicle_info_display(ui):
    # 读取车辆信息，并显示,还有记录下来
    tractor_recv = classOfDF2204.Tractor("recv")
    # 判断文件夹是否存在
    path_prefix = 'tractorStatusInfoRecord/motion_security_info/'
    check_mkdir(path_prefix)
    path_name = path_prefix + time.strftime("%Y-%m-%d", time.localtime()) + '/'
    # 数据存储
    data_record = classOfTractorStatusDataRecord.TractorStatusDataRecord(path_name + "行驶反馈数据" +time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) +".csv")
    while True:
        # print()
        msg_list = tractor_recv.recv_msg_full_dbc_id()
        data_record.data_record([time.time()]+msg_list)
        if msg_list[0] == 0x18FFA127:
            # 记录，格式：时间戳，帧ID，帧内容

            # 运动反馈
            print("收到运动反馈")
            ui.label_motion.setText(str(msg_list[1]))
            # ui.label_motion.repaint()
        else:
            ui.label_security.setText(str(msg_list[1]))
            # ui.label_security.repaint()


def button_status_clicked(ui):
    ui.pushButton_status.setText("正在显示")
    ui.pushButton_status.setEnabled(False)
    ui.pushButton_status.repaint()
    t = threading.Thread(target=vehicle_info_display, args=(ui, ))
    t.start()
    time.sleep(10)


def ui_display():
    app = QApplication(sys.argv)  # zz开启一个windows主线程？
    MainWindow = QMainWindow()
    ui = status_info_display.Ui_MainWindow()  # 加载自定义的UI
    ui.setupUi(MainWindow)  # 设置为主窗口？
    MainWindow.show()
    # ui.pushButton_serial.clicked.connect(click_success)
    # ui.pushButton_pcan.clicked.connect(partial(convert, ui))
    thread_list = []
    ui.pushButton_status.clicked.connect(partial(button_status_clicked, ui))
    ui.pushButton_vcan.clicked.connect(partial(button_vcan_clicked, ui))
    ui.pushButton_pcan.clicked.connect(partial(button_pcan_clicked, ui))
    ui.pushButton_serial.clicked.connect(partial(button_serial_clicked, ui))

    print(app)
    # ui.webView_map.load(QtCore.QUrl("http://baidu.com"))


    # ui.label_motion.setText('测试')
    a = app.exec_()

    # print("a=", a)
    # if a == 0:
    #     # 结束所有线程
    #     print(stop_thread(thread_list[0]))
    sys.exit(a)
    # print("测试是否执行")


# def _async_raise(tid, exctype):
#     """raises the exception, performs cleanup if needed"""
#     tid = ctypes.c_long(tid)
#     if not inspect.isclass(exctype):
#         exctype = type(exctype)
#     res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
#     if res == 0:
#         raise ValueError("invalid thread id")
#     elif res != 1:
#         # """if it returns a number greater than one, you're in trouble,
#         # and you should call it again with exc=NULL to revert the effect"""
#         ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
#         raise SystemError("PyThreadState_SetAsyncExc failed")
#
#
# def stop_thread(thread):
#     _async_raise(thread.ident, SystemExit)
#     return "success"


if __name__ == "__main__":
    ui_display()    # 将界面作为主进程？
    print("是否运行")
