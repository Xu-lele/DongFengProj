# @Time    : 2021/9/10 下午5:05
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : ui_module_main_path_following_1_1.py
# @Software: PyCharm
from PyQt5.QtWidgets import QApplication


def pushButton_ori_start_clicked(ui, is_operation_flag):
    # 设置初始值
    is_operation_flag[4] = 0
    is_operation_flag[5] = 0
    ui.textEdit_0.setText("成功加载任务" + str(is_operation_flag[4]) + "," + "目标点" + str(is_operation_flag[5]) + "\n")


def pushButton_load_clicked(ui, is_operation_flag):
    # 加载左侧任务点击
    # 加载文件中的最后一行
    interrupt_record_doc = "component_ui/interrupt_record_doc/"
    doc_name = "中断记录.txt"
    with open(interrupt_record_doc + doc_name, "r") as file_open:
        # 加载文件中最后一行
        file_list = file_open.readlines()
        end_line = file_list[-1].split(',')
        task_ind = int(end_line[0])
        path_target_point_ind = int(end_line[1])
        is_operation_flag[4] = task_ind
        is_operation_flag[5] = path_target_point_ind
        file_open.close()
    ui.textEdit_0.setText("成功加载任务" + str(is_operation_flag[4]) + "," + "目标点" + str(is_operation_flag[5]) + "\n")

# 先简单设计，可扩展即可
def pushButton_start_clicked(ui, is_operation_flag):
    # “开始任务”被点击，标志位置1
    is_operation_flag[0] = 1
    # ui.pushButton_start.setText("进行任务中...")
    ui.pushButton_start.repaint()
    print("is_operation_flag[0]=", is_operation_flag[0])
    ui.textEdit_0.setText("开始执行任务" + str(is_operation_flag[4]) + "," + "目标点" + str(is_operation_flag[5]) + "\n")


def pushButton_interrupt_clicked(ui, is_operation_flag):
    is_operation_flag[0] = 0
    # 开启一个线程进行标志位监测
    # ui.pushButton_interrupt.setText("程序已中断")
    ui.pushButton_interrupt.repaint()
    print("is_operation_flag[0]=", is_operation_flag[0])

    # 点击中断记录下当前作业任务文件名+作业点序号，追加写在一个文件里。
    interrupt_record_doc = "component_ui/interrupt_record_doc/"
    doc_name = "中断记录.txt"
    with open(interrupt_record_doc+doc_name, "a+") as file_open:
        # 按钮点一次就记录一下
        file_open.write(str(is_operation_flag[2])+","+str(is_operation_flag[3])+"\n")
        file_open.close()

    ui.textEdit_0.setText("任务中断在"+str(is_operation_flag[2])+",目标点"+str(is_operation_flag[3])+"\n")



if __name__ == '__main__':
    pass
