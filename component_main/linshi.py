# @Time    : 2021/10/20 下午2:33
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : linshi.py
# @Software: PyCharm
#
import time

import classOfDF2204

tractor = classOfDF2204.Tractor("recv")
can_msg = tractor.recv_msg_full()
with open(str(time.time())+"can记录.txt","w") as file_open:

    while True:
        can_msg = tractor.recv_msg_full()
        file_open.write(str(can_msg))
        file_open.flush()
        print("记录中"+str(time.time()))

    file_open.close()




