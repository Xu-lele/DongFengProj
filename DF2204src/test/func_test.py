# 进行信息解析的测试

import DF2204src.component.classOfDF2204 as classOfDF2204

tractor_recv = classOfDF2204.Tractor("recv")
while True:
    print(tractor_recv.recv_msg_full_dbc_id())
    #time.sleep(0.1)
