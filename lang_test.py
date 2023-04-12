# 测试提升器命令发送
import time

import component_0.classOfDF2204 as classOfDF2204

tractor = classOfDF2204.Tractor("send")
vcu_cmd = classOfDF2204.VCUCmd(tractor)

vcu_cmd.send_hoist_msg()
i=0
while True:
    time.sleep(1)
    vcu_cmd.send_hoist_msg(hoist_cmd='falling',hoist_ploughing_depth_set=i)
    i=i+1


