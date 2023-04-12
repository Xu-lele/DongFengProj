# 进行刹车功能的演示
import time
import component_0.classOfDF2204 as classOfDF2204

tractor_recv = classOfDF2204.Tractor("recv")
safety_node = classOfDF2204.SafetyGuarantee(tractor_recv)

safety_node.monitor_brake()
