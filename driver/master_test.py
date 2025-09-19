import time
import canopen

# 创建一个网络用来表示CAN总线
network = canopen.Network()

# 添加slave节点，其id是2(需要与slaver.py里一致)，对象字典为Slaver.eds
slaver_node = canopen.RemoteNode(1, '../eds/YiyouServo.eds')
network.add_node(slaver_node)

# 连接到CAN总线 CAN波特率1000K
network.connect(bustype='socketcan', channel='can0', bitrate=1000000)

# 全局广播，id为0x00,内容为0x01,0x00 让节点都进入操作状态
network.send_message(0x000, [0x01, 1])
while 1:
# 使用快速sdo读取节点0x2000地址数据，适用于没有定义变量的区域
    test = slaver_node.sdo[0x2002].raw  # 使用快速sdo读取节点0x2000地址数据
    print("test value=" + str(test))
    time.sleep(1)
"""
while 1:
    temp = slaver_node.sdo['test'].raw  # 使用快速sdo读取节点test地址数据
    print("temp value=" + str(temp))

    slaver_node.sdo['test'].raw = temp + 1  # 使用快速sdo设置节点test地址数据

    time.sleep(1)
"""
