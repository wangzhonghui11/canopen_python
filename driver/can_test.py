import time
import canopen
from threading import Lock
import signal
import sys


class CANopenMonitor:
    def __init__(self, node_id=1, eds_path='', channel='can0', bitrate=1000000):
        self.node_id = node_id
        self.network = canopen.Network()
        self.print_lock = Lock()

        # 添加节点
        self.slave_node = canopen.RemoteNode(node_id, eds_path)
        self.network.add_node(self.slave_node)

        # 连接CAN总线
        self.network.connect(bustype='socketcan', channel=channel, bitrate=bitrate)

        # 设置回调
        # self.network.subscribe(self._receive_callback)

        # 启动节点
        self.network.send_message(0x000, [0x01, node_id])
        self.network.send_message(0x000, [0x80, 1])
        # 信号处理
        self.is_running = True
        signal.signal(signal.SIGINT, self._signal_handler)

    def _print_hex(self, is_send, can_id, data):
        with self.print_lock:
            direction = "send" if is_send else "receive"
            hex_data = ' '.join(f"{byte:02x}" for byte in data)
            print(f"[0x{can_id:x}] {direction}: {hex_data}")

    def _receive_callback(self, message):
        self._print_hex(False, message.arbitration_id, message.data)

    def send_data(self, can_id, data):
        self.network.send_message(can_id, data)
        self._print_hex(True, can_id, data)

    def read_sdo_subindex(self, index, subindex=0):
        try:
            return self.slave_node.sdo[index][subindex].raw
        except Exception as e:
            print(f"SDO read error: {e}")
            return None
    def read_sdo(self, index):
        try:
            return self.slave_node.sdo[index].raw
        except Exception as e:
            print(f"SDO read error: {e}")
            return None
    def write_sdo_subindex(self, index, subindex, value):
        try:
            self.slave_node.sdo[index][subindex].raw = value
            return True
        except Exception as e:
            print(f"SDO write error: {e}")
            return False
    def write_sdo(self, index, value):
        try:
            self.slave_node.sdo[index].raw = value
            return True
        except Exception as e:
            print(f"SDO write error: {e}")
            return False
    def _signal_handler(self, sig, frame):
        print("\nShutting down...")
        self.is_running = False
        self.network.disconnect()
        sys.exit(0)

    def run_test(self):
        test_data = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
        while self.is_running:
            # # 发送测试数据
            # self.send_data(0x701, test_data)
            #
            # # 读取SDO数据示例
            # value = self.read_sdo(0x2002)
            # if value is not None:
            #     print(f"SDO 0x2002 value: {value}")

            time.sleep(0.01)


if __name__ == "__main__":
    # 使用示例
    monitor = CANopenMonitor(node_id=1, eds_path='../eds/YiyouServo.eds')

    # 运行测试循环
    monitor.run_test()

    # 或者手动操作示例:
    # monitor.write_sdo(0x2000, 0, 123)  # 写入数据
    # print(monitor.read_sdo(0x2000, 0)) # 读取数据
    # monitor.send_data(0x123, [0x11, 0x22]) # 发送原始CAN帧