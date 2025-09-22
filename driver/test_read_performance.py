import time
import canopen
from threading import Lock
import signal
import sys


class CANopenMonitor:
    def __init__(self, node_id=1, eds_path='', channel='can0', bitrate=1000000):
        self.node_id = node_id
        self.network = None
        self.bus = None
        self.print_lock = Lock()
        self.is_running = True

        # 初始化信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        # 初始化CANopen网络
        self.initialize_network(eds_path, channel, bitrate)

    def initialize_network(self, eds_path, channel, bitrate):
        """初始化CANopen网络连接"""
        try:
            self.network = canopen.Network()
            self.print_lock = Lock()

            # 添加节点
            self.slave_node = canopen.RemoteNode(self.node_id, eds_path)
            self.network.add_node(self.slave_node)

            # 连接CAN总线
            self.network.connect(bustype='socketcan', channel=channel, bitrate=bitrate)

        except Exception as e:
            print(f"网络初始化失败: {e}")
            self.cleanup()
            raise

    def signal_handler(self, sig, frame):
        """处理中断信号"""
        print("\n接收到中断信号，正在关闭...")
        self.is_running = False
        self.cleanup()
        sys.exit(0)

    def print_message(self, is_tx, can_id, data):
        """格式化打印CAN消息"""
        with self.print_lock:
            direction = "TX" if is_tx else "RX"
            hex_data = ' '.join(f"{byte:02x}" for byte in data)
            print(f"[{direction} 0x{can_id:03X}] {hex_data}")

    def get_node_state(self):
        """获取节点状态"""
        try:
            # 读取对象字典中的状态信息
            state = self.network[self.node_id].sdo[0x6064].raw
            return state
        except Exception as e:
            print(f"获取节点状态失败: {e}")
            return None

    def run_performance_test(self, test_cycles=10000):
        """运行性能测试"""
        print("开始性能测试...")
        start_time = time.time()

        for cycle in range(test_cycles):
            if not self.is_running:
                break

            state = self.get_node_state()
            if state is None:
                break

            # 模拟每100次打印一次进度
            if cycle % 100 == 0:
                print(f"测试进度: {cycle}/{test_cycles}", end='\r')

        end_time = time.time()
        self.print_test_results(start_time, end_time, test_cycles)

    def print_test_results(self, start, end, cycles):
        """打印测试结果"""
        elapsed_ms = (end - start) * 1000
        frequency = cycles / (elapsed_ms / 1000)

        print("\n测试结果:")
        print(f"总循环次数: {cycles}")
        print(f"耗时: {elapsed_ms:.2f}ms")
        print(f"频率: {frequency:.2f}Hz")
        print(f"平均周期: {1000 / frequency:.2f}ms")

    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'network') and self.network:
            self.network.disconnect()
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
        print("资源已释放")


if __name__ == "__main__":
    try:
        # 配置参数
        config = {
            'node_id': 1,
            'eds_path': '../eds/YiyouServo.eds',  # 可选
            'channel': 'can0',
            'bitrate': 1000000
        }

        monitor = CANopenMonitor(**config)
        monitor.run_performance_test()

    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if 'monitor' in locals():
            monitor.cleanup()