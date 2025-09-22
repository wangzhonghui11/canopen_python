import time
import canopen
from threading import Lock
import signal
import sys


class HarmonicDriver:
    def __init__(self, node_id=1, channel='can0', bitrate=1000000,eds_path=''):
        self.node_id = node_id
        self.network = canopen.Network()
        self.print_lock = Lock()
        self.is_running = True
        self.pos = 0
        self.step = 10000
        self.itpv = 10  # 插补周期(ms)
        self.is_sync = False

        # 信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        try:
            self.node = canopen.RemoteNode(node_id, eds_path)  # 无EDS文件
            self.network.add_node(self.node)
            # 连接CAN总线
            self.network.connect(bustype='socketcan', channel=channel, bitrate=bitrate)

            # 添加节点


            print(f"设备初始化完成，节点ID: {node_id}")

        except Exception as e:
            print(f"初始化失败: {e}")
            self.cleanup()
            raise

    def signal_handler(self, sig, frame):
        """处理中断信号"""
        print("\n接收到中断信号，正在关闭...")
        self.is_running = False
        self.cleanup()
        sys.exit(0)

    def print_hex(self, is_send, can_id, data):
        """打印CAN消息"""
        with self.print_lock:
            direction = "TX" if is_send else "RX"
            hex_data = ' '.join(f"{byte:02x}" for byte in data)
            print(f"[{direction} 0x{can_id:03X}] {hex_data}")

    def set_position(self, pos, is_sync=False):
        """设置目标位置"""
        data = pos.to_bytes(4, byteorder='little', signed=True)
        self.network.send_message(0x200 + self.node_id, data)
        self.print_hex(True, 0x200 + self.node_id, data)

        if is_sync:
            self.network.send_message(0x80, bytes([0]))  # 同步帧

    def configure_driver(self):
        """配置驱动器参数"""
        try:
            # 设置操作模式为内插位置模式
            self.node.sdo[0x6060].raw = 7  # 内插位置模式

            # 设置插补周期
            self.node.sdo[0x60C2][1].raw = self.itpv
            self.node.sdo[0x1019].raw = 0
            # 3. 配置RPDO1映射 (控制字 + 目标位置)
            # 配置RPDO
            self.node.sdo[0x1400][1].raw =(0x80 << 24) + 0x200 + self.node_id  # COB-ID
            self.node.sdo[0x1600][0].raw = 0  # 映射条目数
            self.node.sdo[0x1400][2].raw = 0xFF if not self.is_sync else 0x01  # 传输类型

            # 映射RPDO
            self.node.sdo[0x1600][1].raw = (0x60C1 << 16) + 0x0120 # 控制字(0x6040, 16bit)
            self.node.sdo[0x1600][0].raw = 1  # 映射条目数
            # 重置节点
            self.network.send_message(0x000, [0x81, self.node_id])
            time.sleep(0.05)

            # 启动节点
            self.network.send_message(0x000, [0x01, self.node_id])
            time.sleep(0.05)
            self.node.sdo[0x1400][1].raw = 0x200 + self.node_id  # COB-ID
            # 设置控制字
            self._set_control_word(0x06)  # 准备启用
            self._set_control_word(0x07)  # 启用
            self._set_control_word(0x0F)  # 开始运动
            self._set_control_word(0x1F)  # 开始内插运动

            # 获取当前位置
            self.pos = self.node.sdo[0x6064].raw
            return True

        except Exception as e:
            print(f"配置失败: {e}")
            return False

    def _set_control_word(self, value):
        """设置控制字"""
        self.node.sdo[0x6040].raw = value
        time.sleep(0.01)

    def run(self):
        """运行位置控制循环"""
        if not self.configure_driver():
            return

        while self.is_running:
            self.set_position(self.pos, self.is_sync)
            print(f"当前位置: {self.pos}")

            # 计算新位置
            new_pos = self.pos + self.step

            # 边界检查
            if new_pos > 500000:
                self.pos = 500000
                self.step = -abs(self.step)
            elif new_pos < -500000:
                self.pos = -500000
                self.step = abs(self.step)
            else:
                self.pos = new_pos

            time.sleep(self.itpv / 1000)  # 按插补周期等待

    def cleanup(self):
        """清理资源"""
        try:
            if hasattr(self, 'node'):
                self._set_control_word(0x06)  # 停止运动
            if hasattr(self, 'network'):
                self.network.disconnect()
            print("资源已释放")
        except Exception as e:
            print(f"清理失败: {e}")


if __name__ == "__main__":
    try:
        # 配置参数
        driver = HarmonicDriver(
            node_id=1,
            channel='can0',
            bitrate=1000000,
            eds_path='../eds/YiyouServo.eds'
        )

        # 运行控制循环
        driver.run()

    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if 'driver' in locals():
            driver.cleanup()