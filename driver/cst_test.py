import time
import canopen
import threading
import signal
import sys
from dataclasses import dataclass


@dataclass
class TorqueProfile:
    target_torque: int = 300  # 目标力矩值
    slope: int = 300  # 力矩斜率
    is_update: bool = False  # 是否采用更新模式


class HarmonicTorqueController:
    def __init__(self, node_id=1, eds_path='YiyouServo.eds', channel='can0', bitrate=1000000):
        self.node_id = node_id
        self.network = canopen.Network()
        self.running = True
        self.print_lock = threading.Lock()

        # 添加节点
        self.node = canopen.RemoteNode(node_id, eds_path)
        self.network.add_node(self.node)

        # 连接CAN总线
        self.network.connect(bustype='socketcan', channel=channel, bitrate=bitrate)

        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("\nShutting down...")
        self.running = False
        self.network.disconnect()
        sys.exit(0)

    def setup_torque_mode(self, profile: TorqueProfile):
        """配置轮廓力矩模式"""
        try:
            # 设置操作模式为轮廓力矩模式 (对象字典0x6060)
            self.node.sdo[0x6060].raw = 4  # 4 = Profile Torque Mode

            # 设置同步计数器 (对象字典0x1019)
            self.node.sdo[0x1019].raw = 0  # 单变量访问方式

            # 设置力矩斜率 (对象字典0x6087)
            self.node.sdo[0x6087].raw = profile.slope

            # 状态转换序列 (通过控制字0x6040)
            self.node.sdo[0x6040].raw = 0x06  # 准备启动
            self.node.sdo[0x6040].raw = 0x07  # 切换到"Operation Enabled"

            return True
        except Exception as e:
            print(f"[error] Setup failed: {e}")
            return False

    def set_torque(self, torque_value):
        """设置目标力矩"""
        try:
            # 设置目标力矩 (对象字典0x6071)
            self.node.sdo[0x6071].raw = torque_value

            # 激活力矩控制 (控制字0x6040)
            self.node.sdo[0x6040].raw = 0x0F

            with self.print_lock:
                print(f"[test] Torque set to {torque_value}")

            return True
        except Exception as e:
            print(f"[error] Torque set failed: {e}")
            return False

    def stop_control(self):
        """停止控制"""
        try:
            self.node.sdo[0x6040].raw = 0x06  # 切换到"Ready to switch on"
            self.node.sdo[0x6040].raw = 0x00  # 禁用
        except Exception as e:
            print(f"[error] Stop failed: {e}")

    def run_torque_test(self, profile: TorqueProfile):
        """运行力矩控制测试"""
        if not self.setup_torque_mode(profile):
            return False

        toggle_torque = False
        while self.running:
            # 在300和0之间切换力矩值
            target = 300 if not toggle_torque else 0
            if not self.set_torque(target):
                break

            toggle_torque = not toggle_torque

            # 等待周期（简化版，实际应该检查状态字）
            for _ in range(100):
                if not self.running:
                    break
                time.sleep(0.05)

        self.stop_control()


if __name__ == "__main__":
    # 创建力矩配置文件
    torque_profile = TorqueProfile(
        target_torque=300,
        slope=300
    )

    # 初始化控制器
    controller = HarmonicTorqueController(node_id=1, eds_path='../eds/YiyouServo.eds')

    try:
        # 运行测试
        controller.run_torque_test(torque_profile)
    except KeyboardInterrupt:
        controller.stop_control()
        controller.network.disconnect()