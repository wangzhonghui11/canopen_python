import time
import canopen
import threading
import signal
import sys
from dataclasses import dataclass


@dataclass
class MotionProfile:
    velocity: int = 1000000
    acceleration: int = 1000000
    deceleration: int = 1000000
    target_pos: int = 500000
    is_relative: bool = False
    is_immediate: bool = True
    is_update: bool = False


class HarmonicPositionController:
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

    def setup_position_mode(self, profile: MotionProfile):
        """配置轮廓位置模式"""
        try:
            # 设置操作模式为轮廓位置模式 (通常对应对象字典0x6060)
            self.node.sdo[0x6060].raw = 1  # 1 = Profile Position Mode

            # 设置同步计数器 (对象字典0x1019)
            self.node.sdo[0x1019].raw = 0  # 直接赋值，无需子索引

            # 设置轮廓速度 (对象字典0x6081)
            self.node.sdo[0x6081].raw = profile.velocity

            # 设置加速度 (对象字典0x6083)
            self.node.sdo[0x6083].raw = profile.acceleration

            # 设置减速度 (对象字典0x6084)
            self.node.sdo[0x6084].raw = profile.deceleration

            # 状态转换序列 (通过控制字0x6040)
            self.node.sdo[0x6040].raw = 0x06  # 准备启动
            self.node.sdo[0x6040].raw = 0x07  # 切换到"Operation Enabled"

            return True
        except Exception as e:
            print(f"[error] Setup failed: {e}")
            return False

    def move_to_position(self, profile: MotionProfile):
        """执行位置移动"""
        control_word = 0x0F
        if profile.is_relative:
            control_word |= 0x40
        if profile.is_immediate:
            control_word |= 0x20

        try:
            # 设置控制字
            self.node.sdo[0x6040].raw = control_word

            # 设置目标位置 (对象字典0x607A)
            self.node.sdo[0x607A].raw = profile.target_pos

            # 触发移动 (设置bit4)
            self.node.sdo[0x6040].raw = control_word | 0x10

            # 打印状态
            with self.print_lock:
                print(f"[test]moving to {profile.target_pos} \tvel:{profile.velocity} "
                      f"\tacc:{profile.acceleration} \tdec:{profile.deceleration}")

            return True
        except Exception as e:
            print(f"[error] Movement failed: {e}")
            return False

    def stop_control(self):
        """停止控制"""
        try:
            self.node.sdo[0x6040].raw = 0x06  # 切换到"Ready to switch on"
            self.node.sdo[0x6040].raw = 0x00  # 禁用
        except Exception as e:
            print(f"[error] Stop failed: {e}")

    def run_position_test(self, profile: MotionProfile):
        """运行位置控制测试"""
        if not self.setup_position_mode(profile):
            return False

        while self.running:
            if not self.move_to_position(profile):
                break

            # 等待移动完成 (简化版，实际应该检查状态字)
            for _ in range(10):
                if not self.running:
                    break
                time.sleep(0.1)

            # 反转目标位置进行往返运动
            profile.target_pos = -profile.target_pos

        self.stop_control()


if __name__ == "__main__":
    # 创建运动配置文件
    motion_profile = MotionProfile(
        velocity=4000000,
        acceleration=4000000,
        deceleration=4000000,
        target_pos=500000
    )

    # 初始化控制器
    controller = HarmonicPositionController(node_id=1, eds_path='../eds/YiyouServo.eds')

    try:
        # 运行测试
        controller.run_position_test(motion_profile)
    except KeyboardInterrupt:
        controller.stop_control()
        controller.network.disconnect()