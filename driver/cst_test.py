import time
import canopen
from threading import Lock, Thread
import signal
import sys


class HarmonicServoDriver:
    def __init__(self, node_id=1, channel='can0', bitrate=1000000,eds_path='YiyouServo.eds'):
        self.node_id = node_id
        self.network = canopen.Network()
        self.print_lock = Lock()
        self.is_running = True
        self.current_torque = 0
        self.target_torque = 0
        self.is_sync = True
        self.itpv = 4  # 插补周期，单位ms

        # 信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        try:
            # 连接CAN总线
            self.network.connect(
                bustype='socketcan',
                channel=channel,
                bitrate=bitrate
            )

            # 添加节点
            self.node = canopen.RemoteNode(node_id, eds_path)
            self.network.add_node(self.node)

            print(f"Harmonic伺服驱动器初始化成功，节点ID: {node_id}")

        except Exception as e:
            print(f"初始化失败: {e}")
            self.cleanup()
            raise

    def signal_handler(self, sig, frame):
        """处理中断信号"""
        print("\n接收到中断信号，正在安全关闭...")
        self.is_running = False
        self.cleanup()
        sys.exit(0)

    def print_message(self, is_tx, can_id, data):
        """打印CAN消息"""
        with self.print_lock:
            direction = "TX" if is_tx else "RX"
            hex_data = ' '.join(f"{byte:02x}" for byte in data)
            print(f"[{direction} 0x{can_id:03X}] {hex_data}")

    def configure_cyclic_sync_torque_mode(self):
        """配置循环同步力矩模式"""
        try:
            print("配置循环同步力矩模式...")

            # 1. 设置操作模式为循环同步力矩模式 (0x6060 = 10)
            self.node.sdo[0x6060].raw = 10  # 1 = Profile Position Mode
            print("  操作模式设置为循环同步力矩模式 (0x6060 = 10)")

            # 2. 设置插补时间周期 (0x60C2sub1)
            self.node.sdo[0x60C2][1].raw = self.itpv
            print(f"  插补时间周期设置为 {self.itpv}ms (0x60C2sub1)")
            self.node.sdo[0x1019].raw = 0
            # 3. 配置RPDO1映射 (目标力矩)
            self.node.sdo[0x1400][1].raw = (0x80 << 24) + 0x200 +  + self.node_id  # COB-ID (带禁止位)
            self.node.sdo[0x1600][0].raw = 0  # 映射条目数
            self.node.sdo[0x1400][2].raw = 0xFF if not self.is_sync else 0x01  # 传输类型

            self.node.sdo[0x1600][1].raw =  (0x6071 << 16) + 0x10 # 目标力矩(0x6071:00, 16bit)
            self.node.sdo[0x1600][0].raw = 1  # 映射条目数
            # 4. 重置节点应用配置
            self.node.nmt.state = 'RESET'
            print("  节点重置")
            time.sleep(0.05)

            self.node.nmt.state = 'OPERATIONAL'
            print("  节点启动")
            time.sleep(0.05)

            # 5. 重新配置RPDO1 COB-ID（移除禁止位）
            self.node.sdo[0x1400][1].raw = 0x200 + self.node_id  # COB-ID
            print("  RPDO1 COB-ID重新设置为 0x%03X" % (0x200 + self.node_id))

            # # 6. 状态转换序列
            control_words = [
                (0x06, "准备启用"),
                (0x07, "启用"),
                (0x0F, "开始运动")
            ]

            for value, description in control_words:
                self.node.sdo[0x6040].raw = value
                print(f"  控制字: 0x{value:04X} - {description}")
                time.sleep(0.05)

            # 7. 获取当前力矩
            self.current_torque = self.node.sdo[0x6077].raw
            print(f"  当前力矩: {self.current_torque}")

            print("循环同步力矩模式配置完成")
            return True

        except Exception as e:
            print(f"配置失败: {e}")
            return False

    def set_torque(self, torque):
        """设置目标力矩"""
        try:
            # 使用RPDO发送目标力矩
            data = torque.to_bytes(2, byteorder='little', signed=True)
            self.network.send_message(0x200 + self.node_id, data)
            self.print_message(True, 0x200 + self.node_id, data)

            # 如果需要同步，发送同步帧
            if self.is_sync:
                self.network.send_message(0x80, bytes([0]))
                self.print_message(True, 0x80, bytes([0]))

            return True
        except Exception as e:
            print(f"设置力矩失败: {e}")
            return False

    def run_torque_control(self, start_torque=0, step=1, max_torque=1000):
        """运行力矩控制循环"""
        if not self.configure_cyclic_sync_torque_mode():
            return

        try:
            torque = start_torque
            while self.is_running:
                if not self.set_torque(torque):
                    break

                print(f"目标力矩: {torque}")

                # 更新力矩值
                torque += step
                if torque > max_torque:
                    torque = max_torque

                # 按插补周期等待
                time.sleep(self.itpv / 1000)

        finally:
            self.stop_control()

    def stop_control(self):
        """停止控制"""
        try:
            self.node.sdo[0x6040].raw = 0x06  # 回到准备启用状态
            print("控制已停止")
        except Exception as e:
            print(f"停止控制失败: {e}")

    def cleanup(self):
        """清理资源"""
        try:
            if hasattr(self, 'node'):
                self.stop_control()
            if hasattr(self, 'network'):
                self.network.disconnect()
            print("资源已安全释放")
        except Exception as e:
            print(f"清理过程中出错: {e}")


# 使用示例
if __name__ == "__main__":
    try:
        # 创建伺服驱动器实例
        servo = HarmonicServoDriver(
            node_id=1,
            channel='can0',
            bitrate=1000000,
            eds_path='../eds/YiyouServo.eds'
        )

        # 运行力矩控制
        servo.run_torque_control(
            start_torque=0,
            step=1,
            max_torque=100
        )

    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if 'servo' in locals():
            servo.cleanup()