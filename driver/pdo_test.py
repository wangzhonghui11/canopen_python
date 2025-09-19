import time
import canopen
import threading
import signal
import sys
import math
from dataclasses import dataclass
from typing import List

# 常量定义（根据EDS文件调整）
ENCODER_RESOLUTION = 65536  # 编码器分辨率
GEAR_RATIO = 101.0  # 减速比
TORQUE_CONSTANT = 14  # 扭矩常数（Nm/A）
CURRENT_SCALE = 1000.0  # 电流比例系数


@dataclass
class MotorState:
    position: float = 0.0  # 位置（弧度）
    velocity: float = 0.0  # 速度（弧度/秒）
    torque: float = 0.0  # 扭矩（Nm）
    status: int = 0  # 状态字
    error: int = 0  # 错误码


class PDOMonitor:
    def __init__(self, node_id=1, eds_path='YiyouServo.eds', channel='can0', bitrate=1000000):
        self.node_id = node_id
        self.network = canopen.Network()
        self.running = True
        self.print_lock = threading.Lock()
        self.motor_state = MotorState()

        # 添加节点
        self.node = canopen.RemoteNode(node_id, eds_path)
        self.network.add_node(self.node)

        # 连接CAN总线
        self.network.connect(bustype='socketcan', channel=channel, bitrate=bitrate)
        self.network.send_message(0x000, [0x01, node_id])
        self.network.send_message(0x000, [0x80, 1])
        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)

        # 订阅PDO消息
        self.network.subscribe(0x180 + node_id, self._pdo_callback)  # TPDO1
        self.network.subscribe(0x280 + node_id, self._pdo_callback)  # TPDO2

    def _signal_handler(self, sig, frame):
        print("\nShutting down...")
        self.running = False
        self.network.disconnect()
        sys.exit(0)

    @staticmethod
    def counts_to_radians(counts: int) -> float:
        """将编码器计数转换为弧度"""
        return (2 * math.pi * counts) / (ENCODER_RESOLUTION * GEAR_RATIO)

    @staticmethod
    def counts_to_rad_per_sec(counts: int) -> float:
        """将速度计数转换为弧度/秒"""
        return (2 * math.pi * counts) / (ENCODER_RESOLUTION * GEAR_RATIO)

    @staticmethod
    def counts_to_newton_meters(counts: int) -> float:
        """将扭矩计数转换为牛顿米"""
        return counts * TORQUE_CONSTANT / CURRENT_SCALE

    def _pdo_callback(self, message):
        """处理接收到的PDO消息"""
        can_id = message.arbitration_id
        data = message.data

        if can_id == 0x180 + self.node_id:  # TPDO1: 状态字、位置、扭矩
            if len(data) >= 8:
                # 解析状态字 (小端序uint16)
                status = int.from_bytes(data[0:2], byteorder='little')
                # 解析位置 (小端序int32)
                position = int.from_bytes(data[2:6], byteorder='little', signed=True)
                # 解析扭矩 (小端序int16)
                torque = int.from_bytes(data[6:8], byteorder='little', signed=True)

                # 更新状态
                self.motor_state.status = status
                self.motor_state.position = self.counts_to_radians(position)
                self.motor_state.torque = self.counts_to_newton_meters(torque)

                with self.print_lock:
                    print(f"[TPDO1] Status: 0x{status:04X} "
                          f"Pos: {self.motor_state.position:.4f} rad "
                          f"Torque: {self.motor_state.torque:.3f} Nm")

        elif can_id == 0x280 + self.node_id:  # TPDO2: 位置和速度
            if len(data) >= 8:
                # 解析位置 (小端序int32)
                position = int.from_bytes(data[0:4], byteorder='little', signed=True)
                # 解析速度 (小端序int32)
                velocity = int.from_bytes(data[4:8], byteorder='little', signed=True)

                # 更新状态
                self.motor_state.position = self.counts_to_radians(position)
                self.motor_state.velocity = self.counts_to_rad_per_sec(velocity)

                with self.print_lock:
                    print(f"[TPDO2] Pos: {self.motor_state.position:.4f} rad "
                          f"Vel: {self.motor_state.velocity:.2f} rad/s")

    def configure_tpdo(self, node_id=1, auto_report_ms=10):
        """根据YiyouServo EDS文件的TPDO配置"""
        try:
            self.node = self.network[node_id]
            for index in range(4):  # TPDO0到TPDO3（即TPDO1到TPDO4）
                # 1. 禁用TPDO (设置COB-ID最高位)
                cob_id = 0x180 + index * 0x100 + node_id  # 不设置禁止位
                self.node.sdo[0x1800 + index][1].raw = cob_id  # 0x1800:TPDO1, 0x1801:TPDO2...

                # 2. 清空映射
                self.node.sdo[0x1A00 + index][0].raw = 0

                # 3. 配置传输参数
                self.node.sdo[0x1800 + index][2].raw = 0xFF  # 异步传输
                self.node.sdo[0x1800 + index][3].raw = 0  # Inhibit Time
                self.node.sdo[0x1800 + index][5].raw = auto_report_ms  # Event Timer

                print(f"TPDO{index + 1} 基础参数已配置: COB-ID=0x{cob_id:08X}")
                # 设置映射参数 (根据EDS文件中的默认映射)
                # ===== TPDO1配置 (COB-ID: 0x180 + node_id) =====
                # 映射对象1: 实际位置(0x6064:0, 32bit)
                self.node.sdo[0x1A00][1].raw = (0x6064 << 16) | 0x20  # 等效于 (0x6064 << 16) + 0x20
                # 映射对象2: 实际速度(0x606C:0, 32bit)
                self.node.sdo[0x1A00][2].raw = (0x606C << 16) | 0x20
                # 设置映射数量


                # ===== TPDO2配置 (COB-ID: 0x280 + node_id) =====
                # 映射对象1: 实际扭矩(0x6077:0, 16bit)
                self.node.sdo[0x1A01][1].raw = (0x6077 << 16) | 0x10
                # 映射对象2: 状态字(0x6041:0, 16bit)
                self.node.sdo[0x1A01][2].raw = (0x6041 << 16) | 0x10
                # 映射对象3: 错误码(0x603F:0, 16bit)
                self.node.sdo[0x1A01][3].raw = (0x603F << 16) | 0x10
                # 设置映射数量 (注意：3个16bit对象=6字节，未超8字节限制)
                self.node.sdo[0x1A00][0].raw = 2  # 2个映射对象 (共8字节)
                self.node.sdo[0x1A01][0].raw = 3

                # 激活TPDO
                self.node.sdo[0x1800][1].raw = 0x180 + node_id
                self.node.sdo[0x1801][1].raw = 0x280 + node_id

                # ===== 3. 重启节点应用配置 =====
                self.node.nmt.state = 'RESET'
                time.sleep(0.1)
                self.node.nmt.state = 'OPERATIONAL'
                time.sleep(0.5)

                print(f"节点{node_id} TPDO配置成功:")
                print(f"  TPDO1 (0x{0x180 + node_id:03X}): 状态字 + 位置需求 + 扭矩实际")
                print(f"  TPDO2 (0x{0x280 + node_id:03X}): 位置实际 + 速度实际")
                return True

        except Exception as e:
            print(f"TPDO配置失败: {str(e)}")
            return False
    def run(self):
        """主运行循环"""
        if not self.configure_tpdo():
            return

        print("PDO monitor started. Press Ctrl+C to stop.")
        while self.running:
            time.sleep(0.1)


if __name__ == "__main__":
    # 创建PDO监控器
    monitor = PDOMonitor(node_id=1, eds_path='../eds/YiyouServo.eds')

    try:
        # 运行监控
        monitor.run()
    except KeyboardInterrupt:
        print("\nStopping PDO monitor...")
    finally:
        monitor.network.disconnect()