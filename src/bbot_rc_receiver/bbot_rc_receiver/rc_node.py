import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

from bbot_rc_receiver.crsf_demo.receiver import CrsfReceiver
from bbot_rc_receiver.crsf_demo.serial_io import (
    SerialConfig,
    SerialPort
)
from bbot_rc_receiver.crsf_demo.telemetry import (
    TelemetrySource
)

# 控制器 /bbot/telemetry 数据索引
IDX_SPEED = 0        # 实际前进速度 (m/s)
IDX_PITCH = 2        # 实际俯仰角 Pitch (rad)
IDX_HEIGHT = 12      # 当前中心腿高 (m)
IDX_ROLL = 28        # 实际横滚角 Roll (rad)


class CRSFNode(Node):

    def __init__(self):

        super().__init__('crsf_receiver')

        self.publisher = self.create_publisher(
            Joy,
            '/rc_input',
            10
        )

        serial_port = "/dev/rc"
        serial_baudrate = 420000

        serial_cfg = SerialConfig(
            port=serial_port,
            baudrate=serial_baudrate
        )

        self.port = SerialPort(serial_cfg)

        # 回传机器人真实状态：只回传速度、高度、pitch、roll。
        # 帧映射：
        #   ATTITUDE (10Hz)    -> P/R: pitch、roll
        #   GPS (2Hz)          -> GSpd: 速度
        #   BARO_ALTITUDE (2Hz)-> Alt: 腿高
        # 其余帧类型（电池/链路统计/心跳/飞行模式/升降率）全部停发。
        self.telemetry = TelemetrySource(
            fixed_cfg={},

            intervals_cfg={
                "battery": 0.0,
                "gps": 0.5,
                "vario": 0.0,
                "baro_altitude": 0.5,
                "attitude": 0.1,
                "link_statistics": 0.0,
                "flight_mode": 0.0,
                "heartbeat": 0.0
            },

            source="external"
        )

        self.receiver = CrsfReceiver(
            port=self.port,
            telemetry=self.telemetry,
            on_channels=self.on_channels
        )

        if not self.port.is_open:
            self.port.open()

        self.get_logger().info(
            f"CRSF Receiver started on {serial_port} @ {serial_baudrate}"
        )

        self.telemetry_sub = self.create_subscription(
            Float64MultiArray,
            '/bbot/telemetry',
            self.on_telemetry,
            10
        )

        self.timer = self.create_timer(
            0.001,
            self.timer_callback
        )

    def on_telemetry(self, msg: Float64MultiArray):
        """缓存控制器状态，写入 CRSF 回传状态容器。"""

        data = msg.data
        if len(data) <= IDX_HEIGHT:
            return

        speed = data[IDX_SPEED]
        pitch = data[IDX_PITCH]
        height = data[IDX_HEIGHT]
        # [28] 是新加的索引，旧版本控制器没有，缺失时保持 0
        roll = data[IDX_ROLL] if len(data) > IDX_ROLL else 0.0

        self.telemetry.apply_robot_data(
            speed_mps=speed,
            height_m=height,
            pitch_rad=pitch,
            roll_rad=roll
        )

    def timer_callback(self):

        try:
            self.receiver._tick()

        except Exception as e:

            self.get_logger().error(
                f"CRSF error: {e}"
            )

    def normalize(self, ch):

        return (ch - 992.0) / 820.0

    def on_channels(self, channels):

        if len(channels) < 16:
            return

        msg = Joy()

        msg.axes = [
            self.normalize(channels[0]),
            self.normalize(channels[1]),
            self.normalize(channels[2]),
            self.normalize(channels[3])
        ]

        msg.buttons = [
            int(c > 1500)
            for c in channels[4:16]
        ]

        self.publisher.publish(msg)

    def destroy_node(self):

        try:
            self.port.close()

        except Exception:
            pass

        super().destroy_node()


def main():

    rclpy.init()

    node = CRSFNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
