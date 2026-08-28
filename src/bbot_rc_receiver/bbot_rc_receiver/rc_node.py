import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from bbot_rc_receiver.crsf_demo.receiver import CrsfReceiver
from bbot_rc_receiver.crsf_demo.serial_io import (
    SerialConfig,
    SerialPort
)
from bbot_rc_receiver.crsf_demo.telemetry import (
    TelemetrySource
)


class CRSFNode(Node):

    def __init__(self):

        super().__init__('crsf_receiver')

        self.publisher = self.create_publisher(
            Joy,
            '/rc_input',
            10
        )

        serial_cfg = SerialConfig(
            port="/dev/ttyCH341USB1",
            baudrate=420000
        )

        self.port = SerialPort(serial_cfg)

        self.telemetry = TelemetrySource(
            fixed_cfg={
                "battery": {
                    "voltage_v": 16.8,
                    "current_a": 2.0,
                    "capacity_mah": 100,
                    "remaining_pct": 95
                },

                "gps": {
                    "latitude_deg": 35.6895,
                    "longitude_deg": 139.6917,
                    "altitude_m": 10,
                    "satellites": 12
                },

                "flight_mode": "BBOT"
            },

            intervals_cfg={
                "battery": 1.0,
                "gps": 1.0,
                "attitude": 0.1,
                "link_statistics": 1.0,
                "flight_mode": 1.0,
                "heartbeat": 1.0
            },

            source="dynamic"
        )

        self.receiver = CrsfReceiver(
            port=self.port,
            telemetry=self.telemetry,
            on_channels=self.on_channels
        )

        if not self.port.is_open:
            self.port.open()

        self.get_logger().info(
            "CRSF Receiver started on /dev/ttyUSB0 @ 420000"
        )

        self.timer = self.create_timer(
            0.001,
            self.timer_callback
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