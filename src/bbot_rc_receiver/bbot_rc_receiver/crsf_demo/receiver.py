"""CRSF 串口端点：被动解析控制帧并回传遥测。

工作流程
========

1. 打开串口（对端可以是任何 CRSF 主控：真实飞控、真实接收机、
   自定义 CRSF 设备、或串口调试工具）。
2. 持续读取串口字节流，通过 :class:`CrsfParser` 解析帧。
3. 收到 ``RC_CHANNELS_PACKED (0x16)`` 时，回调用户函数（默认仅打印）。
4. 按各遥测帧配置的发送周期，主动向对端回传遥测帧。

本模块不模拟"对端"，也不主动发起通道帧；仅专注于"收控制帧 → 回传遥测"
这一单一职责。CRSF 在 USB-UART 上用的是全双工 TTL，收发互不影响。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from .frames import unpack_rc_channels
from .protocol import CrsfFrame, CrsfFrameType, CrsfParser
from .serial_io import SerialPort
from .telemetry import TelemetrySource


logger = logging.getLogger(__name__)


ChannelsCallback = Callable[[List[int]], None]
FrameCallback = Callable[[CrsfFrame], None]


@dataclass
class ReceiverStats:
    rx_frames: int = 0
    rx_channel_frames: int = 0
    tx_frames: int = 0
    crc_errors: int = 0
    dropped_bytes: int = 0


class CrsfReceiver:
    """CRSF 串口端点：解析进来的控制帧并周期性回传遥测帧。"""

    def __init__(self,
                 port: SerialPort,
                 telemetry: TelemetrySource,
                 on_channels: Optional[ChannelsCallback] = None,
                 on_frame: Optional[FrameCallback] = None) -> None:
        self._port = port
        self._telemetry = telemetry
        self._parser = CrsfParser()
        self._on_channels = on_channels or self._default_print_channels
        self._on_frame = on_frame
        self._running = False
        self.stats = ReceiverStats()
        self.last_channels: Optional[List[int]] = None

    # ------------------------------------------------------------------
    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        """阻塞主循环，直到调用 :meth:`stop` 或 KeyboardInterrupt。"""
        if not self._port.is_open:
            self._port.open()
        self._running = True
        logger.info("CRSF endpoint started (parse-and-reply)")
        try:
            while self._running:
                self._tick()
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt → stopping")
        finally:
            self._running = False
            logger.info("CRSF endpoint stopped. stats=%s", self.stats)

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        # 1) 读串口
        chunk = self._port.read(max_bytes=512)
        if chunk:
            for frame in self._parser.feed(chunk):
                self._handle_frame(frame)

        # 同步解析器统计
        self.stats.crc_errors = self._parser.crc_errors
        self.stats.dropped_bytes = self._parser.dropped_bytes

        # 2) 推进遥测状态
        now = time.monotonic()
        self._telemetry.update(now)

        # 3) 把到期的遥测帧写出
        for raw in self._telemetry.due_frames(now):
            self._port.write(raw)
            self.stats.tx_frames += 1

        # 4) 让出 CPU；CRSF 主控帧约 250Hz，1ms 粒度足够
        time.sleep(0.001)

    # ------------------------------------------------------------------
    def _handle_frame(self, frame: CrsfFrame) -> None:
        self.stats.rx_frames += 1
        if self._on_frame is not None:
            try:
                self._on_frame(frame)
            except Exception as exc:                        # noqa: BLE001
                logger.warning("on_frame callback raised: %s", exc)

        if frame.frame_type == CrsfFrameType.RC_CHANNELS_PACKED:
            try:
                channels = unpack_rc_channels(frame.payload)
            except ValueError as exc:
                logger.warning("bad channel payload: %s", exc)
                return
            self.last_channels = channels
            self.stats.rx_channel_frames += 1
            try:
                self._on_channels(channels)
            except Exception as exc:                        # noqa: BLE001
                logger.warning("on_channels callback raised: %s", exc)

    # ------------------------------------------------------------------
    @staticmethod
    def _default_print_channels(channels: List[int]) -> None:
        # 只打印前 8 通道，避免刷屏
        preview = " ".join(f"{c:4d}" for c in channels[:8])
        logger.info("RC channels (1-8): %s", preview)


__all__ = ["CrsfReceiver", "ReceiverStats"]
