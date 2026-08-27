"""跨平台串口封装（基于 pyserial）。

支持：
- Windows : ``COM3`` 之类的端口名
- Linux   : ``/dev/ttyUSB0``、``/dev/ttyACM0`` 等 USB 转串口设备
- macOS   : ``/dev/tty.usbserial-XXXX``

功能：
- 连接/断开
- 读 / 写 字节
- 列出本机所有可用串口（``list_available_ports``）
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional

import serial
from serial.tools import list_ports

from .protocol import CRSF_BAUDRATE_DEFAULT


logger = logging.getLogger(__name__)


@dataclass
class SerialConfig:
    port: str
    baudrate: int = CRSF_BAUDRATE_DEFAULT
    bytesize: int = serial.EIGHTBITS
    parity: str = serial.PARITY_NONE
    stopbits: float = serial.STOPBITS_ONE
    timeout: float = 0.02
    write_timeout: float = 0.05

    @classmethod
    def from_dict(cls, cfg: dict) -> "SerialConfig":
        parity_map = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
            "M": serial.PARITY_MARK,
            "S": serial.PARITY_SPACE,
        }
        bytesize_map = {
            5: serial.FIVEBITS, 6: serial.SIXBITS,
            7: serial.SEVENBITS, 8: serial.EIGHTBITS,
        }
        stopbits_map = {
            1: serial.STOPBITS_ONE,
            1.5: serial.STOPBITS_ONE_POINT_FIVE,
            2: serial.STOPBITS_TWO,
        }
        return cls(
            port=cfg["port"],
            baudrate=int(cfg.get("baudrate", CRSF_BAUDRATE_DEFAULT)),
            bytesize=bytesize_map.get(int(cfg.get("bytesize", 8)),
                                      serial.EIGHTBITS),
            parity=parity_map.get(str(cfg.get("parity", "N")).upper(),
                                  serial.PARITY_NONE),
            stopbits=stopbits_map.get(float(cfg.get("stopbits", 1)),
                                      serial.STOPBITS_ONE),
            timeout=float(cfg.get("timeout", 0.02)),
            write_timeout=float(cfg.get("write_timeout", 0.05)),
        )


class SerialPort:
    """对 :class:`serial.Serial` 的轻量包装，提供更友好的接口。"""

    def __init__(self, config: SerialConfig) -> None:
        self._cfg = config
        self._ser: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    def open(self) -> None:
        if self._ser is not None and self._ser.is_open:
            return
        logger.info("Opening serial %s @ %d baud",
                    self._cfg.port, self._cfg.baudrate)
        self._ser = serial.Serial(
            port=self._cfg.port,
            baudrate=self._cfg.baudrate,
            bytesize=self._cfg.bytesize,
            parity=self._cfg.parity,
            stopbits=self._cfg.stopbits,
            timeout=self._cfg.timeout,
            write_timeout=self._cfg.write_timeout,
        )

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            logger.info("Closing serial %s", self._cfg.port)
            self._ser.close()
        self._ser = None

    # ------------------------------------------------------------------
    def read(self, max_bytes: int = 256) -> bytes:
        """非阻塞读取（受 timeout 限制）。返回任意长度（含 0）的字节。"""
        if self._ser is None:
            raise RuntimeError("serial not open")
        # in_waiting 让我们一次性把已到达的字节读完，没有则按 timeout 等
        n = max(1, min(max_bytes, self._ser.in_waiting or 1))
        return self._ser.read(n)

    def write(self, data: bytes) -> int:
        if self._ser is None:
            raise RuntimeError("serial not open")
        return self._ser.write(data) or 0

    def flush(self) -> None:
        if self._ser is not None:
            self._ser.flush()

    # ------------------------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def __enter__(self) -> "SerialPort":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def list_available_ports() -> List[str]:
    """列出本机所有可用串口名（用于排错）。"""
    return [p.device for p in list_ports.comports()]


__all__ = ["SerialConfig", "SerialPort", "list_available_ports"]
