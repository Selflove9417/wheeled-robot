"""CRSF 协议层：常量定义、帧封装、流式解析器。

帧结构：
    +------+------+------+----------------+------+
    | addr | len  | type |    payload     | crc  |
    +------+------+------+----------------+------+
       1B     1B     1B       0..62B         1B

- addr：同步/目标地址（0xC8 = Flight Controller）
- len ：从 type 字段到 crc 字段的字节数（即 payload_len + 2）
- type：帧类型
- crc ：CRC8(poly=0xD5)，覆盖 type + payload
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional

from .crc import crc8


# ---------------------------------------------------------------------------
# 地址（同步字节）
# ---------------------------------------------------------------------------
class CrsfAddress(IntEnum):
    BROADCAST = 0x00
    USB = 0x10
    TBS_CORE_PNP_PRO = 0x80
    RESERVED1 = 0x8A
    CURRENT_SENSOR = 0xC0
    GPS = 0xC2
    TBS_BLACKBOX = 0xC4
    FLIGHT_CONTROLLER = 0xC8
    RESERVED2 = 0xCA
    RACE_TAG = 0xCC
    RADIO_TRANSMITTER = 0xEA
    CRSF_RECEIVER = 0xEC
    CRSF_TRANSMITTER = 0xEE


# ---------------------------------------------------------------------------
# 帧类型
# ---------------------------------------------------------------------------
class CrsfFrameType(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALTITUDE = 0x09
    HEARTBEAT = 0x0B
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    SUBSET_RC_CHANNELS_PACKED = 0x17
    LINK_RX_ID = 0x1C
    LINK_TX_ID = 0x1D
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    # 扩展类型
    DEVICE_PING = 0x28
    DEVICE_INFO = 0x29
    PARAMETER_SETTINGS_ENTRY = 0x2B
    PARAMETER_READ = 0x2C
    PARAMETER_WRITE = 0x2D
    COMMAND = 0x32
    RADIO_ID = 0x3A


# ---------------------------------------------------------------------------
# 帧上限（按 CRSF 规范）
# ---------------------------------------------------------------------------
CRSF_MAX_PACKET_SIZE: int = 64        # 包含 addr/len/type/payload/crc
CRSF_MAX_PAYLOAD_LEN: int = 60        # = 64 - 4
CRSF_SYNC_BYTE: int = int(CrsfAddress.FLIGHT_CONTROLLER)   # 0xC8
CRSF_BAUDRATE_DEFAULT: int = 420_000


# ---------------------------------------------------------------------------
# 帧数据类
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CrsfFrame:
    """已解析的 CRSF 帧。"""

    address: int
    frame_type: int
    payload: bytes

    def to_bytes(self) -> bytes:
        return build_frame(self.frame_type, self.payload, address=self.address)


# ---------------------------------------------------------------------------
# 打包
# ---------------------------------------------------------------------------
def build_frame(frame_type: int,
                payload: bytes,
                address: int = CRSF_SYNC_BYTE) -> bytes:
    """构造一个完整的 CRSF 帧（含 addr/len/type/payload/crc）。"""
    if len(payload) > CRSF_MAX_PAYLOAD_LEN:
        raise ValueError(
            f"payload too long: {len(payload)} > {CRSF_MAX_PAYLOAD_LEN}")
    if not 0 <= frame_type <= 0xFF:
        raise ValueError(f"invalid frame_type: {frame_type}")
    if not 0 <= address <= 0xFF:
        raise ValueError(f"invalid address: {address}")

    length = len(payload) + 2          # type(1) + payload + crc(1)
    body = bytes([frame_type]) + payload
    checksum = crc8(body)
    return bytes([address, length]) + body + bytes([checksum])


# ---------------------------------------------------------------------------
# 流式解析器
# ---------------------------------------------------------------------------
class CrsfParser:
    """字节流解析器。

    使用方法::

        parser = CrsfParser()
        for frame in parser.feed(chunk_from_serial):
            handle(frame)

    解析器是有状态的：当一次只接收到帧的一部分字节时，下次喂入剩余字节
    仍然能够正常拼接。CRC 校验失败的帧会被丢弃（计入 ``crc_errors``），
    并尝试在缓冲区中重新同步到下一个有效帧。
    """

    # 允许出现的同步字节（地址）。CRSF 实际上对地址校验比较宽松，
    # 这里取常见的几个，避免误同步。
    _VALID_SYNC: frozenset = frozenset({
        int(CrsfAddress.FLIGHT_CONTROLLER),
        int(CrsfAddress.RADIO_TRANSMITTER),
        int(CrsfAddress.CRSF_RECEIVER),
        int(CrsfAddress.CRSF_TRANSMITTER),
        int(CrsfAddress.BROADCAST),
        int(CrsfAddress.USB),
    })

    def __init__(self) -> None:
        self._buf: bytearray = bytearray()
        self.crc_errors: int = 0
        self.dropped_bytes: int = 0

    def feed(self, data: bytes) -> List[CrsfFrame]:
        """喂入新的字节，返回解析出来的所有完整帧。"""
        if data:
            self._buf.extend(data)
        return self._extract()

    def reset(self) -> None:
        self._buf.clear()

    # ------------------------------------------------------------------
    def _extract(self) -> List[CrsfFrame]:
        out: List[CrsfFrame] = []
        while True:
            # 1) 找同步字节
            while self._buf and self._buf[0] not in self._VALID_SYNC:
                self._buf.pop(0)
                self.dropped_bytes += 1

            if len(self._buf) < 2:
                return out

            length = self._buf[1]
            # 长度合法性检查：length 表示 type + payload + crc
            # 合法范围 2 (空 payload) .. CRSF_MAX_PAYLOAD_LEN+2
            if length < 2 or length > CRSF_MAX_PAYLOAD_LEN + 2:
                # 非法长度 → 丢弃同步字节，重新同步
                self._buf.pop(0)
                self.dropped_bytes += 1
                continue

            total = length + 2  # +addr +len
            if len(self._buf) < total:
                return out      # 数据不足，等下次

            address = self._buf[0]
            frame_type = self._buf[2]
            payload = bytes(self._buf[3:1 + length])      # type 后到 crc 前
            recv_crc = self._buf[1 + length]              # 帧最后一字节
            calc_crc = crc8(self._buf[2:1 + length])      # type + payload

            if recv_crc == calc_crc:
                out.append(CrsfFrame(address=address,
                                     frame_type=frame_type,
                                     payload=payload))
                del self._buf[:total]
            else:
                self.crc_errors += 1
                # CRC 失败：丢弃首字节，尝试重新同步
                self._buf.pop(0)
                self.dropped_bytes += 1
                continue


__all__ = [
    "CrsfAddress",
    "CrsfFrameType",
    "CrsfFrame",
    "CrsfParser",
    "build_frame",
    "CRSF_MAX_PACKET_SIZE",
    "CRSF_MAX_PAYLOAD_LEN",
    "CRSF_SYNC_BYTE",
    "CRSF_BAUDRATE_DEFAULT",
]
