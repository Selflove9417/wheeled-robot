"""CRSF CRC8 实现。

CRSF 使用 CRC-8/DVB-S2，多项式为 0xD5，初始值 0x00，无反射，无 XOR-Out。
计算范围：从 frame[1]（type 字段，紧跟 length 之后）开始，
到 payload 末尾（不含最后的 CRC 字节）。

实现采用预计算查找表，性能足够实时使用。
"""

from __future__ import annotations

from typing import Iterable

# CRSF CRC8 多项式
_POLY: int = 0xD5


def _build_table(poly: int) -> list[int]:
    table: list[int] = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC8_TABLE: list[int] = _build_table(_POLY)


def crc8(data: Iterable[int], initial: int = 0x00) -> int:
    """计算 CRSF 标准 CRC8（poly=0xD5）。

    Args:
        data: 待计算的字节序列（int 可迭代，每个元素 0~255）。
        initial: 初始 CRC 值，默认 0x00。

    Returns:
        计算所得的 8 位 CRC 值（0~255）。
    """
    crc = initial & 0xFF
    for byte in data:
        crc = _CRC8_TABLE[(crc ^ byte) & 0xFF]
    return crc


__all__ = ["crc8"]
