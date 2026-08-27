"""CRSF 各帧类型的编/解码。

所有多字节整型字段：**大端**序（big-endian）。
单位与缩放因子参考 Betaflight `crsf_protocol.h` 与 ExpressLRS 文档。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Tuple

from .protocol import (
    CRSF_SYNC_BYTE,
    CrsfAddress,
    CrsfFrameType,
    build_frame,
)


# ===========================================================================
# 0x16  RC_CHANNELS_PACKED  ——  16ch × 11bit = 22 bytes
# ===========================================================================
RC_CHANNELS_COUNT: int = 16
RC_CHANNELS_PAYLOAD_LEN: int = 22
RC_CHANNEL_MIN: int = 172     # ≈ 988us
RC_CHANNEL_MID: int = 992     # ≈ 1500us
RC_CHANNEL_MAX: int = 1811    # ≈ 2012us


def pack_rc_channels(channels: List[int]) -> bytes:
    """将 16 个 11bit 通道值打包为 22 字节 payload（小端位拼接）。"""
    if len(channels) != RC_CHANNELS_COUNT:
        raise ValueError(f"need {RC_CHANNELS_COUNT} channels, got {len(channels)}")

    bits = 0
    for i, ch in enumerate(channels):
        v = max(0, min(0x7FF, int(ch))) & 0x7FF
        bits |= v << (11 * i)

    out = bytearray(RC_CHANNELS_PAYLOAD_LEN)
    for i in range(RC_CHANNELS_PAYLOAD_LEN):
        out[i] = (bits >> (8 * i)) & 0xFF
    return bytes(out)


def unpack_rc_channels(payload: bytes) -> List[int]:
    """解析 22 字节 payload 为 16 通道列表。"""
    if len(payload) != RC_CHANNELS_PAYLOAD_LEN:
        raise ValueError(
            f"payload length {len(payload)} != {RC_CHANNELS_PAYLOAD_LEN}")

    bits = 0
    for i, b in enumerate(payload):
        bits |= b << (8 * i)

    return [(bits >> (11 * i)) & 0x7FF for i in range(RC_CHANNELS_COUNT)]


def build_rc_channels_frame(channels: List[int]) -> bytes:
    """构造完整的 RC 通道帧（地址=FC）。"""
    return build_frame(
        CrsfFrameType.RC_CHANNELS_PACKED,
        pack_rc_channels(channels),
        address=int(CrsfAddress.FLIGHT_CONTROLLER),
    )


# ===========================================================================
# 0x08  BATTERY_SENSOR
#   voltage   : uint16, 0.1 V
#   current   : uint16, 0.1 A
#   capacity  : uint24, mAh used
#   remaining : uint8 , %
# ===========================================================================
@dataclass
class BatterySensor:
    voltage_v: float = 0.0
    current_a: float = 0.0
    capacity_mah: int = 0
    remaining_pct: int = 0

    def to_payload(self) -> bytes:
        v = int(round(self.voltage_v * 10)) & 0xFFFF
        c = int(round(self.current_a * 10)) & 0xFFFF
        cap = int(self.capacity_mah) & 0xFFFFFF
        rem = int(self.remaining_pct) & 0xFF
        return (
            struct.pack(">HH", v, c)
            + bytes([(cap >> 16) & 0xFF, (cap >> 8) & 0xFF, cap & 0xFF])
            + bytes([rem])
        )

    @classmethod
    def from_payload(cls, payload: bytes) -> "BatterySensor":
        if len(payload) != 8:
            raise ValueError(f"battery payload must be 8B, got {len(payload)}")
        v, c = struct.unpack(">HH", payload[0:4])
        cap = (payload[4] << 16) | (payload[5] << 8) | payload[6]
        rem = payload[7]
        return cls(voltage_v=v / 10.0,
                   current_a=c / 10.0,
                   capacity_mah=cap,
                   remaining_pct=rem)


def build_battery_frame(b: BatterySensor) -> bytes:
    return build_frame(CrsfFrameType.BATTERY_SENSOR, b.to_payload())


# ===========================================================================
# 0x02  GPS
#   latitude   : int32, deg * 1e7
#   longitude  : int32, deg * 1e7
#   groundspeed: uint16, km/h * 10  (= 0.1 km/h)
#   heading    : uint16, deg * 100
#   altitude   : uint16, m + 1000   (offset 1000)
#   satellites : uint8
# ===========================================================================
@dataclass
class GpsData:
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    ground_speed_kmh: float = 0.0
    heading_deg: float = 0.0
    altitude_m: int = 0
    satellites: int = 0

    def to_payload(self) -> bytes:
        lat = int(round(self.latitude_deg * 1e7))
        lon = int(round(self.longitude_deg * 1e7))
        spd = int(round(self.ground_speed_kmh * 10)) & 0xFFFF
        hdg = int(round(self.heading_deg * 100)) & 0xFFFF
        alt = (int(self.altitude_m) + 1000) & 0xFFFF
        sat = int(self.satellites) & 0xFF
        return struct.pack(">iiHHHB", lat, lon, spd, hdg, alt, sat)

    @classmethod
    def from_payload(cls, payload: bytes) -> "GpsData":
        if len(payload) != 15:
            raise ValueError(f"gps payload must be 15B, got {len(payload)}")
        lat, lon, spd, hdg, alt, sat = struct.unpack(">iiHHHB", payload)
        return cls(
            latitude_deg=lat / 1e7,
            longitude_deg=lon / 1e7,
            ground_speed_kmh=spd / 10.0,
            heading_deg=hdg / 100.0,
            altitude_m=alt - 1000,
            satellites=sat,
        )


def build_gps_frame(g: GpsData) -> bytes:
    return build_frame(CrsfFrameType.GPS, g.to_payload())


# ===========================================================================
# 0x07  VARIO  ——  vertical_speed: int16, cm/s
# ===========================================================================
@dataclass
class Vario:
    vertical_speed_mps: float = 0.0

    def to_payload(self) -> bytes:
        cms = int(round(self.vertical_speed_mps * 100))
        return struct.pack(">h", max(-32768, min(32767, cms)))

    @classmethod
    def from_payload(cls, payload: bytes) -> "Vario":
        if len(payload) != 2:
            raise ValueError(f"vario payload must be 2B, got {len(payload)}")
        (cms,) = struct.unpack(">h", payload)
        return cls(vertical_speed_mps=cms / 100.0)


def build_vario_frame(v: Vario) -> bytes:
    return build_frame(CrsfFrameType.VARIO, v.to_payload())


# ===========================================================================
# 0x09  BARO_ALTITUDE
#   altitude       : uint16  —— 高位为 1 表示 m，0 表示 dm；常用 m
#                    本实现按 "altitude_dm + 10000" 编码（dm，offset 10000，
#                    即 0 = -1000m, 65535 ≈ 5553.5m），与 OpenTX 的 BARO_ALT 
#                    解码一致；不使用最高位标志位。
#   vertical_speed : int16   —— cm/s（可选，新版协议字段）
# ===========================================================================
@dataclass
class BaroAltitude:
    altitude_m: float = 0.0
    vertical_speed_mps: float = 0.0

    def to_payload(self) -> bytes:
        dm = int(round(self.altitude_m * 10)) + 10000
        dm = max(0, min(0xFFFF, dm))
        cms = int(round(self.vertical_speed_mps * 100))
        cms = max(-32768, min(32767, cms))
        return struct.pack(">Hh", dm, cms)

    @classmethod
    def from_payload(cls, payload: bytes) -> "BaroAltitude":
        if len(payload) == 2:
            (dm,) = struct.unpack(">H", payload)
            return cls(altitude_m=(dm - 10000) / 10.0, vertical_speed_mps=0.0)
        if len(payload) == 4:
            dm, cms = struct.unpack(">Hh", payload)
            return cls(altitude_m=(dm - 10000) / 10.0,
                       vertical_speed_mps=cms / 100.0)
        raise ValueError(f"baro payload len {len(payload)} not in (2,4)")


def build_baro_altitude_frame(b: BaroAltitude) -> bytes:
    return build_frame(CrsfFrameType.BARO_ALTITUDE, b.to_payload())


# ===========================================================================
# 0x1E  ATTITUDE
#   pitch / roll / yaw  : int16, rad * 10000
# ===========================================================================
@dataclass
class Attitude:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0

    @staticmethod
    def _deg2raw(deg: float) -> int:
        rad = math.radians(deg)
        return max(-32768, min(32767, int(round(rad * 10000))))

    @staticmethod
    def _raw2deg(raw: int) -> float:
        return math.degrees(raw / 10000.0)

    def to_payload(self) -> bytes:
        return struct.pack(">hhh",
                           self._deg2raw(self.pitch_deg),
                           self._deg2raw(self.roll_deg),
                           self._deg2raw(self.yaw_deg))

    @classmethod
    def from_payload(cls, payload: bytes) -> "Attitude":
        if len(payload) != 6:
            raise ValueError(f"attitude payload must be 6B, got {len(payload)}")
        p, r, y = struct.unpack(">hhh", payload)
        return cls(pitch_deg=cls._raw2deg(p),
                   roll_deg=cls._raw2deg(r),
                   yaw_deg=cls._raw2deg(y))


def build_attitude_frame(a: Attitude) -> bytes:
    return build_frame(CrsfFrameType.ATTITUDE, a.to_payload())


# ===========================================================================
# 0x21  FLIGHT_MODE  ——  以 NUL 结尾的 ASCII 字符串
# ===========================================================================
@dataclass
class FlightMode:
    mode: str = "ACRO"

    def to_payload(self) -> bytes:
        s = self.mode.encode("ascii", errors="replace")
        if not s.endswith(b"\x00"):
            s += b"\x00"
        return s

    @classmethod
    def from_payload(cls, payload: bytes) -> "FlightMode":
        s = payload.rstrip(b"\x00").decode("ascii", errors="replace")
        return cls(mode=s)


def build_flight_mode_frame(m: FlightMode) -> bytes:
    return build_frame(CrsfFrameType.FLIGHT_MODE, m.to_payload())


# ===========================================================================
# 0x14  LINK_STATISTICS  —— 10 bytes，全部 uint8 / int8
#   uplink_rssi_ant1 (uint8, -dBm)
#   uplink_rssi_ant2 (uint8, -dBm)
#   uplink_lq        (uint8, %)
#   uplink_snr       (int8 , dB)
#   active_antenna   (uint8, 0/1)
#   rf_mode          (uint8)
#   uplink_tx_power  (uint8, enum)
#   downlink_rssi    (uint8, -dBm)
#   downlink_lq      (uint8, %)
#   downlink_snr     (int8 , dB)
# ===========================================================================
# 发射功率枚举（mW）→ 协议值
_TX_POWER_TABLE: Tuple[Tuple[int, int], ...] = (
    (0, 0),     # 0 mW
    (10, 1),    # 10 mW
    (25, 2),    # 25 mW
    (100, 3),   # 100 mW
    (500, 4),   # 500 mW
    (1000, 5),  # 1 W
    (2000, 6),  # 2 W
    (250, 7),   # 250 mW
    (50, 8),    # 50 mW
)


def _tx_power_to_enum(mw: int) -> int:
    for power_mw, code in _TX_POWER_TABLE:
        if power_mw == mw:
            return code
    # 找不到精确匹配则回退为 100mW
    return 3


@dataclass
class LinkStatistics:
    uplink_rssi_dbm: int = -60      # 通常为负数；协议字段是 -dBm
    uplink_rssi_ant2_dbm: int = -60
    uplink_lq_pct: int = 100
    uplink_snr_db: int = 0
    active_antenna: int = 0
    rf_mode: int = 2                # 0:4Hz 1:50Hz 2:150Hz ...
    tx_power_mw: int = 100
    downlink_rssi_dbm: int = -60
    downlink_lq_pct: int = 100
    downlink_snr_db: int = 0

    def to_payload(self) -> bytes:
        return bytes([
            (-self.uplink_rssi_dbm) & 0xFF,
            (-self.uplink_rssi_ant2_dbm) & 0xFF,
            self.uplink_lq_pct & 0xFF,
        ]) + struct.pack(">b", max(-128, min(127, self.uplink_snr_db))) + bytes([
            self.active_antenna & 0xFF,
            self.rf_mode & 0xFF,
            _tx_power_to_enum(self.tx_power_mw) & 0xFF,
            (-self.downlink_rssi_dbm) & 0xFF,
            self.downlink_lq_pct & 0xFF,
        ]) + struct.pack(">b", max(-128, min(127, self.downlink_snr_db)))

    @classmethod
    def from_payload(cls, payload: bytes) -> "LinkStatistics":
        if len(payload) != 10:
            raise ValueError(f"link stats payload must be 10B, got {len(payload)}")
        rssi1, rssi2, lq = payload[0], payload[1], payload[2]
        (snr,) = struct.unpack(">b", payload[3:4])
        ant, rfm, txp = payload[4], payload[5], payload[6]
        d_rssi, d_lq = payload[7], payload[8]
        (d_snr,) = struct.unpack(">b", payload[9:10])
        # 反查 tx_power 表
        tx_mw = 0
        for mw, code in _TX_POWER_TABLE:
            if code == txp:
                tx_mw = mw
                break
        return cls(
            uplink_rssi_dbm=-rssi1,
            uplink_rssi_ant2_dbm=-rssi2,
            uplink_lq_pct=lq,
            uplink_snr_db=snr,
            active_antenna=ant,
            rf_mode=rfm,
            tx_power_mw=tx_mw,
            downlink_rssi_dbm=-d_rssi,
            downlink_lq_pct=d_lq,
            downlink_snr_db=d_snr,
        )


def build_link_statistics_frame(s: LinkStatistics) -> bytes:
    return build_frame(CrsfFrameType.LINK_STATISTICS, s.to_payload())


# ===========================================================================
# 0x0B  HEARTBEAT  —— 2B，源地址（设备发送方地址）
# ===========================================================================
@dataclass
class Heartbeat:
    origin_address: int = int(CrsfAddress.FLIGHT_CONTROLLER)

    def to_payload(self) -> bytes:
        return struct.pack(">H", self.origin_address & 0xFFFF)

    @classmethod
    def from_payload(cls, payload: bytes) -> "Heartbeat":
        if len(payload) != 2:
            raise ValueError(f"heartbeat payload must be 2B, got {len(payload)}")
        (a,) = struct.unpack(">H", payload)
        return cls(origin_address=a)


def build_heartbeat_frame(h: Heartbeat) -> bytes:
    return build_frame(CrsfFrameType.HEARTBEAT, h.to_payload())


__all__ = [
    # 通道
    "RC_CHANNEL_MIN", "RC_CHANNEL_MID", "RC_CHANNEL_MAX",
    "RC_CHANNELS_COUNT", "RC_CHANNELS_PAYLOAD_LEN",
    "pack_rc_channels", "unpack_rc_channels", "build_rc_channels_frame",
    # 遥测数据类
    "BatterySensor", "GpsData", "Vario", "BaroAltitude",
    "Attitude", "FlightMode", "LinkStatistics", "Heartbeat",
    # 帧构造器
    "build_battery_frame", "build_gps_frame", "build_vario_frame",
    "build_baro_altitude_frame", "build_attitude_frame",
    "build_flight_mode_frame", "build_link_statistics_frame",
    "build_heartbeat_frame",
]
