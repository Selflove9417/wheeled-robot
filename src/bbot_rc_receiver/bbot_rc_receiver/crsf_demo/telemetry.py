"""遥测数据源。

支持两种模式：

* ``static``  —— 所有字段固定不变（来自配置文件 ``telemetry.fixed``）
* ``dynamic`` —— 部分字段随时间缓慢变化，便于在示波器/监视器上观察
                 双向链路是否真的把"变化"传递到了对端。

类 :class:`TelemetrySource` 既是数据存储，也是带时间戳的"轮询调度器"：
通过 :meth:`due_frames` 在每个主循环 tick 中拿到此刻应该发送的帧字节序列。
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .frames import (
    Attitude,
    BaroAltitude,
    BatterySensor,
    FlightMode,
    GpsData,
    Heartbeat,
    LinkStatistics,
    Vario,
    build_attitude_frame,
    build_baro_altitude_frame,
    build_battery_frame,
    build_flight_mode_frame,
    build_gps_frame,
    build_heartbeat_frame,
    build_link_statistics_frame,
    build_vario_frame,
)


logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 状态容器
# ---------------------------------------------------------------------------
@dataclass
class TelemetryState:
    battery: BatterySensor = field(default_factory=BatterySensor)
    gps: GpsData = field(default_factory=GpsData)
    vario: Vario = field(default_factory=Vario)
    baro: BaroAltitude = field(default_factory=BaroAltitude)
    attitude: Attitude = field(default_factory=Attitude)
    flight_mode: FlightMode = field(default_factory=FlightMode)
    link: LinkStatistics = field(default_factory=LinkStatistics)
    heartbeat: Heartbeat = field(default_factory=Heartbeat)


# ---------------------------------------------------------------------------
# 数据源
# ---------------------------------------------------------------------------
# 飞行模式循环（dynamic 模式下每隔 5s 切一次，验证字符串遥测）
_FLIGHT_MODES_CYCLE = ("ACRO", "ANGL", "HRZN", "RTH*", "WAIT")


class TelemetrySource:
    def __init__(self,
                 fixed_cfg: Dict,
                 intervals_cfg: Dict,
                 source: str = "dynamic") -> None:
        if source not in ("static", "dynamic"):
            raise ValueError(f"unknown telemetry source: {source}")
        self._source = source
        self._intervals: Dict[str, float] = {
            k: float(v) for k, v in intervals_cfg.items()
        }
        self._next_due: Dict[str, float] = {}
        self._t0: float = time.monotonic()
        self.state: TelemetryState = self._init_state(fixed_cfg)

    # ------------------------------------------------------------------
    @staticmethod
    def _init_state(cfg: Dict) -> TelemetryState:
        s = TelemetryState()
        b = cfg.get("battery", {})
        s.battery = BatterySensor(
            voltage_v=float(b.get("voltage_v", 16.8)),
            current_a=float(b.get("current_a", 0.0)),
            capacity_mah=int(b.get("capacity_mah", 0)),
            remaining_pct=int(b.get("remaining_pct", 100)),
        )
        g = cfg.get("gps", {})
        s.gps = GpsData(
            latitude_deg=float(g.get("latitude_deg", 0.0)),
            longitude_deg=float(g.get("longitude_deg", 0.0)),
            ground_speed_kmh=float(g.get("ground_speed_kmh", 0.0)),
            heading_deg=float(g.get("heading_deg", 0.0)),
            altitude_m=int(g.get("altitude_m", 0)),
            satellites=int(g.get("satellites", 0)),
        )
        a = cfg.get("attitude", {})
        s.attitude = Attitude(
            pitch_deg=float(a.get("pitch_deg", 0.0)),
            roll_deg=float(a.get("roll_deg", 0.0)),
            yaw_deg=float(a.get("yaw_deg", 0.0)),
        )
        ba = cfg.get("baro", {})
        s.baro = BaroAltitude(
            altitude_m=float(ba.get("altitude_m", 0.0)),
            vertical_speed_mps=float(ba.get("vertical_speed_mps", 0.0)),
        )
        s.vario = Vario(vertical_speed_mps=s.baro.vertical_speed_mps)
        s.flight_mode = FlightMode(mode=str(cfg.get("flight_mode", "ACRO")))
        lk = cfg.get("link", {})
        s.link = LinkStatistics(
            uplink_rssi_dbm=int(lk.get("uplink_rssi_dbm", -60)),
            uplink_rssi_ant2_dbm=int(lk.get("uplink_rssi_dbm", -60)),
            uplink_lq_pct=int(lk.get("uplink_lq_pct", 100)),
            uplink_snr_db=int(lk.get("uplink_snr_db", 0)),
            rf_mode=int(lk.get("rf_mode", 2)),
            tx_power_mw=int(lk.get("tx_power_mw", 100)),
            downlink_rssi_dbm=int(lk.get("downlink_rssi_dbm", -60)),
            downlink_lq_pct=int(lk.get("downlink_lq_pct", 100)),
            downlink_snr_db=int(lk.get("downlink_snr_db", 0)),
        )
        return s

    # ------------------------------------------------------------------
    def update(self, now: Optional[float] = None) -> None:
        """根据当前模式刷新一次状态。"""
        if self._source == "static":
            return

        now = now if now is not None else time.monotonic()
        t = now - self._t0
        s = self.state

        # 电池：每秒 ~0.005V 缓慢下降（不低于 12.0V）
        s.battery.voltage_v = max(12.0, s.battery.voltage_v - 0.005 * 0.05)
        s.battery.current_a = 8.0 + 4.0 * math.sin(t * 0.5)        # 4..12 A
        s.battery.capacity_mah = int(t * 5)                        # 累计耗电
        s.battery.remaining_pct = max(
            0, min(100, int(100 - t * 0.05)))

        # GPS：以中心点为圆心做半径 ~50m 的圆周（约 4.5e-4 deg）
        omega = 0.05  # rad/s
        radius_deg = 0.00045
        s.gps.latitude_deg = s.gps.latitude_deg  # 保持基准（在 init 已设）
        # 用相对偏移：直接重新构造 lat/lon 围绕配置中心
        # 这里使用首次基准：保存到属性
        if not hasattr(self, "_gps_base"):
            self._gps_base = (s.gps.latitude_deg, s.gps.longitude_deg)
        lat0, lon0 = self._gps_base
        s.gps.latitude_deg = lat0 + radius_deg * math.sin(omega * t)
        s.gps.longitude_deg = lon0 + radius_deg * math.cos(omega * t)
        s.gps.ground_speed_kmh = 30.0 + 10.0 * math.sin(t * 0.3)
        s.gps.heading_deg = (math.degrees(omega * t) + 90.0) % 360.0
        s.gps.altitude_m = int(50 + 20 * math.sin(t * 0.2))
        s.gps.satellites = 12

        # 姿态：pitch/roll 摆动，yaw 缓慢旋转
        s.attitude.pitch_deg = 15.0 * math.sin(t * 0.7)
        s.attitude.roll_deg = 20.0 * math.sin(t * 0.5 + 0.3)
        s.attitude.yaw_deg = (t * 30.0) % 360.0

        # 气压高度：跟随 GPS 高度，叠加 ±2m 抖动
        s.baro.altitude_m = s.gps.altitude_m + 2.0 * math.sin(t * 1.3)
        s.baro.vertical_speed_mps = 2.0 * 0.2 * math.cos(t * 0.2)
        s.vario.vertical_speed_mps = s.baro.vertical_speed_mps

        # 飞行模式：每 5s 切一次
        s.flight_mode.mode = _FLIGHT_MODES_CYCLE[
            int(t // 5) % len(_FLIGHT_MODES_CYCLE)]

        # 链路：RSSI 在 -90..-40 间正弦，LQ 在 80..100 间正弦
        s.link.uplink_rssi_dbm = int(-65 + 25 * math.sin(t * 0.4))
        s.link.uplink_rssi_ant2_dbm = s.link.uplink_rssi_dbm - 2
        s.link.uplink_lq_pct = int(90 + 10 * math.sin(t * 0.6))
        s.link.uplink_snr_db = int(8 + 4 * math.sin(t * 0.3))
        s.link.downlink_rssi_dbm = int(-70 + 20 * math.sin(t * 0.4 + 1.0))
        s.link.downlink_lq_pct = int(88 + 10 * math.sin(t * 0.6 + 0.5))
        s.link.downlink_snr_db = int(6 + 3 * math.sin(t * 0.3 + 0.7))

    # ------------------------------------------------------------------
    def due_frames(self, now: Optional[float] = None) -> List[bytes]:
        """返回此刻到期的所有遥测帧。"""
        now = now if now is not None else time.monotonic()
        out: List[bytes] = []
        s = self.state

        builders = {
            "battery": lambda: build_battery_frame(s.battery),
            "gps": lambda: build_gps_frame(s.gps),
            "vario": lambda: build_vario_frame(s.vario),
            "baro_altitude": lambda: build_baro_altitude_frame(s.baro),
            "attitude": lambda: build_attitude_frame(s.attitude),
            "flight_mode": lambda: build_flight_mode_frame(s.flight_mode),
            "link_statistics": lambda: build_link_statistics_frame(s.link),
            "heartbeat": lambda: build_heartbeat_frame(s.heartbeat),
        }

        for name, build in builders.items():
            interval = self._intervals.get(name, 0.0)
            if interval <= 0:
                continue
            due = self._next_due.get(name, 0.0)
            if now >= due:
                try:
                    out.append(build())
                except Exception as exc:                    # noqa: BLE001
                    logger.warning("build %s failed: %s", name, exc)
                # 防止漂移：基于上次目标时间累加
                self._next_due[name] = max(due, now) + interval
        return out


__all__ = ["TelemetryState", "TelemetrySource"]
