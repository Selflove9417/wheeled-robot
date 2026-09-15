#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bbot 数据绘图统一脚本。

子图分组（可任意组合，不带任何分组参数时默认 --balance）：
  --balance   速度/俯仰/角速度跟踪 + 轮毂电流
  --torque    四关节力矩反馈
  --temp      关节电机线圈温度 + MOS 温度
  --legs      虚拟腿高 + 膝关节电流反馈
  --steering  转向环：直线航向误差 + Yaw-rate 跟踪 + 差动电流
  --all       全部子图

转向图第一幅图说明：
  1. 只在“机器人正在行驶且 target_curvature == 0”时绘制航向误差；
  2. 人工转向区间（target_curvature != 0）用灰色背景标出，不参与直线误差统计；
  3. 航向误差定义：
       e_heading = wrap(target_yaw - actual_yaw)
     并转换为 degree；
  4. ±1 deg / ±3 deg 参考线用于快速判断直线航向保持质量。

常用选项：
  --stats          在跟踪子图上叠加统计框，并打印到终端
  --start / --end  时间窗口（秒）
  --dir            数据目录，默认本脚本所在目录
  --out            输出 PNG 文件名，默认 plot_data.png
  --no-show        只保存图片，不弹窗

示例：
  python3 plot_data.py --steering --stats
  python3 plot_data.py --steering --stats --start 10 --end 50
  python3 plot_data.py --balance --steering --stats
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

SAMPLE_DT = 0.005

# 与当前控制器保持一致
HEADING_DRIVE_THRESHOLD = 0.01
CURVATURE_ZERO_EPS = 1e-6


# ===============================================================
# 数据文件清单
# ===============================================================
DATASET_FILES = {
    "angle": {
        "data": "angle_data.txt",
        "target": "target_angle_data.txt",
        "timestamp": "timestamp_angle.txt",
        "target_timestamp": "timestamp_target_angle.txt",
    },
    "speed": {
        "data": "speed_data.txt",
        "target": "target_speed_data.txt",
        "timestamp": "timestamp_speed.txt",
        "target_timestamp": "timestamp_target_speed.txt",
    },
    "gyro": {
        "data": "gyro_data.txt",
        "target": "target_gyro_data.txt",
        "timestamp": "timestamp_gyro.txt",
        "target_timestamp": "timestamp_target_gyro.txt",
    },
}

CURRENT_FILES = {
    "left": "left_current_data.txt",
    "right": "right_current_data.txt",
    "timestamp_left": "timestamp_left_current.txt",
    "timestamp_right": "timestamp_right_current.txt",
}

TORQUE_FILES = {
    "motor1": "hip_left_torque.txt",
    "motor2": "knee_left_torque.txt",
    "motor3": "hip_right_torque.txt",
    "motor4": "knee_right_torque.txt",
    "timestamp": "timestamp_joint_temp.txt",
}

TEMPERATURE_FILES = {
    "hip_left_motor": "hip_left_motor_temp.txt",
    "hip_left_mos": "hip_left_mos_temp.txt",
    "knee_left_motor": "knee_left_motor_temp.txt",
    "knee_left_mos": "knee_left_mos_temp.txt",
    "hip_right_motor": "hip_right_motor_temp.txt",
    "hip_right_mos": "hip_right_mos_temp.txt",
    "knee_right_motor": "knee_right_motor_temp.txt",
    "knee_right_mos": "knee_right_mos_temp.txt",
    "timestamp": "timestamp_joint_temp.txt",
}

LEG_DIAG_FILES = {
    "height": "current_height.txt",
    "left_height": "left_leg_height.txt",
    "right_height": "right_leg_height.txt",
    "knee_left_current": "knee_left_current_feedback.txt",
    "knee_right_current": "knee_right_current_feedback.txt",
    "knee_left_kt": "knee_left_kt.txt",
    "knee_right_kt": "knee_right_kt.txt",
    "timestamp": "timestamp_leg_diag.txt",
}

STEERING_FILES = {
    "yaw": {
        "data": "yaw_data.txt",
        "target": "target_yaw_data.txt",
        "timestamp": "timestamp_yaw.txt",
        "target_timestamp": "timestamp_yaw.txt",
    },
    "yaw_rate": {
        "data": "yaw_rate_data.txt",
        "target": "target_yaw_rate_data.txt",
        "timestamp": "timestamp_yaw.txt",
        "target_timestamp": "timestamp_yaw.txt",
    },
    "diff_current": "yaw_diff_current_data.txt",
    "curvature": "yaw_curvature_data.txt",
    "timestamp": "timestamp_yaw.txt",

    # PID 控制器实际记录的是 target_speed_smoothed_
    "target_speed": "target_speed_data.txt",
    "target_speed_timestamp": "timestamp_target_speed.txt",
}

ARGS = None


# ===============================================================
# 数据读取与对齐
# ===============================================================
def read_data(file_path, name):
    try:
        with open(file_path, "r") as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f"Loaded {name}: {len(data)} points")
        return np.asarray(data, dtype=float)
    except Exception as e:
        print(f"Failed to read {name}: {e}")
        return np.array([], dtype=float)


def relative_time(t, n):
    """真实时间戳转相对时间；时间戳退化时回退到 200 Hz 采样序号。"""
    if n <= 0:
        return np.array([], dtype=float)
    if len(t) >= n and len(t) > 1 and (t[n - 1] - t[0]) > 0.1:
        return t[:n] - t[0]
    return np.arange(n, dtype=float) * SAMPLE_DT


def process_dataset(base, files, label):
    d = {k: os.path.join(base, v) for k, v in files.items()}
    data = read_data(d["data"], label + " actual")
    target = read_data(d["target"], label + " target")
    t = read_data(d["timestamp"], label + " time")
    tt = read_data(d["target_timestamp"], label + " target time")

    n = min(len(data), len(target), len(t), len(tt))
    if n == 0:
        return {
            "data": np.array([]),
            "target": np.array([]),
            "time": np.array([]),
            "target_time": np.array([]),
        }

    return {
        "data": data[:n],
        "target": target[:n],
        "time": relative_time(t, n),
        "target_time": relative_time(tt, n),
    }


def process_current(base):
    d = {k: os.path.join(base, v) for k, v in CURRENT_FILES.items()}
    left = read_data(d["left"], "left current")
    right = read_data(d["right"], "right current")
    tl = read_data(d["timestamp_left"], "left time")
    tr = read_data(d["timestamp_right"], "right time")

    n = min(len(left), len(right), len(tl), len(tr))
    if n == 0:
        return {
            "left": np.array([]),
            "right": np.array([]),
            "tl": np.array([]),
            "tr": np.array([]),
        }

    return {
        "left": left[:n],
        "right": right[:n],
        "tl": relative_time(tl, n),
        "tr": relative_time(tr, n),
    }


def process_motor(base):
    motor = {}
    for key, name in TORQUE_FILES.items():
        if key == "timestamp":
            continue
        motor[key] = read_data(os.path.join(base, name), key + " torque")

    t = read_data(
        os.path.join(base, TORQUE_FILES["timestamp"]),
        "joint torque time",
    )

    valid_lens = [len(v) for v in motor.values() if len(v) > 0]
    n = min(valid_lens) if valid_lens else 0
    if n == 0:
        motor["time"] = np.array([])
        return motor

    for key in list(motor.keys()):
        motor[key] = motor[key][:n] if len(motor[key]) >= n else np.full(n, np.nan)

    motor["time"] = relative_time(t, n)
    return motor


def process_channel_group(base, files, group_name):
    group = {}
    for key, name in files.items():
        if key == "timestamp":
            continue
        group[key] = read_data(
            os.path.join(base, name),
            key.replace("_", " "),
        )

    t = read_data(
        os.path.join(base, files["timestamp"]),
        group_name + " time",
    )

    if len(t) == 0:
        group["time"] = np.array([])
        return group

    valid_lens = [len(t)]
    valid_lens.extend(len(v) for v in group.values() if len(v) > 0)
    n = min(valid_lens)

    t = t[:n]
    for key in list(group.keys()):
        group[key] = group[key][:n] if len(group[key]) >= n else np.full(n, np.nan)

    group["time"] = relative_time(t, n)
    return group


def _interp_to_time(src_time, src_value, dst_time):
    """把一个日志通道插值到另一个日志的相对时间轴。"""
    if len(src_time) == 0 or len(src_value) == 0 or len(dst_time) == 0:
        return np.full(len(dst_time), np.nan)

    n = min(len(src_time), len(src_value))
    src_time = np.asarray(src_time[:n], dtype=float)
    src_value = np.asarray(src_value[:n], dtype=float)

    valid = np.isfinite(src_time) & np.isfinite(src_value)
    src_time = src_time[valid]
    src_value = src_value[valid]

    if len(src_time) == 0:
        return np.full(len(dst_time), np.nan)
    if len(src_time) == 1:
        return np.full(len(dst_time), src_value[0])

    order = np.argsort(src_time)
    src_time = src_time[order]
    src_value = src_value[order]

    # 删除重复时间点，否则 np.interp 的行为不够直观
    unique_time, unique_idx = np.unique(src_time, return_index=True)
    unique_value = src_value[unique_idx]

    return np.interp(
        dst_time,
        unique_time,
        unique_value,
        left=unique_value[0],
        right=unique_value[-1],
    )


def process_steering_extras(base):
    """
    转向环辅助数据。

    yaw 时间轴上返回：
      diff            差动电流
      curv            target_curvature
      target_speed    target_speed_smoothed_（插值到 yaw 时间轴）
      straight_mask   正在行驶 + 没有人工曲率转向
      manual_mask     target_curvature != 0
      stopped_mask    非人工转向，但目标速度不够大
    """
    diff = read_data(
        os.path.join(base, STEERING_FILES["diff_current"]),
        "yaw diff current",
    )
    curv = read_data(
        os.path.join(base, STEERING_FILES["curvature"]),
        "target curvature",
    )
    t_yaw_raw = read_data(
        os.path.join(base, STEERING_FILES["timestamp"]),
        "steering time",
    )

    target_speed = read_data(
        os.path.join(base, STEERING_FILES["target_speed"]),
        "steering target speed",
    )
    t_speed_raw = read_data(
        os.path.join(base, STEERING_FILES["target_speed_timestamp"]),
        "steering target speed time",
    )

    n = min(len(diff), len(curv), len(t_yaw_raw))
    if n == 0:
        return {
            "diff": np.array([]),
            "curv": np.array([]),
            "target_speed": np.array([]),
            "straight_mask": np.array([], dtype=bool),
            "manual_mask": np.array([], dtype=bool),
            "stopped_mask": np.array([], dtype=bool),
            "time": np.array([]),
        }

    time = relative_time(t_yaw_raw, n)
    diff = diff[:n]
    curv = curv[:n]

    ns = min(len(target_speed), len(t_speed_raw))
    if ns > 0:
        speed_time = relative_time(t_speed_raw, ns)
        target_speed_on_yaw = _interp_to_time(
            speed_time,
            target_speed[:ns],
            time,
        )
    else:
        target_speed_on_yaw = np.full(n, np.nan)

    manual_mask = np.abs(curv) > CURVATURE_ZERO_EPS

    # target_speed_data.txt 在 PID 控制器中记录 target_speed_smoothed_
    drive_mask = (
        np.isfinite(target_speed_on_yaw)
        & (np.abs(target_speed_on_yaw) > HEADING_DRIVE_THRESHOLD)
    )

    straight_mask = drive_mask & (~manual_mask)
    stopped_mask = (~manual_mask) & (~drive_mask)

    return {
        "diff": diff,
        "curv": curv,
        "target_speed": target_speed_on_yaw,
        "straight_mask": straight_mask,
        "manual_mask": manual_mask,
        "stopped_mask": stopped_mask,
        "time": time,
    }


# ===============================================================
# 统计与坐标辅助
# ===============================================================
def _window_mask(time):
    if len(time) == 0:
        return np.array([], dtype=bool)
    lo = ARGS.start if ARGS.start is not None else -np.inf
    hi = ARGS.end if ARGS.end is not None else np.inf
    return (time >= lo) & (time <= hi)


def add_stats_box(ax, time, data, target, unit):
    if len(data) == 0 or len(data) != len(target):
        return None

    error = data - target
    if len(time) == len(error):
        mask = _window_mask(time)
        if np.any(mask):
            error = error[mask]

    error = error[np.isfinite(error)]
    if len(error) == 0:
        return None

    rmse = float(np.sqrt(np.mean(error ** 2)))
    mae = float(np.mean(np.abs(error)))
    max_error = float(np.max(np.abs(error)))

    text = (
        f"RMSE: {rmse:.4f} {unit}\n"
        f"MAE: {mae:.4f} {unit}\n"
        f"MaxE: {max_error:.4f} {unit}\n"
        f"N: {len(error)}"
    )
    ax.text(
        0.02, 0.98, text,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="left",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )
    return rmse, mae, max_error


def add_scalar_stats_box(ax, time, values, valid_mask, unit):
    """对单条误差曲线做统计；用于 Straight-Line Heading Error。"""
    if len(time) == 0 or len(values) == 0:
        return None

    n = min(len(time), len(values), len(valid_mask))
    time = time[:n]
    values = values[:n]
    valid_mask = valid_mask[:n].astype(bool)

    mask = valid_mask & np.isfinite(values) & _window_mask(time)
    x = values[mask]

    if len(x) == 0:
        return None

    rmse = float(np.sqrt(np.mean(x ** 2)))
    mae = float(np.mean(np.abs(x)))
    max_error = float(np.max(np.abs(x)))

    text = (
        "Straight only\n"
        f"RMSE: {rmse:.2f} {unit}\n"
        f"MAE: {mae:.2f} {unit}\n"
        f"MaxE: {max_error:.2f} {unit}\n"
        f"N: {len(x)}"
    )
    ax.text(
        0.02, 0.98, text,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="left",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.55),
    )

    return rmse, mae, max_error


def auto_ylim(ax, x, y):
    if len(x) == 0 or len(y) == 0:
        return

    if len(x) == len(y):
        mask = _window_mask(x)
        window = y[mask] if np.any(mask) else y
    else:
        window = y

    window = np.asarray(window, dtype=float)
    window = window[np.isfinite(window)]
    if len(window) == 0:
        return

    ymin = float(np.min(window))
    ymax = float(np.max(window))
    r = ymax - ymin
    if r < 1e-6:
        r = max(abs(ymax), 0.1)
    margin = r * 0.15
    ax.set_ylim(ymin - margin, ymax + margin)


def nice_tick_step(span):
    for step in (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 30.0, 60.0, 120.0):
        if span / step <= 12:
            return step
    return 300.0


def print_knee_kt_summary(leg_diag):
    for key, label in (
        ("knee_left_kt", "Left"),
        ("knee_right_kt", "Right"),
    ):
        kt = leg_diag.get(key, np.array([]))
        valid = kt[np.isfinite(kt)] if len(kt) > 0 else kt
        if len(valid) > 0:
            print(f"{label} Knee Kt median: {np.median(valid):.4f} Nm/A")


def shade_true_regions(ax, time, mask, label=None, alpha=0.10):
    """
    把布尔 mask 中连续为 True 的区间画成背景色。
    仅第一个区间带 legend label，避免图例重复。
    """
    if len(time) == 0 or len(mask) == 0:
        return

    n = min(len(time), len(mask))
    time = np.asarray(time[:n])
    mask = np.asarray(mask[:n], dtype=bool)

    if not np.any(mask):
        return

    edges = np.diff(mask.astype(np.int8))
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)

    if mask[0]:
        starts.insert(0, 0)
    if mask[-1]:
        ends.append(n)

    first = True
    for s, e in zip(starts, ends):
        if e <= s:
            continue
        left = time[s]
        right = time[e - 1]
        if e < n:
            right = time[e]
        ax.axvspan(
            left,
            right,
            color="0.75",
            alpha=alpha,
            linewidth=0,
            label=label if first else None,
        )
        first = False


# ===============================================================
# 通用绘图
# ===============================================================
def plot_tracking(
    ax,
    ds,
    actual_label,
    target_label,
    ylabel,
    title,
    unit,
    actual_color,
    target_color,
):
    if len(ds["data"]) == 0:
        return None

    ax.plot(
        ds["time"],
        ds["data"],
        label=actual_label,
        color=actual_color,
        lw=1.6,
    )
    ax.plot(
        ds["target_time"],
        ds["target"],
        label=target_label,
        color=target_color,
        linestyle="--",
        lw=1.8,
    )

    ax.set_ylabel(ylabel, fontsize=11, fontweight="bold")
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    auto_ylim(ax, ds["time"], ds["data"])

    if ARGS.stats:
        return add_stats_box(
            ax,
            ds["time"],
            ds["data"],
            ds["target"],
            unit,
        )
    return None


# ===============================================================
# Balance
# ===============================================================
def panel_speed(ax, datasets):
    return plot_tracking(
        ax,
        datasets["speed"],
        "Actual Speed",
        "Target Speed",
        "Speed (m/s)",
        "Robot Forward Speed Tracking",
        "m/s",
        "#1f77b4",
        "#ff7f0e",
    )


def panel_pitch(ax, datasets):
    return plot_tracking(
        ax,
        datasets["angle"],
        "Actual Pitch",
        "Target Pitch",
        "Pitch (rad)",
        "Chassis Pitch Balance Tracking",
        "rad",
        "#2ca02c",
        "#d62728",
    )


def panel_gyro(ax, datasets):
    return plot_tracking(
        ax,
        datasets["gyro"],
        "Actual Pitch Rate",
        "Target Pitch Rate",
        "Pitch Rate (rad/s)",
        "Pitch Angular Velocity Response",
        "rad/s",
        "#9467bd",
        "#8c564b",
    )


def panel_wheel_current(ax, datasets):
    current = datasets["current"]
    if len(current["left"]) == 0:
        return

    ax.plot(
        current["tl"],
        current["left"],
        label="Left Wheel Current",
        color="#17becf",
        lw=1.5,
    )
    ax.plot(
        current["tr"],
        -current["right"],
        label="Right Wheel Current (Inverted)",
        color="#e377c2",
        lw=1.5,
    )

    ax.set_ylabel("Current (mA)", fontsize=11, fontweight="bold")
    ax.set_title("Wheel Hub Motor Output Current", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)
    auto_ylim(ax, current["tl"], current["left"])


# ===============================================================
# Torque / Temperature / Legs
# ===============================================================
def panel_torque(ax, datasets):
    motor = datasets["motor"]
    if len(motor.get("time", [])) == 0:
        return

    labels = {
        "motor1": "Motor ID 1 (Left Hip)",
        "motor2": "Motor 2 (Left Knee)",
        "motor3": "Motor 3 (Right Hip)",
        "motor4": "Motor 4 (Right Knee - Inverted)",
    }
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]

    plotted = []
    for i in range(1, 5):
        key = f"motor{i}"
        if len(motor.get(key, np.array([]))) > 0:
            y = -motor[key] if key == "motor4" else motor[key]
            ax.plot(
                motor["time"],
                y,
                label=labels[key],
                color=colors[i - 1],
                lw=1.6,
            )
            plotted.append(y)

    ax.set_ylabel("Torque (Nm)", fontsize=11, fontweight="bold")
    ax.set_title("Joint Motors Feedback Torque", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", ncol=2, framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    if plotted:
        all_time = np.concatenate([motor["time"]] * len(plotted))
        auto_ylim(ax, all_time, np.concatenate(plotted))


def plot_temperature(
    ax,
    temp,
    suffix,
    title,
    ylabel,
    derating,
    shutdown,
    derating_label,
    shutdown_label,
):
    colors = {
        "hip_left": "#e41a1c",
        "knee_left": "#377eb8",
        "hip_right": "#4daf4a",
        "knee_right": "#984ea3",
    }
    names = {
        "hip_left": "Left Hip",
        "knee_left": "Left Knee",
        "hip_right": "Right Hip",
        "knee_right": "Right Knee",
    }

    if len(temp.get("time", [])) > 0:
        for key in colors:
            values = temp.get(f"{key}_{suffix}", np.array([]))
            if len(values) > 0:
                ax.plot(
                    temp["time"],
                    values,
                    label=names[key],
                    color=colors[key],
                    lw=1.6,
                )

    ax.axhline(
        derating,
        color="orange",
        linestyle="--",
        lw=1.4,
        label=derating_label,
    )
    ax.axhline(
        shutdown,
        color="red",
        linestyle="--",
        lw=1.4,
        label=shutdown_label,
    )

    ax.set_ylabel(ylabel, fontsize=11, fontweight="bold")
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)


def panel_motor_temp(ax, datasets):
    plot_temperature(
        ax,
        datasets["temperature"],
        "motor",
        "Joint Motor Winding Temperature",
        "Motor Temp (C)",
        105.0,
        120.0,
        "Motor Derating Start (105 C)",
        "Motor Shutdown (120 C)",
    )


def panel_mos_temp(ax, datasets):
    plot_temperature(
        ax,
        datasets["temperature"],
        "mos",
        "Joint Motor Driver MOS Temperature",
        "MOS Temp (C)",
        95.0,
        110.0,
        "MOS Derating Start (95 C)",
        "MOS Shutdown (110 C)",
    )


def panel_leg_height(ax, datasets):
    leg = datasets["leg_diag"]
    if len(leg.get("time", [])) == 0:
        return

    series = []
    specs = [
        ("height", "Center Height", "#000000", 1.8),
        ("left_height", "Left Leg Height", "#377eb8", 1.4),
        ("right_height", "Right Leg Height", "#984ea3", 1.4),
    ]

    for key, label, color, lw in specs:
        if len(leg.get(key, np.array([]))) > 0:
            ax.plot(
                leg["time"],
                leg[key],
                label=label,
                color=color,
                lw=lw,
            )
            series.append(leg[key])

    ax.set_ylabel("Leg Height (m)", fontsize=11, fontweight="bold")
    ax.set_title("Virtual Leg Height", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    if series:
        all_time = np.concatenate([leg["time"]] * len(series))
        auto_ylim(ax, all_time, np.concatenate(series))


def panel_knee_current(ax, datasets):
    leg = datasets["leg_diag"]
    if len(leg.get("time", [])) == 0:
        return

    series = []

    if len(leg.get("knee_left_current", np.array([]))) > 0:
        ax.plot(
            leg["time"],
            leg["knee_left_current"],
            label="Left Knee Current",
            color="#377eb8",
            lw=1.6,
        )
        series.append(leg["knee_left_current"])

    if len(leg.get("knee_right_current", np.array([]))) > 0:
        ax.plot(
            leg["time"],
            -leg["knee_right_current"],
            label="Right Knee Current (Inverted)",
            color="#984ea3",
            lw=1.6,
        )
        series.append(-leg["knee_right_current"])

    ax.set_ylabel("Current (A)", fontsize=11, fontweight="bold")
    ax.set_title("Knee Motor Current Feedback", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    if series:
        all_time = np.concatenate([leg["time"]] * len(series))
        auto_ylim(ax, all_time, np.concatenate(series))


# ===============================================================
# Steering
# ===============================================================
def panel_yaw_heading(ax, datasets):
    """
    第一幅转向图：
      只看真正的直线行驶阶段；
      只绘制直线航向误差与 0° 基准线。
    """
    yaw = datasets["yaw"]
    extra = datasets["steering"]

    if len(yaw["data"]) == 0 or len(extra["time"]) == 0:
        return None

    n = min(
        len(yaw["data"]),
        len(yaw["target"]),
        len(yaw["time"]),
        len(extra["straight_mask"]),
        len(extra["manual_mask"]),
    )
    if n == 0:
        return None

    time = yaw["time"][:n]
    actual = yaw["data"][:n]
    target = yaw["target"][:n]

    straight_mask = extra["straight_mask"][:n]
    manual_mask = extra["manual_mask"][:n]

    # wrap 到 [-pi, pi]；不使用 unwrap 后直接相减，避免 ±pi 跨界误差
    error_rad = np.arctan2(
        np.sin(target - actual),
        np.cos(target - actual),
    )
    error_deg = np.degrees(error_rad)

    # 只在直线行驶段显示误差；转向/停车时断线
    display_error = np.where(straight_mask, error_deg, np.nan)

    # 第一张图只保留：
    # 1) 直线行驶时的航向误差；
    # 2) 0° 基准线。
    ax.axhline(
        0.0,
        color="#d62728",
        linestyle="--",
        lw=1.5,
        label="Zero Error",
    )

    ax.plot(
        time,
        display_error,
        color="#2ca02c",
        lw=1.8,
        label="Straight-Line Heading Error",
    )

    ax.set_ylabel("Heading Error (deg)", fontsize=11, fontweight="bold")
    ax.set_title("Straight-Line Heading Error", fontsize=13, fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.9)

    # 只根据直线有效样本决定对称纵轴；最少显示 ±5 deg
    window_mask = _window_mask(time)
    valid = straight_mask & window_mask & np.isfinite(error_deg)
    if np.any(valid):
        max_abs = float(np.max(np.abs(error_deg[valid])))
        y_lim = max(5.0, np.ceil(max_abs / 5.0) * 5.0)
        ax.set_ylim(-y_lim, y_lim)
    else:
        ax.set_ylim(-5.0, 5.0)

    if ARGS.stats:
        return add_scalar_stats_box(
            ax,
            time,
            error_deg,
            straight_mask,
            "deg",
        )

    return None


def panel_yaw_rate(ax, datasets):
    result = plot_tracking(
        ax,
        datasets["yaw_rate"],
        "Actual Yaw Rate",
        "Target Yaw Rate",
        "Yaw Rate (rad/s)",
        "Yaw Rate Tracking",
        "rad/s",
        "#9467bd",
        "#8c564b",
    )

    extra = datasets.get("steering", {})
    if len(extra.get("time", [])) > 0:
        shade_true_regions(
            ax,
            extra["time"],
            extra["manual_mask"],
            label=None,
            alpha=0.10,
        )

    return result


def panel_yaw_diff(ax, datasets):
    extra = datasets["steering"]
    if len(extra["time"]) == 0:
        return

    ax.plot(
        extra["time"],
        extra["diff"],
        label="Yaw Diff Current",
        color="#17becf",
        lw=1.5,
    )

    shade_true_regions(
        ax,
        extra["time"],
        extra["manual_mask"],
        label=None,
        alpha=0.10,
    )

    ax.axhline(0.0, color="0.45", linestyle=":", lw=1.0)
    ax.set_ylabel("Current (mA)", fontsize=11, fontweight="bold")
    ax.set_title("Yaw Differential Wheel Current", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)
    auto_ylim(ax, extra["time"], extra["diff"])


# ===============================================================
# 命令行
# ===============================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="bbot 数据绘图"
    )

    parser.add_argument(
        "--dir",
        default=os.path.dirname(os.path.abspath(__file__)),
        help="数据目录（默认：本脚本所在目录）",
    )
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="显示窗口起点（秒）",
    )
    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="显示窗口终点（秒）",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="输出 PNG 文件名（默认 plot_data.png）",
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="跟踪子图叠加统计框",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="只保存 PNG，不弹出窗口",
    )

    group = parser.add_argument_group(
        "子图分组（不带任何分组参数时默认 --balance）"
    )
    group.add_argument(
        "--balance",
        action="store_true",
        help="速度/俯仰/角速度跟踪 + 轮毂电流",
    )
    group.add_argument(
        "--torque",
        action="store_true",
        help="四关节力矩",
    )
    group.add_argument(
        "--temp",
        action="store_true",
        help="关节温度",
    )
    group.add_argument(
        "--legs",
        action="store_true",
        help="腿高/膝关节电流",
    )
    group.add_argument(
        "--steering",
        action="store_true",
        help="转向环（直线航向误差/Yaw-rate/差动电流）",
    )
    group.add_argument(
        "--all",
        action="store_true",
        help="全部子图",
    )

    return parser.parse_args()


# ===============================================================
# 主流程
# ===============================================================
def main():
    global ARGS
    ARGS = parse_args()
    base = ARGS.dir

    plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial"]
    plt.rcParams["axes.unicode_minus"] = False

    any_group = (
        ARGS.balance
        or ARGS.torque
        or ARGS.temp
        or ARGS.legs
        or ARGS.steering
        or ARGS.all
    )

    enable = {
        "balance": ARGS.balance or ARGS.all or not any_group,
        "torque": ARGS.torque or ARGS.all,
        "temp": ARGS.temp or ARGS.all,
        "legs": ARGS.legs or ARGS.all,
        "steering": ARGS.steering or ARGS.all,
    }

    datasets = {}

    if enable["balance"]:
        datasets["speed"] = process_dataset(
            base,
            DATASET_FILES["speed"],
            "speed",
        )
        datasets["angle"] = process_dataset(
            base,
            DATASET_FILES["angle"],
            "pitch",
        )
        datasets["gyro"] = process_dataset(
            base,
            DATASET_FILES["gyro"],
            "gyro",
        )
        datasets["current"] = process_current(base)

    if enable["torque"]:
        datasets["motor"] = process_motor(base)

    if enable["temp"]:
        datasets["temperature"] = process_channel_group(
            base,
            TEMPERATURE_FILES,
            "joint temperature",
        )

    if enable["legs"]:
        datasets["leg_diag"] = process_channel_group(
            base,
            LEG_DIAG_FILES,
            "leg diagnostic",
        )
        print_knee_kt_summary(datasets["leg_diag"])

    if enable["steering"]:
        datasets["yaw"] = process_dataset(
            base,
            STEERING_FILES["yaw"],
            "yaw",
        )
        datasets["yaw_rate"] = process_dataset(
            base,
            STEERING_FILES["yaw_rate"],
            "yaw rate",
        )
        datasets["steering"] = process_steering_extras(base)

    panels = []

    if enable["balance"]:
        panels += [
            ("Speed Tracking", panel_speed, "speed"),
            ("Pitch Tracking", panel_pitch, "angle"),
            ("Pitch Rate", panel_gyro, "gyro"),
            ("Wheel Current", panel_wheel_current, "current"),
        ]

    if enable["torque"]:
        panels.append(
            ("Joint Torque", panel_torque, "motor")
        )

    if enable["temp"]:
        panels += [
            ("Motor Temp", panel_motor_temp, "temperature"),
            ("MOS Temp", panel_mos_temp, "temperature"),
        ]

    if enable["legs"]:
        panels += [
            ("Leg Height", panel_leg_height, "leg_diag"),
            ("Knee Current", panel_knee_current, "leg_diag"),
        ]

    if enable["steering"]:
        panels += [
            ("Straight Heading Error", panel_yaw_heading, "yaw"),
            ("Yaw Rate", panel_yaw_rate, "yaw_rate"),
            ("Yaw Diff Current", panel_yaw_diff, "steering"),
        ]

    def has_data(key):
        ds = datasets.get(key, {})
        if key == "current":
            return len(ds.get("left", np.array([]))) > 0
        return len(ds.get("time", np.array([]))) > 0

    active = []
    for name, fn, key in panels:
        if has_data(key):
            active.append((name, fn, key))
        else:
            print(f"Skipped panel '{name}': 数据为空")

    if not active:
        print("没有可绘制的数据，请确认数据目录中存在 txt 日志文件。")
        sys.exit(1)

    stats_results = {}

    fig, axes = plt.subplots(
        len(active),
        1,
        figsize=(16, 3.0 * len(active) + 1.0),
        sharex=True,
        layout="constrained",
    )

    if len(active) == 1:
        axes = [axes]

    for ax, (name, fn, key) in zip(axes, active):
        result = fn(ax, datasets)
        if result is not None:
            stats_results[name] = result

    spans = []
    for ds in datasets.values():
        t = ds.get("time", np.array([]))
        if len(t) > 0:
            spans.append((t[0], t[-1]))

    t_min = min(s[0] for s in spans) if spans else 0.0
    t_max = max(s[1] for s in spans) if spans else 1.0

    t_start = ARGS.start if ARGS.start is not None else t_min
    t_end = ARGS.end if ARGS.end is not None else t_max

    for ax in axes:
        ax.set_xlim(t_start, t_end)
        step = nice_tick_step(max(t_end - t_start, 1e-6))
        ax.xaxis.set_major_locator(MultipleLocator(step))
        ax.xaxis.set_minor_locator(MultipleLocator(step / 2.0))

    axes[-1].set_xlabel(
        "Time (s)",
        fontsize=11,
        fontweight="bold",
    )

    output = os.path.join(
        base,
        ARGS.out or "plot_data.png",
    )
    fig.savefig(output, dpi=300)
    print("Saved figure:", output)

    if stats_results:
        print(
            "\n统计结果（窗口 %.2f ~ %.2f s）："
            % (t_start, t_end)
        )
        for name, (rmse, mae, max_error) in stats_results.items():
            print(
                f"{name:<22} "
                f"RMSE={rmse:.4f}, "
                f"MAE={mae:.4f}, "
                f"MaxE={max_error:.4f}"
            )

    if not ARGS.no_show:
        plt.show()


if __name__ == "__main__":
    main()
