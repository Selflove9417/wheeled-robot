#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bbot 数据绘图统一脚本（合并原 data_plots.py ~ data_plots5.py）。

子图分组（可任意组合，不带任何分组参数时默认 --balance）：
  --balance  速度/俯仰/角速度跟踪 + 轮毂电流（4 个子图）
  --torque   四关节力矩反馈（1 个子图）
  --temp     关节电机线圈温度 + MOS 温度（2 个子图）
  --legs     虚拟腿高 + 膝关节电流反馈（2 个子图）
  --all      以上全部

常用选项：
  --stats          在跟踪子图上叠加 RMSE/MAE/MaxE 统计框，并打印到终端
  --start / --end  时间窗口（秒），默认覆盖全部数据
  --dir            数据目录，默认本脚本所在目录
  --out            输出 PNG 文件名，默认 plot_data.png

示例：
  python3 plot_data.py --all --stats
  python3 plot_data.py --balance --stats --start 10 --end 40
  python3 plot_data.py --temp --legs --out run1.png
"""
import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

SAMPLE_DT = 0.005  # 控制频率 200Hz，时间戳退化时的采样周期假设

# ===============================================================
# 数据文件清单（相对于数据目录）
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

ARGS = None  # 由 main() 填充，供 auto_ylim 使用


# ===============================================================
# 数据读取与对齐
# ===============================================================
def read_data(file_path, name):
    try:
        with open(file_path, "r") as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f"Loaded {name}: {len(data)} points")
        return np.array(data)
    except Exception as e:
        print(f"Failed to read {name}: {e}")
        return np.array([])


def relative_time(t, n):
    """真实时间戳转相对时间；时间戳退化时回退到采样序号。"""
    if len(t) >= n and len(t) > 1 and (t[n - 1] - t[0]) > 0.1:
        return t[:n] - t[0]
    return np.arange(n) * SAMPLE_DT


def process_dataset(base, files, label):
    d = {k: os.path.join(base, v) for k, v in files.items()}
    data = read_data(d["data"], label + " actual")
    target = read_data(d["target"], label + " target")
    t = read_data(d["timestamp"], label + " time")
    tt = read_data(d["target_timestamp"], label + " target time")

    n = min(len(data), len(target), len(t), len(tt))
    if n == 0:
        return {"data": np.array([]), "target": np.array([]),
                "time": np.array([]), "target_time": np.array([])}

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
        return {"left": np.array([]), "right": np.array([]),
                "tl": np.array([]), "tr": np.array([])}

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

    t = read_data(os.path.join(base, TORQUE_FILES["timestamp"]),
                  "joint torque time")

    valid_lens = [len(v) for v in motor.values() if len(v) > 0]
    n = min(valid_lens) if valid_lens else 0
    if n == 0:
        motor["time"] = np.array([])
        return motor

    for key in motor:
        if key == "time":
            continue
        motor[key] = motor[key][:n] if len(motor[key]) >= n else np.zeros(n)

    # 力矩与关节温度共用同一个反馈时间戳文件
    motor["time"] = relative_time(t, n)
    return motor


def process_channel_group(base, files, group_name):
    """通用处理：N 个数据通道 + 1 个共享时间戳文件（温度 / 腿部诊断）。"""
    group = {}
    for key, name in files.items():
        if key == "timestamp":
            continue
        group[key] = read_data(os.path.join(base, name), key.replace("_", " "))

    t = read_data(os.path.join(base, files["timestamp"]), group_name + " time")
    if len(t) == 0:
        group["time"] = np.array([])
        return group

    valid_lens = [len(t)]
    for value in group.values():
        if len(value) > 0:
            valid_lens.append(len(value))
    n = min(valid_lens)

    t = t[:n]
    for key in list(group.keys()):
        group[key] = group[key][:n] if len(group[key]) >= n else np.full(n, np.nan)

    group["time"] = relative_time(t, n)
    return group


# ===============================================================
# 统计与坐标辅助
# ===============================================================
def add_stats_box(ax, time, data, target, unit):
    """在时间窗口内统计跟踪误差，叠加到子图右上角并返回统计值。"""
    if len(data) == 0 or len(data) != len(target):
        return None

    error = data - target
    if (ARGS.start is not None or ARGS.end is not None) and len(time) == len(data):
        lo = ARGS.start if ARGS.start is not None else -np.inf
        hi = ARGS.end if ARGS.end is not None else np.inf
        mask = (time >= lo) & (time <= hi)
        if np.sum(mask) > 0:
            error = error[mask]

    error = error[np.isfinite(error)]
    if len(error) == 0:
        return None

    rmse = np.sqrt(np.mean(error**2))
    mae = np.mean(np.abs(error))
    max_error = np.max(np.abs(error))

    text = (
        f"RMSE: {rmse:.4f} {unit}\n"
        f"MAE: {mae:.4f} {unit}\n"
        f"MaxE: {max_error:.4f} {unit}\n"
        f"N: {len(error)}"
    )
    ax.text(0.98, 0.98, text, transform=ax.transAxes, fontsize=9,
            verticalalignment="top", horizontalalignment="right",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
    return rmse, mae, max_error


def auto_ylim(ax, x, y):
    if len(x) == 0 or len(y) == 0:
        return
    if len(x) == len(y):
        lo = ARGS.start if ARGS.start is not None else -np.inf
        hi = ARGS.end if ARGS.end is not None else np.inf
        mask = (x >= lo) & (x <= hi)
        window = y[mask] if np.sum(mask) > 0 else y
    else:
        window = y

    window = np.asarray(window)
    window = window[np.isfinite(window)]
    if len(window) == 0:
        return

    ymin = np.min(window)
    ymax = np.max(window)
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
    for key, label in (("knee_left_kt", "Left"), ("knee_right_kt", "Right")):
        kt = leg_diag.get(key, np.array([]))
        valid = kt[np.isfinite(kt)] if len(kt) > 0 else kt
        if len(valid) > 0:
            print(f"{label} Knee Kt median: {np.median(valid):.4f} Nm/A")


# ===============================================================
# 子图绘制函数：每个函数负责一个子图
# ===============================================================
def plot_tracking(ax, ds, actual_label, target_label, ylabel, title, unit,
                  actual_color, target_color):
    if len(ds["data"]) == 0:
        return None
    ax.plot(ds["time"], ds["data"], label=actual_label,
            color=actual_color, lw=1.6)
    ax.plot(ds["target_time"], ds["target"], label=target_label,
            color=target_color, linestyle="--", lw=1.8)
    ax.set_ylabel(ylabel, fontsize=11, fontweight="bold")
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)
    auto_ylim(ax, ds["time"], ds["data"])
    if ARGS.stats:
        return add_stats_box(ax, ds["time"], ds["data"], ds["target"], unit)
    return None


def panel_speed(ax, datasets):
    return plot_tracking(
        ax, datasets["speed"],
        "Actual Speed", "Target Speed",
        "Speed (m/s)", "Robot Forward Speed Tracking", "m/s",
        "#1f77b4", "#ff7f0e")


def panel_pitch(ax, datasets):
    return plot_tracking(
        ax, datasets["angle"],
        "Actual Pitch", "Target Pitch",
        "Pitch (rad)", "Chassis Pitch Balance Tracking", "rad",
        "#2ca02c", "#d62728")


def panel_gyro(ax, datasets):
    return plot_tracking(
        ax, datasets["gyro"],
        "Actual Pitch Rate", "Target Pitch Rate",
        "Pitch Rate (rad/s)", "Pitch Angular Velocity Response", "rad/s",
        "#9467bd", "#8c564b")


def panel_wheel_current(ax, datasets):
    current = datasets["current"]
    if len(current["left"]) == 0:
        return
    ax.plot(current["tl"], current["left"], label="Left Wheel Current",
            color="#17becf", lw=1.5)
    ax.plot(current["tr"], -current["right"],
            label="Right Wheel Current (Inverted)", color="#e377c2", lw=1.5)
    ax.set_ylabel("Current (mA)", fontsize=11, fontweight="bold")
    ax.set_title("Wheel Hub Motor Output Current", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)
    auto_ylim(ax, current["tl"], current["left"])


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
            ax.plot(motor["time"], y, label=labels[key],
                    color=colors[i - 1], lw=1.6)
            plotted.append(y)

    ax.set_ylabel("Torque (Nm)", fontsize=11, fontweight="bold")
    ax.set_title("Joint Motors Feedback Torque", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", ncol=2, framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    if plotted:
        all_time = np.concatenate([motor["time"]] * len(plotted))
        auto_ylim(ax, all_time, np.concatenate(plotted))


def plot_temperature(ax, temp, suffix, title, ylabel,
                     derating, shutdown, derating_label, shutdown_label):
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
                ax.plot(temp["time"], values, label=names[key],
                        color=colors[key], lw=1.6)

    ax.axhline(derating, color="orange", linestyle="--", lw=1.4,
               label=derating_label)
    ax.axhline(shutdown, color="red", linestyle="--", lw=1.4,
               label=shutdown_label)
    ax.set_ylabel(ylabel, fontsize=11, fontweight="bold")
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)


def panel_motor_temp(ax, datasets):
    plot_temperature(
        ax, datasets["temperature"], "motor",
        "Joint Motor Winding Temperature", "Motor Temp (C)",
        105.0, 120.0,
        "Motor Derating Start (105 C)", "Motor Shutdown (120 C)")


def panel_mos_temp(ax, datasets):
    plot_temperature(
        ax, datasets["temperature"], "mos",
        "Joint Motor Driver MOS Temperature", "MOS Temp (C)",
        95.0, 110.0,
        "MOS Derating Start (95 C)", "MOS Shutdown (110 C)")


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
            ax.plot(leg["time"], leg[key], label=label, color=color, lw=lw)
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
        ax.plot(leg["time"], leg["knee_left_current"],
                label="Left Knee Current", color="#377eb8", lw=1.6)
        series.append(leg["knee_left_current"])
    if len(leg.get("knee_right_current", np.array([]))) > 0:
        ax.plot(leg["time"], -leg["knee_right_current"],
                label="Right Knee Current (Inverted)", color="#984ea3", lw=1.6)
        series.append(-leg["knee_right_current"])

    ax.set_xlabel("Time (s)", fontsize=11, fontweight="bold")
    ax.set_ylabel("Current (A)", fontsize=11, fontweight="bold")
    ax.set_title("Knee Motor Current Feedback", fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    if series:
        all_time = np.concatenate([leg["time"]] * len(series))
        auto_ylim(ax, all_time, np.concatenate(series))


# ===============================================================
# 主流程
# ===============================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="bbot 数据绘图（合并原 data_plots.py ~ data_plots5.py）")
    parser.add_argument("--dir", default=os.path.dirname(os.path.abspath(__file__)),
                        help="数据目录（默认：本脚本所在目录）")
    parser.add_argument("--start", type=float, default=None,
                        help="显示窗口起点（秒），默认覆盖全部数据")
    parser.add_argument("--end", type=float, default=None,
                        help="显示窗口终点（秒），默认覆盖全部数据")
    parser.add_argument("--out", default=None,
                        help="输出 PNG 文件名（默认 plot_data.png）")
    parser.add_argument("--stats", action="store_true",
                        help="跟踪子图叠加 RMSE/MAE/MaxE 统计框")
    parser.add_argument("--no-show", action="store_true",
                        help="只保存 PNG，不弹出窗口")

    group = parser.add_argument_group("子图分组（不带任何分组参数时默认 --balance）")
    group.add_argument("--balance", action="store_true",
                       help="速度/俯仰/角速度跟踪 + 轮毂电流")
    group.add_argument("--torque", action="store_true", help="四关节力矩")
    group.add_argument("--temp", action="store_true", help="关节温度")
    group.add_argument("--legs", action="store_true", help="腿高/膝关节电流")
    group.add_argument("--all", action="store_true", help="全部子图")
    return parser.parse_args()


def main():
    global ARGS
    ARGS = parse_args()
    base = ARGS.dir

    plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial"]
    plt.rcParams["axes.unicode_minus"] = False

    any_group = ARGS.balance or ARGS.torque or ARGS.temp or ARGS.legs or ARGS.all
    enable = {
        "balance": ARGS.balance or ARGS.all or not any_group,
        "torque": ARGS.torque or ARGS.all,
        "temp": ARGS.temp or ARGS.all,
        "legs": ARGS.legs or ARGS.all,
    }

    # 只加载需要的组
    datasets = {}
    if enable["balance"]:
        datasets["speed"] = process_dataset(base, DATASET_FILES["speed"], "speed")
        datasets["angle"] = process_dataset(base, DATASET_FILES["angle"], "pitch")
        datasets["gyro"] = process_dataset(base, DATASET_FILES["gyro"], "gyro")
        datasets["current"] = process_current(base)
    if enable["torque"]:
        datasets["motor"] = process_motor(base)
    if enable["temp"]:
        datasets["temperature"] = process_channel_group(
            base, TEMPERATURE_FILES, "joint temperature")
    if enable["legs"]:
        datasets["leg_diag"] = process_channel_group(
            base, LEG_DIAG_FILES, "leg diagnostic")
        print_knee_kt_summary(datasets["leg_diag"])

    panels = []
    if enable["balance"]:
        panels += [("Speed Tracking", panel_speed, "speed"),
                   ("Pitch Tracking", panel_pitch, "angle"),
                   ("Pitch Rate", panel_gyro, "gyro"),
                   ("Wheel Current", panel_wheel_current, "current")]
    if enable["torque"]:
        panels.append(("Joint Torque", panel_torque, "motor"))
    if enable["temp"]:
        panels += [("Motor Temp", panel_motor_temp, "temperature"),
                   ("MOS Temp", panel_mos_temp, "temperature")]
    if enable["legs"]:
        panels += [("Leg Height", panel_leg_height, "leg_diag"),
                   ("Knee Current", panel_knee_current, "leg_diag")]

    # 数据为空的组直接跳过对应子图
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
    fig, axes = plt.subplots(len(active), 1,
                             figsize=(16, 3.0 * len(active) + 1.0),
                             sharex=True, layout="constrained")
    if len(active) == 1:
        axes = [axes]
    for ax, (name, fn, key) in zip(axes, active):
        result = fn(ax, datasets)
        if result is not None:
            stats_results[name] = result

    # 时间窗口：未指定时取所有已加载数据的时间范围
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
        step = nice_tick_step(t_end - t_start)
        ax.xaxis.set_major_locator(MultipleLocator(step))
        ax.xaxis.set_minor_locator(MultipleLocator(step / 2.0))
    axes[-1].set_xlabel("Time (s)", fontsize=11, fontweight="bold")

    output = os.path.join(base, ARGS.out or "plot_data.png")
    fig.savefig(output, dpi=300)
    print("Saved figure:", output)

    if stats_results:
        print("\n统计结果（窗口 %.2f ~ %.2f s）：" % (t_start, t_end))
        for name, (rmse, mae, max_error) in stats_results.items():
            print(f"{name:<14} RMSE={rmse:.4f}, MAE={mae:.4f}, MaxE={max_error:.4f}")

    if not ARGS.no_show:
        plt.show()


if __name__ == "__main__":
    main()
