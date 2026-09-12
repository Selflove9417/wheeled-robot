#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.ticker import MultipleLocator  # 新增导入

# 目标数据存储路径
base_path = "/home/robot/bbot_real/src/bbot_real/src/data_logs/"

angle_files = {
    "data": base_path + "angle_data.txt",
    "target": base_path + "target_angle_data.txt",
    "timestamp": base_path + "timestamp_angle.txt",
    "target_timestamp": base_path + "timestamp_target_angle.txt",
}

speed_files = {
    "data": base_path + "speed_data.txt",
    "target": base_path + "target_speed_data.txt",
    "timestamp": base_path + "timestamp_speed.txt",
    "target_timestamp": base_path + "timestamp_target_speed.txt",
}

gyro_files = {
    "data": base_path + "gyro_data.txt",
    "target": base_path + "target_gyro_data.txt",
    "timestamp": base_path + "timestamp_gyro.txt",
    "target_timestamp": base_path + "timestamp_target_gyro.txt",
}

current_files = {
    "left": base_path + "left_current_data.txt",
    "right": base_path + "right_current_data.txt",
    "timestamp_left": base_path + "timestamp_left_current.txt",
    "timestamp_right": base_path + "timestamp_right_current.txt",
}


def check_files_exist(files_dict, data_type):
    required_files = [
        (files_dict["data"], f"{data_type}_data"),
        (files_dict["target"], f"target_{data_type}_data"),
        (files_dict["timestamp"], f"timestamp_{data_type}"),
        (files_dict["target_timestamp"], f"timestamp_target_{data_type}"),
    ]

    ok = True
    for file_path, name in required_files:
        if not os.path.exists(file_path):
            print(f"缺少 {name}: {file_path}")
            ok = False

    return ok


def read_data(file_path, data_name):
    try:
        with open(file_path, "r") as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f"{data_name}: {len(data)} 个数据点")
        return data
    except Exception as e:
        print(f"读取 {data_name} 失败: {e}")
        return []


def check_current_files_exist():
    ok = True
    required_files = [
        (current_files["left"], "left_current_data"),
        (current_files["right"], "right_current_data"),
        (current_files["timestamp_left"], "timestamp_left_current"),
        (current_files["timestamp_right"], "timestamp_right_current"),
    ]
    for file_path, name in required_files:
        if not os.path.exists(file_path):
            print(f"缺少 {name}: {file_path}")
            ok = False
    return ok


def process_current_dataset():
    left = np.array(read_data(current_files["left"], "下发电流(左)"))
    right = np.array(read_data(current_files["right"], "下发电流(右)"))
    t_left = np.array(read_data(current_files["timestamp_left"], "下发电流时间戳(左)"))
    t_right = np.array(read_data(current_files["timestamp_right"], "下发电流时间戳(右)"))

    n = min(len(left), len(right), len(t_left), len(t_right))
    if n == 0:
        return None

    left = left[:n]
    right = right[:n]
    t_left = t_left[:n]
    t_right = t_right[:n]

    rel_left = t_left - t_left[0]
    rel_right = t_right - t_right[0]
    if np.ptp(rel_left) < 1e-6 or np.ptp(rel_right) < 1e-6:
        sample_dt = 0.005
        rel_left = np.arange(n, dtype=float) * sample_dt
        rel_right = np.arange(n, dtype=float) * sample_dt
        print("下发电流: 时间戳未包含有效毫秒信息，改用采样序号作为横轴")

    return {
        "left": left,
        "right": right,
        "relative_left": rel_left,
        "relative_right": rel_right,
    }


def process_dataset(files_dict, label):
    data = read_data(files_dict["data"], f"{label}实际值")
    target_data = read_data(files_dict["target"], f"{label}目标值")
    timestamps = read_data(files_dict["timestamp"], f"{label}时间戳")
    target_timestamps = read_data(files_dict["target_timestamp"], f"{label}目标时间戳")

    min_len = min(len(data), len(target_data), len(timestamps), len(target_timestamps))

    if min_len == 0:
        return None

    data = np.array(data[:min_len])
    target_data = np.array(target_data[:min_len])
    timestamps = np.array(timestamps[:min_len])
    target_timestamps = np.array(target_timestamps[:min_len])

    relative_time = timestamps - timestamps[0]
    target_relative_time = target_timestamps - target_timestamps[0]

    if np.ptp(relative_time) < 1e-6 or np.ptp(target_relative_time) < 1e-6:
        sample_dt = 0.005
        relative_time = np.arange(len(data), dtype=float) * sample_dt
        target_relative_time = np.arange(len(target_data), dtype=float) * sample_dt
        print(f"{label}: 时间戳未包含有效毫秒信息，改用采样序号作为横轴")

    return {
        "data": data,
        "target_data": target_data,
        "relative_time": relative_time,
        "target_relative_time": target_relative_time,
        "label": label,
    }


def add_stats_box(ax, data, target_data, unit):
    error = data - target_data
    rmse = np.sqrt(np.mean(error**2))
    mae = np.mean(np.abs(error))
    max_error = np.max(np.abs(error))

    text = (
        f"RMSE: {rmse:.4f} {unit}\n"
        f"MAE: {mae:.4f} {unit}\n"
        f"MaxE: {max_error:.4f} {unit}\n"
        f"N: {len(data)}"
    )

    ax.text(
        0.98,
        0.98,
        text,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    return rmse, mae, max_error
def auto_ylim(ax, x, y, start, end, margin=0.1):
    """
    根据当前时间窗口自动调整Y轴范围

    margin:
        上下额外留出的比例
    """

    mask = (x >= start) & (x <= end)

    if np.sum(mask) == 0:
        return

    y_window = y[mask]

    ymin = np.min(y_window)
    ymax = np.max(y_window)

    value_range = ymax - ymin

    # 防止数据几乎不变化导致ylim一样
    if value_range < 1e-6:
        value_range = max(abs(ymax), 1.0)

    offset = value_range * margin

    ax.set_ylim(
        ymin - offset,
        ymax + offset
    )

def main():
    TIME_START = 0
    TIME_END = 15
    print("=" * 60)
    print("Balance Controller Data Plot")
    print("=" * 60)

    all_ok = True
    all_ok &= check_files_exist(angle_files, "angle")
    all_ok &= check_files_exist(speed_files, "speed")
    all_ok &= check_files_exist(gyro_files, "gyro")

    has_current = check_current_files_exist()

    if not all_ok:
        print("数据文件不完整，请先运行控制代码生成 txt 数据。")
        return

    angle_data = process_dataset(angle_files, "angle")
    speed_data = process_dataset(speed_files, "speed")
    gyro_data = process_dataset(gyro_files, "gyro")

    if angle_data is None or speed_data is None or gyro_data is None:
        print("数据为空，无法绘图。")
        return

    current_data = process_current_dataset() if has_current else None

    if current_data is not None:
        fig, axes = plt.subplots(4, 1, figsize=(16, 16), sharex=True)
    else:
        fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)

    # ---------- 修改 1：交换前两个子图的绘制顺序 ----------
    # 原 ax1 = axes[0] 画角度，ax2 = axes[1] 画速度
    # 现 ax1 画速度，ax2 画角度
    ax1 = axes[0]  # 速度
    ax2 = axes[1]  # 角度
    ax3 = axes[2]  # 角速率

    # 绘制速度 (第一个子图)
    ax1.plot(speed_data["relative_time"], speed_data["data"], linewidth=2, label="current_speed")
    ax1.plot(speed_data["target_relative_time"], speed_data["target_data"], linewidth=2, label="target_speed")
    ax1.set_ylabel("m/s")
    ax1.set_title("Speed Tracking")
    ax1.grid(True, alpha=0.3, linestyle="--")
    ax1.legend(loc="upper right")
    speed_stats = add_stats_box(ax1, speed_data["data"], speed_data["target_data"], "m/s")

    # 绘制角度 (第二个子图)
    ax2.plot(angle_data["relative_time"], angle_data["data"], linewidth=2, label="current_pitch")
    ax2.plot(angle_data["target_relative_time"], angle_data["target_data"], linewidth=2, label="target_pitch")
    ax2.set_ylabel("rad")
    ax2.set_title("Pitch Tracking")
    ax2.grid(True, alpha=0.3, linestyle="--")
    ax2.legend(loc="upper right")
    angle_stats = add_stats_box(ax2, angle_data["data"], angle_data["target_data"], "rad")

    # 绘制角速率 (第三个子图)
    ax3.plot(gyro_data["relative_time"], gyro_data["data"], linewidth=2, label="current_pitch_rate")
    ax3.plot(gyro_data["target_relative_time"], gyro_data["target_data"], linewidth=2, label="target_pitch_rate")
    ax3.set_xlabel("time / s")
    ax3.set_ylabel("rad/s")
    ax3.set_title("Pitch Rate Tracking")
    ax3.grid(True, alpha=0.3, linestyle="--")
    ax3.legend(loc="upper right")
    gyro_stats = add_stats_box(ax3, gyro_data["data"], gyro_data["target_data"], "rad/s")

    # 如果有电流数据，绘制第四个子图
    if current_data is not None:
        ax4 = axes[3]
        ax4.plot(
            current_data["relative_left"],
            current_data["left"],
            linewidth=1,
            label="left_cmd_mA",
        )
        ax4.plot(
            current_data["relative_right"],
            -current_data["right"],
            linewidth=1,
            label="right_cmd_mA",
        )
        ax4.set_xlabel("time / s")
        ax4.set_ylabel("mA")
        ax4.set_title("Wheel Command Current")
        ax4.grid(True, alpha=0.3, linestyle="--")
        ax4.legend(loc="upper right")

    # ---------- 修改 2：设置横轴刻度间隔为 0.1 秒 ----------
    # 对每个子图设置主刻度间隔为 0.1
    for ax in axes:
        ax.xaxis.set_major_locator(MultipleLocator(0.2))
        ax.set_xlim(TIME_START, TIME_END)
        auto_ylim( ax1,
            speed_data["relative_time"],
            speed_data["data"],
            TIME_START,
            TIME_END
        )
        auto_ylim( ax2,
            angle_data["relative_time"],
            angle_data["data"],
            TIME_START,
            TIME_END
        )
        auto_ylim( ax3,
            gyro_data["relative_time"],
            gyro_data["data"],
            TIME_START,
            TIME_END
        )
        if current_data is not None:
            auto_ylim( ax4,
                current_data["relative_left"],
                current_data["left"],
                TIME_START,
                TIME_END
            )
    plt.tight_layout()

    output_path = base_path + "all_metrics_comparison.png"
    plt.savefig(output_path, dpi=300, bbox_inches="tight")

    print("\n图像已保存：")
    print(output_path)

    print("\n统计结果：")
    # 注意统计顺序与子图顺序一致（现在第一个是速度，第二个是角度）
    print(f"Speed      RMSE={speed_stats[0]:.4f}, MAE={speed_stats[1]:.4f}, MaxE={speed_stats[2]:.4f}")
    print(f"Pitch      RMSE={angle_stats[0]:.4f}, MAE={angle_stats[1]:.4f}, MaxE={angle_stats[2]:.4f}")
    print(f"PitchRate  RMSE={gyro_stats[0]:.4f}, MAE={gyro_stats[1]:.4f}, MaxE={gyro_stats[2]:.4f}")
    if current_data is not None:
        print(
            f"WheelCmd   LeftMean={np.mean(current_data['left']):.1f} mA, "
            f"RightMean={np.mean(current_data['right']):.1f} mA"
        )

    plt.show()


if __name__ == "__main__":
    main()