#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import os

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


def main():
    print("=" * 60)
    print("Balance Controller Data Plot")
    print("=" * 60)

    all_ok = True
    all_ok &= check_files_exist(angle_files, "angle")
    all_ok &= check_files_exist(speed_files, "speed")
    all_ok &= check_files_exist(gyro_files, "gyro")

    if not all_ok:
        print("数据文件不完整，请先运行控制代码生成 txt 数据。")
        return

    angle_data = process_dataset(angle_files, "angle")
    speed_data = process_dataset(speed_files, "speed")
    gyro_data = process_dataset(gyro_files, "gyro")

    if angle_data is None or speed_data is None or gyro_data is None:
        print("数据为空，无法绘图。")
        return

    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)

    ax1 = axes[0]
    ax1.plot(angle_data["relative_time"], angle_data["data"], linewidth=2, label="current_pitch")
    ax1.plot(angle_data["target_relative_time"], angle_data["target_data"], linewidth=2, label="target_pitch")
    ax1.set_ylabel("rad")
    ax1.set_title("Pitch Tracking")
    ax1.grid(True, alpha=0.3, linestyle="--")
    ax1.legend(loc="upper right")
    angle_stats = add_stats_box(ax1, angle_data["data"], angle_data["target_data"], "rad")

    ax2 = axes[1]
    ax2.plot(speed_data["relative_time"], speed_data["data"], linewidth=2, label="current_speed")
    ax2.plot(speed_data["target_relative_time"], speed_data["target_data"], linewidth=2, label="target_speed")
    ax2.set_ylabel("m/s")
    ax2.set_title("Speed Tracking")
    ax2.grid(True, alpha=0.3, linestyle="--")
    ax2.legend(loc="upper right")
    speed_stats = add_stats_box(ax2, speed_data["data"], speed_data["target_data"], "m/s")

    ax3 = axes[2]
    ax3.plot(gyro_data["relative_time"], gyro_data["data"], linewidth=2, label="current_pitch_rate")
    ax3.plot(gyro_data["target_relative_time"], gyro_data["target_data"], linewidth=2, label="target_pitch_rate")
    ax3.set_xlabel("time / s")
    ax3.set_ylabel("rad/s")
    ax3.set_title("Pitch Rate Tracking")
    ax3.grid(True, alpha=0.3, linestyle="--")
    ax3.legend(loc="upper right")
    gyro_stats = add_stats_box(ax3, gyro_data["data"], gyro_data["target_data"], "rad/s")

    plt.tight_layout()

    output_path = base_path + "all_metrics_comparison.png"
    plt.savefig(output_path, dpi=300, bbox_inches="tight")

    print("\n图像已保存：")
    print(output_path)

    print("\n统计结果：")
    print(f"Pitch      RMSE={angle_stats[0]:.4f}, MAE={angle_stats[1]:.4f}, MaxE={angle_stats[2]:.4f}")
    print(f"Speed      RMSE={speed_stats[0]:.4f}, MAE={speed_stats[1]:.4f}, MaxE={speed_stats[2]:.4f}")
    print(f"PitchRate  RMSE={gyro_stats[0]:.4f}, MAE={gyro_stats[1]:.4f}, MaxE={gyro_stats[2]:.4f}")

    plt.show()


if __name__ == "__main__":
    main()