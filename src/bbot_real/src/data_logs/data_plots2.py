#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

base_path = "/home/robot/bbot_real/src/bbot_real/src/data_logs/"

# ===============================
# 时间显示范围 (单位: 秒)
# ===============================
TIME_START = 0
TIME_END = 65.0


# ==============================================================================
# 因克斯 (ENCOS) 电机 CAN 协议解析工具 (PDF 手册第 9.3 节 & 第 10 节)
# ==============================================================================
def parse_encos_feedback(data_bytes: bytes) -> dict:
    if len(data_bytes) < 2:
        return {}
    frame_type = (data_bytes[0] >> 5) & 0x07
    err_code = data_bytes[0] & 0x1F

    info = {"frame_type": frame_type, "error_code": err_code}
    if frame_type == 1 and len(data_bytes) >= 8:
        pos_raw = (data_bytes[1] << 8) | data_bytes[2]
        spd_raw = (data_bytes[3] << 4) | (data_bytes[4] >> 4)
        info["pos_rad"] = (pos_raw / 65535.0) * 25.0 - 12.5
        info["spd_rad_s"] = (spd_raw / 4095.0) * 36.0 - 18.0
        info["motor_temp"] = (data_bytes[6] - 50) / 2.0
    elif frame_type == 2 and len(data_bytes) >= 8:
        info["pos_deg"] = struct.unpack(">f", data_bytes[1:5])[0]
        info["current_a"] = struct.unpack(">h", data_bytes[5:7])[0] / 100.0
    elif frame_type == 3 and len(data_bytes) >= 8:
        info["spd_rpm"] = struct.unpack(">f", data_bytes[1:5])[0]
        info["current_a"] = struct.unpack(">h", data_bytes[5:7])[0] / 100.0
    elif frame_type == 5 and len(data_bytes) >= 4:
        query_id = data_bytes[1]
        if query_id == 3 and len(data_bytes) >= 6:
            info["current_a"] = struct.unpack(">f", data_bytes[2:6])[0]
        elif query_id == 22 and len(data_bytes) >= 4:
            info["kt"] = ((data_bytes[2] << 8) | data_bytes[3]) / 100.0
    return info


# ===============================
# 数据文件路径
# ===============================
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

def get_motor_file(primary_name, fallback_name):
    p = os.path.join(base_path, primary_name)
    return p if os.path.exists(p) else os.path.join(base_path, fallback_name)

motor_files = {
    "motor1": get_motor_file("hip_left_torque.txt", "motor1_torque.txt"),
    "motor2": get_motor_file("knee_left_torque.txt", "motor2_torque.txt"),
    "motor3": get_motor_file("hip_right_torque.txt", "motor3_torque.txt"),
    "motor4": get_motor_file("knee_right_torque.txt", "motor4_torque.txt"),
}


# ===============================
# 文件读取
# ===============================
def read_data(file_path, name):
    try:
        with open(file_path, "r") as f:
            data = [float(line.strip()) for line in f if line.strip()]
        print(f"Loaded {name}: {len(data)} points")
        return np.array(data)
    except Exception as e:
        print(f"Failed to read {name}: {e}")
        return np.array([])


# ===============================
# 数据对齐与处理
# ===============================
def process_dataset(files, label):
    data = read_data(files["data"], label + " actual")
    target = read_data(files["target"], label + " target")
    t = read_data(files["timestamp"], label + " time")
    tt = read_data(files["target_timestamp"], label + " target time")

    n = min(len(data), len(target), len(t), len(tt))
    data = data[:n]
    target = target[:n]

    if len(t) > 1 and (t[n - 1] - t[0]) > 0.1:
        t = t[:n] - t[0]
        tt = tt[:n] - tt[0]
    else:
        t = np.arange(n) * 0.005
        tt = np.arange(n) * 0.005

    return {"data": data, "target": target, "time": t, "target_time": tt}


def process_current():
    left = read_data(current_files["left"], "left current")
    right = read_data(current_files["right"], "right current")
    tl = read_data(current_files["timestamp_left"], "left time")
    tr = read_data(current_files["timestamp_right"], "right time")

    n = min(len(left), len(right), len(tl), len(tr))
    left = left[:n]
    right = right[:n]

    if len(tl) > 1 and (tl[n - 1] - tl[0]) > 0.1:
        tl = tl[:n] - tl[0]
        tr = tr[:n] - tr[0]
    else:
        tl = np.arange(n) * 0.005
        tr = np.arange(n) * 0.005

    return {"left": left, "right": right, "tl": tl, "tr": tr}


def process_motor():
    motor = {}
    for key, file in motor_files.items():
        data = read_data(file, key + " torque")
        motor[key] = data

    valid_lens = [len(v) for v in motor.values() if len(v) > 0]
    n = min(valid_lens) if valid_lens else 0

    for key in motor:
        if len(motor[key]) >= n:
            motor[key] = motor[key][:n]
        else:
            motor[key] = np.zeros(n)

    motor["time"] = np.arange(n) * 0.005
    return motor


def auto_ylim(ax, x, y):
    if len(x) == 0 or len(y) == 0:
        return

    if len(x) == len(y):
        mask = (x >= TIME_START) & (x <= TIME_END)
        data = y[mask] if np.sum(mask) > 0 else y
    else:
        data = y

    ymin = np.min(data)
    ymax = np.max(data)
    r = ymax - ymin
    if r < 1e-6:
        r = max(abs(ymax), 0.1)
    margin = r * 0.15
    ax.set_ylim(ymin - margin, ymax + margin)


# ===============================
# 主绘图流程
# ===============================
def main():
    plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial"]
    plt.rcParams["axes.unicode_minus"] = False

    angle = process_dataset(angle_files, "pitch")
    speed = process_dataset(speed_files, "speed")
    gyro = process_dataset(gyro_files, "gyro")
    current = process_current()
    motor = process_motor()

    fig, axes = plt.subplots(5, 1, figsize=(16, 20), sharex=True)

    # 1. 速度跟踪
    ax1 = axes[0]
    ax1.plot(speed["time"], speed["data"], label="Actual Speed", color="#1f77b4", lw=1.6)
    ax1.plot(speed["target_time"], speed["target"], label="Target Speed", color="#ff7f0e", linestyle="--", lw=1.8)
    ax1.set_ylabel("Speed (m/s)", fontsize=11, fontweight="bold")
    ax1.set_title("Robot Forward Speed Tracking", fontsize=13, fontweight="bold")
    ax1.legend(loc="upper right", framealpha=0.9)
    ax1.grid(True, linestyle=":", alpha=0.6)

    # 2. 俯仰角跟踪
    ax2 = axes[1]
    ax2.plot(angle["time"], angle["data"], label="Actual Pitch", color="#2ca02c", lw=1.6)
    ax2.plot(angle["target_time"], angle["target"], label="Target Pitch", color="#d62728", linestyle="--", lw=1.8)
    ax2.set_ylabel("Pitch (rad)", fontsize=11, fontweight="bold")
    ax2.set_title("Chassis Pitch Balance Tracking", fontsize=13, fontweight="bold")
    ax2.legend(loc="upper right", framealpha=0.9)
    ax2.grid(True, linestyle=":", alpha=0.6)

    # 3. 俯仰角速度
    ax3 = axes[2]
    ax3.plot(gyro["time"], gyro["data"], label="Actual Pitch Rate", color="#9467bd", lw=1.4)
    ax3.plot(gyro["target_time"], gyro["target"], label="Target Pitch Rate", color="#8c564b", linestyle="--", lw=1.6)
    ax3.set_ylabel("Pitch Rate (rad/s)", fontsize=11, fontweight="bold")
    ax3.set_title("Pitch Angular Velocity Response", fontsize=13, fontweight="bold")
    ax3.legend(loc="upper right", framealpha=0.9)
    ax3.grid(True, linestyle=":", alpha=0.6)

    # 4. 轮毂电机电流
    ax4 = axes[3]
    ax4.plot(current["tl"], current["left"], label="Left Wheel Current", color="#17becf", lw=1.5)
    ax4.plot(current["tr"], -current["right"], label="Right Wheel Current (Inverted)", color="#e377c2", lw=1.5)
    ax4.set_ylabel("Current (mA)", fontsize=11, fontweight="bold")
    ax4.set_title("Wheel Hub Motor Output Current", fontsize=13, fontweight="bold")
    ax4.legend(loc="upper right", framealpha=0.9)
    ax4.grid(True, linestyle=":", alpha=0.6)

    # 5. 四关节电机力矩反馈（已将右侧电机 Motor 3 / Motor 4 取反以便同向对比）
    ax5 = axes[4]
    motor_labels = {
        "motor1": "Motor ID 1 (Left Hip)",
        "motor2": "Motor 2 (Left Knee)",
        "motor3": "Motor 3 (Right Hip)",
        "motor4": "Motor 4 (Right Knee - Inverted)",
    }
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
    invert_motors = ["motor4"]  # 将原本在零点下方的右侧关节力矩取反

    plotted_motor_data = []
    for i in range(1, 5):
        key = f"motor{i}"
        if key in motor and len(motor[key]) > 0:
            # 取反操作
            data_to_plot = -motor[key] if key in invert_motors else motor[key]
            ax5.plot(motor["time"], data_to_plot, label=motor_labels[key], color=colors[i - 1], lw=1.6)
            plotted_motor_data.append(data_to_plot)

    ax5.set_xlabel("Time (s)", fontsize=11, fontweight="bold")
    ax5.set_ylabel("Torque (Nm)", fontsize=11, fontweight="bold")
    ax5.set_title("Joint Motors Feedback Torque (CAN Query 0x07/0x03)", fontsize=13, fontweight="bold")
    ax5.legend(loc="upper right", ncol=2, framealpha=0.9)
    ax5.grid(True, linestyle=":", alpha=0.6)

    # 坐标范围设置
    for ax in axes:
        ax.set_xlim(TIME_START, TIME_END)
        ax.xaxis.set_major_locator(MultipleLocator(1.0))
        ax.xaxis.set_minor_locator(MultipleLocator(0.2))

    auto_ylim(ax1, speed["time"], speed["data"])
    auto_ylim(ax2, angle["time"], angle["data"])
    auto_ylim(ax3, gyro["time"], gyro["data"])
    auto_ylim(ax4, current["tl"], current["left"])

    # 对第 5 张图使用翻转后的数据计算 Y 轴缩放
    if plotted_motor_data:
        all_motor_time = np.concatenate([motor["time"]] * len(plotted_motor_data))
        auto_ylim(ax5, all_motor_time, np.concatenate(plotted_motor_data))

    plt.tight_layout()
    output = base_path + "all_metrics_comparison.png"
    plt.savefig(output, dpi=300, bbox_inches="tight")
    print("Saved figure successfully:", output)
    plt.show()


if __name__ == "__main__":
    main()