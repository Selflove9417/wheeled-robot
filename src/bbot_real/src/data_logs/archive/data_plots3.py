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
TIME_END = 100.0


# ==============================================================================
# 因克斯 (ENCOS) 电机 CAN 协议解析工具 (PDF 手册第 9.3 节 & 第 10.5 节)
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
        if query_id == 3 and len(data_bytes) >= 6:  # 查询当前相电流 (A)
            info["current_a"] = struct.unpack(">f", data_bytes[2:6])[0]
        elif query_id == 4 and len(data_bytes) >= 6:  # 查询当前功率 (W)
            info["power_w"] = struct.unpack(">f", data_bytes[2:6])[0]
        elif query_id == 22 and len(data_bytes) >= 4:  # 查询扭矩常数 Kt
            info["kt"] = ((data_bytes[2] << 8) | data_bytes[3]) / 100.0
    return info


# ===============================
# 数据文件路径配置
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

wheel_current_files = {
    "left": base_path + "left_current_data.txt",
    "right": base_path + "right_current_data.txt",
    "timestamp_left": base_path + "timestamp_left_current.txt",
    "timestamp_right": base_path + "timestamp_right_current.txt",
}

def get_file(primary, fallback):
    p = os.path.join(base_path, primary)
    return p if os.path.exists(p) else os.path.join(base_path, fallback)

# 关节电机三维状态文件（力矩、电流、功率）
joint_torque_files = {
    "motor1": get_file("hip_left_torque.txt", "motor1_torque.txt"),
    "motor2": get_file("knee_left_torque.txt", "motor2_torque.txt"),
    "motor3": get_file("hip_right_torque.txt", "motor3_torque.txt"),
    "motor4": get_file("knee_right_torque.txt", "motor4_torque.txt"),
}

joint_current_files = {
    "motor1": get_file("hip_left_current.txt", "motor1_current.txt"),
    "motor2": get_file("knee_left_current.txt", "motor2_current.txt"),
    "motor3": get_file("hip_right_current.txt", "motor3_current.txt"),
    "motor4": get_file("knee_right_current.txt", "motor4_current.txt"),
}

joint_power_files = {
    "motor1": get_file("hip_left_power.txt", "motor1_power.txt"),
    "motor2": get_file("knee_left_power.txt", "motor2_power.txt"),
    "motor3": get_file("hip_right_power.txt", "motor3_power.txt"),
    "motor4": get_file("knee_right_power.txt", "motor4_power.txt"),
}


# ===============================
# 文件读取
# ===============================
def read_data(file_path, name):
    try:
        with open(file_path, "r") as f:
            data = [float(line.strip()) for line in f if line.strip()]
        return np.array(data)
    except Exception as e:
        return np.array([])


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


def process_wheel_current():
    left = read_data(wheel_current_files["left"], "left current")
    right = read_data(wheel_current_files["right"], "right current")
    tl = read_data(wheel_current_files["timestamp_left"], "left time")
    tr = read_data(wheel_current_files["timestamp_right"], "right time")

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


def process_joint_group(file_dict):
    group = {}
    for key, file in file_dict.items():
        group[key] = read_data(file, key)

    valid_lens = [len(v) for v in group.values() if len(v) > 0]
    n = min(valid_lens) if valid_lens else 0

    for key in group:
        if len(group[key]) >= n:
            group[key] = group[key][:n]
        else:
            group[key] = np.zeros(n)

    group["time"] = np.arange(n) * 0.005
    return group


def auto_ylim(ax, x, y):
    if len(x) == 0 or len(y) == 0:
        return

    if len(x) == len(y):
        mask = (x >= TIME_START) & (x <= TIME_END)
        data = y[mask] if np.sum(mask) > 0 else y
    else:
        data = y

    if len(data) == 0:
        return

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

    speed = process_dataset(speed_files, "speed")
    angle = process_dataset(angle_files, "pitch")
    gyro = process_dataset(gyro_files, "gyro")
    wheel_cur = process_wheel_current()
    joint_tor = process_joint_group(joint_torque_files)
    joint_cur = process_joint_group(joint_current_files)
    joint_pwr = process_joint_group(joint_power_files)

    fig, axes = plt.subplots(7, 1, figsize=(16, 26), sharex=True)

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
    ax4.plot(wheel_cur["tl"], wheel_cur["left"], label="Left Wheel Current", color="#17becf", lw=1.5)
    ax4.plot(wheel_cur["tr"], -wheel_cur["right"], label="Right Wheel Current (Inverted)", color="#e377c2", lw=1.5)
    ax4.set_ylabel("Current (mA)", fontsize=11, fontweight="bold")
    ax4.set_title("Wheel Hub Motor Output Current", fontsize=13, fontweight="bold")
    ax4.legend(loc="upper right", framealpha=0.9)
    ax4.grid(True, linestyle=":", alpha=0.6)

    # 关节电机颜色与标签配置
    motor_labels = {
        "motor1": "Motor 1 (Left Hip)",
        "motor2": "Motor 2 (Left Knee)",
        "motor3": "Motor 3 (Right Hip)",
        "motor4": "Motor 4 (Right Knee - Inverted)",
    }
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
    invert_motors = ["motor4"]

    def plot_motor_dataset(ax, data_group, unit_str, title_str):
        plotted = []
        for i in range(1, 5):
            k = f"motor{i}"
            if k in data_group and len(data_group[k]) > 0:
                y = -data_group[k] if k in invert_motors else data_group[k]
                ax.plot(data_group["time"], y, label=motor_labels[k], color=colors[i - 1], lw=1.6)
                plotted.append(y)
        ax.set_ylabel(unit_str, fontsize=11, fontweight="bold")
        ax.set_title(title_str, fontsize=13, fontweight="bold")
        ax.legend(loc="upper right", ncol=2, framealpha=0.9)
        ax.grid(True, linestyle=":", alpha=0.6)
        return plotted

    # 5. 四关节电机力矩反馈
    tor_plotted = plot_motor_dataset(axes[4], joint_tor, "Torque (Nm)", "Joint Motors Feedback Torque (Nm)")
    # 6. 四关节电机相电流反馈
    cur_plotted = plot_motor_dataset(axes[5], joint_cur, "Current (A)", "Joint Motors Feedback Phase Current (A)")
    # 7. 四关节电机功率反馈
    pwr_plotted = plot_motor_dataset(axes[6], joint_pwr, "Power (W)", "Joint Motors Output Power (W)")

    axes[6].set_xlabel("Time (s)", fontsize=11, fontweight="bold")

    # 坐标范围与刻度设置
    for ax in axes:
        ax.set_xlim(TIME_START, TIME_END)
        ax.xaxis.set_major_locator(MultipleLocator(1.0))
        ax.xaxis.set_minor_locator(MultipleLocator(0.2))

    auto_ylim(ax1, speed["time"], speed["data"])
    auto_ylim(ax2, angle["time"], angle["data"])
    auto_ylim(ax3, gyro["time"], gyro["data"])
    auto_ylim(ax4, wheel_cur["tl"], wheel_cur["left"])

    if tor_plotted:
        auto_ylim(axes[4], np.concatenate([joint_tor["time"]] * len(tor_plotted)), np.concatenate(tor_plotted))
    if cur_plotted:
        auto_ylim(axes[5], np.concatenate([joint_cur["time"]] * len(cur_plotted)), np.concatenate(cur_plotted))
    if pwr_plotted:
        auto_ylim(axes[6], np.concatenate([joint_pwr["time"]] * len(pwr_plotted)), np.concatenate(pwr_plotted))

    plt.tight_layout()
    output = base_path + "all_metrics_comparison.png"
    plt.savefig(output, dpi=300, bbox_inches="tight")
    print("Saved figure successfully:", output)
    plt.show()


if __name__ == "__main__":
    main()