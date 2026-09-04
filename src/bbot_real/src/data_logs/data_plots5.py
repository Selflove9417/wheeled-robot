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
TIME_START = 1.0
TIME_END = 270.0

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
        pos_raw = (
            (data_bytes[1] << 8) |
            data_bytes[2]
        )
        spd_raw = (
            (data_bytes[3] << 4) |
            (data_bytes[4] >> 4)
        )
        info["pos_rad"] = (
            (pos_raw / 65535.0) * 25.0 - 12.5
        )
        info["spd_rad_s"] = (
            (spd_raw / 4095.0) * 36.0 - 18.0
        )
        info["motor_temp"] = (
            data_bytes[6] - 50
        ) / 2.0
        info["mos_temp"] = (
            data_bytes[7] - 50
        ) / 2.0

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

temperature_files = {
    "hip_left_motor": base_path + "hip_left_motor_temp.txt",
    "hip_left_mos": base_path + "hip_left_mos_temp.txt",
    "knee_left_motor": base_path + "knee_left_motor_temp.txt",
    "knee_left_mos": base_path + "knee_left_mos_temp.txt",
    "hip_right_motor": base_path + "hip_right_motor_temp.txt",
    "hip_right_mos": base_path + "hip_right_mos_temp.txt",
    "knee_right_motor": base_path + "knee_right_motor_temp.txt",
    "knee_right_mos": base_path + "knee_right_mos_temp.txt",
    "timestamp": base_path + "timestamp_joint_temp.txt",
}

# ===============================
# 新增：腿高 / 膝关节诊断日志
# ===============================
leg_diag_files = {
    "height": base_path + "current_height.txt",
    "left_height": base_path + "left_leg_height.txt",
    "right_height": base_path + "right_leg_height.txt",
    "knee_left_current": base_path + "knee_left_current_feedback.txt",
    "knee_right_current": base_path + "knee_right_current_feedback.txt",
    "knee_left_kt": base_path + "knee_left_kt.txt",
    "knee_right_kt": base_path + "knee_right_kt.txt",
    "timestamp": base_path + "timestamp_leg_diag.txt",
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
    if n == 0:
        return {
            "data": np.array([]),
            "target": np.array([]),
            "time": np.array([]),
            "target_time": np.array([]),
        }

    data = data[:n]
    target = target[:n]

    if len(t) > 1 and (t[n - 1] - t[0]) > 0.1:
        t = t[:n] - t[0]
        tt = tt[:n] - tt[0]
    else:
        t = np.arange(n) * 0.005
        tt = np.arange(n) * 0.005

    return {
        "data": data,
        "target": target,
        "time": t,
        "target_time": tt,
    }

def process_current():
    left = read_data(current_files["left"], "left current")
    right = read_data(current_files["right"], "right current")
    tl = read_data(current_files["timestamp_left"], "left time")
    tr = read_data(current_files["timestamp_right"], "right time")

    n = min(len(left), len(right), len(tl), len(tr))
    if n == 0:
        return {
            "left": np.array([]),
            "right": np.array([]),
            "tl": np.array([]),
            "tr": np.array([]),
        }

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

def process_temperature():
    temp = {}
    for key, file_path in temperature_files.items():
        if key == "timestamp":
            continue
        temp[key] = read_data(
            file_path,
            key.replace("_", " ")
        )

    t = read_data(
        temperature_files["timestamp"],
        "joint temperature time"
    )

    if len(t) == 0:
        temp["time"] = np.array([])
        return temp

    valid_lengths = [len(t)]
    for value in temp.values():
        if len(value) > 0:
            valid_lengths.append(len(value))
    n = min(valid_lengths)

    t = t[:n]
    for key in list(temp.keys()):
        if len(temp[key]) >= n:
            temp[key] = temp[key][:n]
        else:
            temp[key] = np.full(n, np.nan)

    if len(t) > 1 and (t[-1] - t[0]) > 0.1:
        t = t - t[0]
    else:
        t = np.arange(n) * 0.005

    temp["time"] = t
    return temp

def process_leg_diag():
    diag = {}
    for key, file_path in leg_diag_files.items():
        if key == "timestamp":
            continue
        diag[key] = read_data(
            file_path,
            key.replace("_", " ")
        )

    t = read_data(
        leg_diag_files["timestamp"],
        "leg diagnostic time"
    )

    if len(t) == 0:
        diag["time"] = np.array([])
        return diag

    n = len(t)
    valid_lengths = [len(t)]
    for value in diag.values():
        if len(value) > 0:
            valid_lengths.append(len(value))
    n = min(valid_lengths)

    t = t[:n]
    for key in list(diag.keys()):
        if len(diag[key]) >= n:
            diag[key] = diag[key][:n]
        else:
            diag[key] = np.full(n, np.nan)

    if len(t) > 1 and (t[-1] - t[0]) > 0.1:
        t = t - t[0]
    else:
        t = np.arange(n) * 0.005

    diag["time"] = t
    return diag

def auto_ylim(ax, x, y):
    if len(x) == 0 or len(y) == 0:
        return
    if len(x) == len(y):
        mask = (x >= TIME_START) & (x <= TIME_END)
        data = y[mask] if np.sum(mask) > 0 else y
    else:
        data = y

    data = np.asarray(data)
    data = data[np.isfinite(data)]
    if len(data) == 0:
        return

    ymin = np.min(data)
    ymax = np.max(data)
    r = ymax - ymin
    if r < 1e-6:
        r = max(abs(ymax), 0.1)
    margin = r * 0.15
    ax.set_ylim(ymin - margin, ymax + margin)

def print_knee_kt_summary(leg_diag):
    left_kt = leg_diag.get("knee_left_kt", np.array([]))
    right_kt = leg_diag.get("knee_right_kt", np.array([]))

    if len(left_kt) > 0:
        left_valid = left_kt[np.isfinite(left_kt)]
        if len(left_valid) > 0:
            print(f"Left Knee Kt median:  {np.median(left_valid):.4f} Nm/A")

    if len(right_kt) > 0:
        right_valid = right_kt[np.isfinite(right_kt)]
        if len(right_valid) > 0:
            print(f"Right Knee Kt median: {np.median(right_valid):.4f} Nm/A")

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
    temperature = process_temperature()
    leg_diag = process_leg_diag()

    print_knee_kt_summary(leg_diag)

    # 启用约束布局，替代tight_layout，大幅提升渲染速度
    fig, axes = plt.subplots(9, 1, figsize=(16, 26), sharex=True, layout='constrained')

    # ============================================================
    # 1. 速度跟踪
    # ============================================================
    ax1 = axes[0]
    ax1.plot(
        speed["time"],
        speed["data"],
        label="Actual Speed",
        color="#1f77b4",
        lw=1.6
    )
    ax1.plot(
        speed["target_time"],
        speed["target"],
        label="Target Speed",
        color="#ff7f0e",
        linestyle="--",
        lw=1.8
    )
    ax1.set_ylabel("Speed (m/s)", fontsize=11, fontweight="bold")
    ax1.set_title("Robot Forward Speed Tracking", fontsize=13, fontweight="bold")
    ax1.legend(loc="upper right", framealpha=0.9)
    ax1.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 2. 俯仰角跟踪
    # ============================================================
    ax2 = axes[1]
    ax2.plot(
        angle["time"],
        angle["data"],
        label="Actual Pitch",
        color="#2ca02c",
        lw=1.6
    )
    ax2.plot(
        angle["target_time"],
        angle["target"],
        label="Target Pitch",
        color="#d62728",
        linestyle="--",
        lw=1.8
    )
    ax2.set_ylabel("Pitch (rad)", fontsize=11, fontweight="bold")
    ax2.set_title("Chassis Pitch Balance Tracking", fontsize=13, fontweight="bold")
    ax2.legend(loc="upper right", framealpha=0.9)
    ax2.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 3. 俯仰角速度
    # ============================================================
    ax3 = axes[2]
    ax3.plot(
        gyro["time"],
        gyro["data"],
        label="Actual Pitch Rate",
        color="#9467bd",
        lw=1.4
    )
    ax3.plot(
        gyro["target_time"],
        gyro["target"],
        label="Target Pitch Rate",
        color="#8c564b",
        linestyle="--",
        lw=1.6
    )
    ax3.set_ylabel("Pitch Rate (rad/s)", fontsize=11, fontweight="bold")
    ax3.set_title("Pitch Angular Velocity Response", fontsize=13, fontweight="bold")
    ax3.legend(loc="upper right", framealpha=0.9)
    ax3.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 4. 轮毂电机电流
    # ============================================================
    ax4 = axes[3]
    ax4.plot(
        current["tl"],
        current["left"],
        label="Left Wheel Current",
        color="#17becf",
        lw=1.5
    )
    ax4.plot(
        current["tr"],
        -current["right"],
        label="Right Wheel Current (Inverted)",
        color="#e377c2",
        lw=1.5
    )
    ax4.set_ylabel("Current (mA)", fontsize=11, fontweight="bold")
    ax4.set_title("Wheel Hub Motor Output Current", fontsize=13, fontweight="bold")
    ax4.legend(loc="upper right", framealpha=0.9)
    ax4.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 5. 四关节电机力矩反馈
    # ============================================================
    ax5 = axes[4]
    motor_labels = {
        "motor1": "Motor ID 1 (Left Hip)",
        "motor2": "Motor 2 (Left Knee)",
        "motor3": "Motor 3 (Right Hip)",
        "motor4": "Motor 4 (Right Knee - Inverted)",
    }
    colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
    invert_motors = ["motor4"]
    plotted_motor_data = []

    for i in range(1, 5):
        key = f"motor{i}"
        if key in motor and len(motor[key]) > 0:
            data_to_plot = -motor[key] if key in invert_motors else motor[key]
            ax5.plot(
                motor["time"],
                data_to_plot,
                label=motor_labels[key],
                color=colors[i - 1],
                lw=1.6
            )
            plotted_motor_data.append(data_to_plot)

    ax5.set_ylabel("Torque (Nm)", fontsize=11, fontweight="bold")
    ax5.set_title(
        "Joint Motors Feedback Torque (CAN Query 0x07/0x03)",
        fontsize=13,
        fontweight="bold"
    )
    ax5.legend(loc="upper right", ncol=2, framealpha=0.9)
    ax5.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 6. 四关节电机线圈温度
    # ============================================================
    ax6 = axes[5]
    temp_colors = {
        "hip_left": "#e41a1c",
        "knee_left": "#377eb8",
        "hip_right": "#4daf4a",
        "knee_right": "#984ea3",
    }

    if len(temperature.get("time", [])) > 0:
        ax6.plot(
            temperature["time"],
            temperature["hip_left_motor"],
            label="Left Hip Motor",
            color=temp_colors["hip_left"],
            lw=1.6
        )
        ax6.plot(
            temperature["time"],
            temperature["knee_left_motor"],
            label="Left Knee Motor",
            color=temp_colors["knee_left"],
            lw=1.8
        )
        ax6.plot(
            temperature["time"],
            temperature["hip_right_motor"],
            label="Right Hip Motor",
            color=temp_colors["hip_right"],
            lw=1.6
        )
        ax6.plot(
            temperature["time"],
            temperature["knee_right_motor"],
            label="Right Knee Motor",
            color=temp_colors["knee_right"],
            lw=1.8
        )

    ax6.axhline(
        105.0,
        color="orange",
        linestyle="--",
        lw=1.4,
        label="Motor Derating Start (105 C)"
    )
    ax6.axhline(
        120.0,
        color="red",
        linestyle="--",
        lw=1.4,
        label="Motor Shutdown (120 C)"
    )
    ax6.set_ylabel("Motor Temp (C)", fontsize=11, fontweight="bold")
    ax6.set_title("Joint Motor Winding Temperature", fontsize=13, fontweight="bold")
    ax6.legend(loc="upper left", ncol=2, framealpha=0.9)
    ax6.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 7. 四关节驱动 MOS 温度
    # ============================================================
    ax7 = axes[6]
    if len(temperature.get("time", [])) > 0:
        ax7.plot(
            temperature["time"],
            temperature["hip_left_mos"],
            label="Left Hip MOS",
            color=temp_colors["hip_left"],
            lw=1.6
        )
        ax7.plot(
            temperature["time"],
            temperature["knee_left_mos"],
            label="Left Knee MOS",
            color=temp_colors["knee_left"],
            lw=1.8
        )
        ax7.plot(
            temperature["time"],
            temperature["hip_right_mos"],
            label="Right Hip MOS",
            color=temp_colors["hip_right"],
            lw=1.6
        )
        ax7.plot(
            temperature["time"],
            temperature["knee_right_mos"],
            label="Right Knee MOS",
            color=temp_colors["knee_right"],
            lw=1.8
        )

    ax7.axhline(
        95.0,
        color="orange",
        linestyle="--",
        lw=1.4,
        label="MOS Derating Start (95 C)"
    )
    ax7.axhline(
        110.0,
        color="red",
        linestyle="--",
        lw=1.4,
        label="MOS Shutdown (110 C)"
    )
    ax7.set_ylabel("MOS Temp (C)", fontsize=11, fontweight="bold")
    ax7.set_title("Joint Motor Driver MOS Temperature", fontsize=13, fontweight="bold")
    ax7.legend(loc="upper left", ncol=2, framealpha=0.9)
    ax7.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 8. 虚拟腿高度
    # ============================================================
    ax8 = axes[7]
    if len(leg_diag.get("time", [])) > 0:
        ax8.plot(
            leg_diag["time"],
            leg_diag["height"],
            label="Center Height",
            color="#000000",
            lw=1.8
        )
        ax8.plot(
            leg_diag["time"],
            leg_diag["left_height"],
            label="Left Leg Height",
            color="#377eb8",
            lw=1.4
        )
        ax8.plot(
            leg_diag["time"],
            leg_diag["right_height"],
            label="Right Leg Height",
            color="#984ea3",
            lw=1.4
        )
    ax8.set_ylabel("Leg Height (m)", fontsize=11, fontweight="bold")
    ax8.set_title("Virtual Leg Height", fontsize=13, fontweight="bold")
    ax8.legend(loc="upper right", framealpha=0.9)
    ax8.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 9. 左右膝关节反馈电流
    # ============================================================
    ax9 = axes[8]
    if len(leg_diag.get("time", [])) > 0:
        ax9.plot(
            leg_diag["time"],
            leg_diag["knee_left_current"],
            label="Left Knee Current",
            color="#377eb8",
            lw=1.6
        )
        ax9.plot(
            leg_diag["time"],
            -leg_diag["knee_right_current"],
            label="Right Knee Current (Inverted)",
            color="#984ea3",
            lw=1.6
        )
    ax9.set_xlabel("Time (s)", fontsize=11, fontweight="bold")
    ax9.set_ylabel("Current (A)", fontsize=11, fontweight="bold")
    ax9.set_title("Knee Motor Current Feedback", fontsize=13, fontweight="bold")
    ax9.legend(loc="upper right", framealpha=0.9)
    ax9.grid(True, linestyle=":", alpha=0.6)

    # ============================================================
    # 坐标范围与刻度设置
    # ============================================================
    for ax in axes:
        ax.set_xlim(TIME_START, TIME_END)
        ax.xaxis.set_major_locator(MultipleLocator(1.0))
        ax.xaxis.set_minor_locator(MultipleLocator(0.5))  # 次刻度间隔调大，避免数量超限

    auto_ylim(ax1, speed["time"], speed["data"])
    auto_ylim(ax2, angle["time"], angle["data"])
    auto_ylim(ax3, gyro["time"], gyro["data"])
    auto_ylim(ax4, current["tl"], current["left"])

    if plotted_motor_data:
        all_motor_time = np.concatenate(
            [motor["time"]] * len(plotted_motor_data)
        )
        auto_ylim(
            ax5,
            all_motor_time,
            np.concatenate(plotted_motor_data)
        )

    if len(leg_diag.get("time", [])) > 0:
        # 第8张图统一自动缩放
        height_series = []
        for key in ["height", "left_height", "right_height"]:
            if key in leg_diag and len(leg_diag[key]) > 0:
                height_series.append(leg_diag[key])
        if height_series:
            all_height_time = np.concatenate(
                [leg_diag["time"]] * len(height_series)
            )
            auto_ylim(
                ax8,
                all_height_time,
                np.concatenate(height_series)
            )

        # 第9张图统一自动缩放
        current_series = []
        if len(leg_diag.get("knee_left_current", [])) > 0:
            current_series.append(leg_diag["knee_left_current"])
        if len(leg_diag.get("knee_right_current", [])) > 0:
            current_series.append(-leg_diag["knee_right_current"])
        if current_series:
            all_knee_current_time = np.concatenate(
                [leg_diag["time"]] * len(current_series)
            )
            auto_ylim(
                ax9,
                all_knee_current_time,
                np.concatenate(current_series)
            )

    output = base_path + "all_metrics_comparison.png"
    plt.savefig(output, dpi=300, bbox_inches="tight")
    print("Saved figure successfully:", output)
    plt.show()

if __name__ == "__main__":
    main()
