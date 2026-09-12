# bbot_real 双轮足平衡机器人

ROS2 工作区，包含 PID / LQR 两套平衡控制器、CAN 电机驱动、IMU 与遥控接收节点。

目录结构：

```
src/bbot_real          主功能包（控制器 + 驱动 + 数据记录）
src/bbot_rc_receiver   CRSF 遥控接收节点
src/wit_ros2_imu       WIT IMU 驱动
```

---

## 1. CAN 总线初始化（上电后执行一次）

```bash
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

## 2. 编译

```bash
cd /home/robot/bbot_real
source install/setup.bash
colcon build
```

## 3. 启动机器人

### 一键启动（控制器 + IMU + 遥控接收）

```bash
source install/setup.bash
ros2 launch bbot_real bringup.launch.py controller:=lqr   # LQR 控制器
ros2 launch bbot_real bringup.launch.py controller:=pid   # PID 控制器
```

### 或单独启动控制器（带 IMU + 遥控接收 + 延迟启动）

```bash
ros2 launch bbot_real pid_balance.launch.py
ros2 launch bbot_real lqr_balance.launch.py
ros2 launch bbot_real pid_balance.launch.py can_interface:=can0
```

### 键盘控制（PID 控制器窗口内直接按）

| 按键 | 功能 |
|---|---|
| W / S | 前进 / 后退 |
| A / D | 左转 / 右转 |
| Space | 停止移动与转向 |
| ↑ / ↓ | 升高 / 降低机身 |

### 配套节点（另开终端）

```bash
# IMU
ros2 launch wit_ros2_imu rviz_and_imu.launch.py

# 遥控器接收
ros2 run bbot_rc_receiver rc_node

# Foxglove 可视化桥接
ros2 run foxglove_bridge foxglove_bridge
```

---

## 4. 电机测试

```bash
# 单腿测试（默认）
ros2 run bbot_real single_leg_test_node

# 左腿
ros2 run bbot_real single_leg_test_node --ros-args -p can_interface:=can0 -p leg_control_mode:=servo -p hip_motor_id:=1 -p knee_motor_id:=2

# 右腿
ros2 run bbot_real single_leg_test_node --ros-args -p can_interface:=can0 -p leg_control_mode:=servo -p hip_motor_id:=3 -p knee_motor_id:=4

# 双腿测试
ros2 run bbot_real dual_leg_test_node

# 轮毂电机测试
ros2 run bbot_real wheel_motor_test_node
ros2 run bbot_real wheel_motor_test_node --ros-args -p can_interface:=can0 -p wheel_motor_node_id:=5
```

---

## 5. 零点标定

```bash
# 1. 确保 CAN 已配置
sudo ip link set can0 up type can bitrate 1000000

# 2. 运行标定工具
cd /home/robot/bbot_real
source install/setup.bash
ros2 run bbot_real calibrate_motors

# 3. 按提示操作：手动把双腿掰直（小腿+大腿成直线，垂直于地面），按回车确认
```

关节电机 CAN ID 分配：左腿 1(髋) 2(膝)，右腿 3(髋) 4(膝)，轮毂电机 5。

关节方向符号约定：左腿 +-，右腿 -+。

---

## 6. 数据记录与画图

控制器运行时自动把日志写入 `src/bbot_real/src/data_logs/`（angle / speed / gyro / 电流 / 温度 / 腿部诊断等 txt 文件）。

### 离线画图（合并了原 data_plots.py ~ data_plots5.py）

```bash
cd /home/robot/bbot_real/src/bbot_real/src/data_logs

python3 plot_data.py --all --stats              # 全部 9 个子图 + RMSE/MAE 统计框
python3 plot_data.py --balance --stats          # 只画平衡跟踪（速度/俯仰/角速度/轮电流）
python3 plot_data.py --torque                   # 四关节力矩
python3 plot_data.py --temp                     # 关节电机线圈/MOS 温度
python3 plot_data.py --legs                     # 腿高 + 膝关节电流反馈
python3 plot_data.py --steering                 # 转向环（横摆角/角速度跟踪 + 差动电流）

# 指定时间窗口 / 输出文件名 / 数据目录
python3 plot_data.py --balance --start 10 --end 40 --out run1.png
python3 plot_data.py --all --dir /path/to/logs --out run2.png
```

不指定 `--start/--end` 时默认覆盖全部数据；输出默认保存为数据目录下的 `plot_data.png`。

### 遥测话题（Foxglove 用）

话题：`/bbot/telemetry`（`std_msgs/Float64MultiArray`，200Hz）

| 索引 | 含义 | 索引 | 含义 |
|---|---|---|---|
| data[0] | 实际前进速度 (m/s) | data[14] | 右腿高 (m) |
| data[1] | 目标前进速度 (m/s) | data[15] | 左膝电流反馈 |
| data[2] | 实际俯仰角 Pitch (rad) | data[16] | 右膝电流反馈 |
| data[3] | 目标俯仰角 (rad) | data[17] | 左膝力矩系数 Kt |
| data[4] | 实际角速度 Pitch Rate | data[18] | 右膝力矩系数 Kt |
| data[5] | 目标角速度 | data[19] | 航向角 Yaw (rad) |
| data[6] | 左轮电流 (mA) | data[20] | Heading Hold 目标航向 (rad) |
| data[7] | 右轮电流 (取反, mA) | data[21] | 实际 Yaw 角速度 (rad/s) |
| data[8] | 左髋力矩 (Nm) | data[22] | 目标 Yaw 角速度 (rad/s) |
| data[9] | 左膝力矩 (Nm) | data[23] | Yaw 差动电流 (mA) |
| data[10] | 右髋力矩 (Nm) | data[24] | 航向误差 (rad) |
| data[11] | 右膝力矩 (取反, Nm) | data[25] | 目标曲率 κ (1/m) |
| data[12] | 中心腿高 (m) | data[26] | 转向用速度幅值 (m/s) |
| data[13] | 左腿高 (m) | data[27] | Heading Hold 使能 (0/1) |
| data[28] | 实际横滚角 Roll (rad) | | |

### 遥控器回传（CRSF 遥测）

`rc_node` 订阅 `/bbot/telemetry`，把真实状态回传给遥控器屏幕显示，只回传四个量：

| 遥控器传感器 | CRSF 帧 | 内容 |
|---|---|---|
| P / R（姿态） | ATTITUDE 10Hz | pitch、roll（度） |
| GSpd（GPS速度） | GPS 2Hz | 机器人速度 m/s→km/h（0.1km/h 分辨率） |
| Alt（气压高度） | BARO_ALTITUDE 2Hz | 当前腿高 m（0.1m 分辨率） |

电池 / 链路统计 / 心跳 / 飞行模式等其余帧类型全部停发。

---

## 7. CAN 调试工具

```bash
# 查看CAN帧信息
candump -x can0
cansend can0 123#DEADBEEF

# 单独设置波特率 / 启动网卡
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 查看串口（IMU / 遥控接收机）
ls /dev/ttyCH341USB*
```

---

## 8. 日常 Git 流程

```bash
# 0. 多端开发时，先拉取最新代码避免冲突
git pull

# 1. 查看本地修改状态
git status

# 2. 添加修改到暂存区
git add .                        # 添加所有修改和新文件
git add src/your_file.cpp        # 或只添加特定文件

# 3. 提交并添加说明
git commit -m "更新说明（例如：修改平衡控制算法参数）"

# 4. 推送到远程仓库（日常直接 push，不需要 -u / -f）
git push
```
