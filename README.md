# 启动命令
## 1. CAN总线初始化（启动时执行一次）

sudo ip link set can0 up type can bitrate 1000000

## 2. 编译

cd /home/ubuntu2404/bbot_real
source install/setup.bash
colcon build

## 3. 一键启动（LQR控制器 + IMU + RC接收）

source install/setup.bash
ros2 launch bbot_real bringup.launch.py controller:=lqr

## 4. 或单独启动PID控制器

ros2 launch bbot_real pid_balance.launch.py
ros2 launch bbot_real pid_balance.launch.py can_interface:=can0

---

## CAN
### 1. 设置 can0 波特率为 1000000 (1M)，系统默认就是正常通信模式
sudo ip link set can0 type can bitrate 1000000

### 2. 启动 can0 网卡
sudo ip link set can0 up

### 3. 查看CAN帧信息
candump -x can0
cansend can0 123#DEADBEEF

---
查看串口
ls /dev/ttyCH341USB*
---

---

单腿测试
ros2 run bbot_real single_leg_test_node
左腿
ros2 run bbot_real single_leg_test_node --ros-args -p can_interface:=can0 -p leg_control_mode:=servo -p hip_motor_id:=1 -p knee_motor_id:=2

右腿
ros2 run bbot_real single_leg_test_node --ros-args -p can_interface:=can0 -p leg_control_mode:=servo -p hip_motor_id:=3 -p knee_motor_id:=4

双腿测试
ros2 run bbot_real dual_leg_test_node 

轮子测试
ros2 run bbot_real wheel_motor_test_node
ros2 run bbot_real wheel_motor_test_node --ros-args -p can_interface:=can0 -p wheel_motor_node_id:=5

---

## 设置零点

### 1. 确保CAN已配置

sudo ip link set can0 up type can bitrate 1000000

### 2. 运行标定工具

cd /home/ubuntu2404/bbot_real
source install/setup.bash
ros2 run bbot_real calibrate_motors

### 3. 按提示操作：

手动把双腿掰直（小腿+大腿成直线，垂直于地面）

### 4. 按回车确认

---

左腿：+-
右腿：-+

---
启动imu
ros2 launch wit_ros2_imu rviz_and_imu.launch.py

启动遥控器
ros2 run bbot_rc_receiver rc_node

---
日常更新和提交代码的标准流程如下：

**1. 查看本地修改状态**

```bash
git status

```

*用于确认哪些文件被修改、添加或删除。*

---

**2. 添加修改到暂存区**

```bash
# 添加所有修改和新文件
git add .

# 或者只添加特定文件/目录
git add src/your_file.cpp

```

---

**3. 提交修改并添加说明**

```bash
git commit -m "更新说明（例如：修改平衡控制算法参数）"

```

---

**4. 推送到远程 GitHub 仓库**

```bash
git push

```

*(日常更新直接使用 `git push` 即可，不需要再加 `-u` 或 `-f` 强推参数)*

---

**日常协作/多端开发时的补充操作：**

如果在其他电脑上也有修改，在本地修改前先拉取远程最新代码以避免冲突：

```bash
git pull

```