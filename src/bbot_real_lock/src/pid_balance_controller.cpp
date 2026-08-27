#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "bbot_real/pid.hpp"
#include "bbot_real/kinematics.hpp"
#include "bbot_real/can_interface.hpp"
#include "bbot_real/joint_motor_driver.hpp"
#include "bbot_real/wheel_motor_driver.hpp"

#include <fstream>
#include <filesystem>

using namespace std::chrono_literals;

//   工具函数

static double clamp_value(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

static double low_pass_filter(double new_val, double old_val, double alpha)
{
    return alpha * new_val + (1.0 - alpha) * old_val;
}

static double lerp(double a, double b, double t)
{
    return a + (b - a) * t;
}

//   PID参数结构

struct PIDParam
{
    float p, i, d, ramp, limit;
};

//   主控制器

class PIDBalanceController : public rclcpp::Node
{
public:
    PIDBalanceController()
        : Node("pid_balance_controller")
    {
        // ---- PID参数（站立 / 下蹲）----
        PIDParam speed_stand = {0.30f, 0.015f, 0.005f, 0.0f, 0.50f};
        PIDParam speed_squat = {0.25f, 0.010f, 0.002f, 0.0f, 0.45f};

        PIDParam angle_stand = {7.0f, 0.0f, 0.05f, 0.0f, 2.5f};
        PIDParam angle_squat = {6.0f, 0.0f, 0.03f, 0.0f, 2.2f};

        PIDParam gyro_stand = {2.2f, 0.0f, 0.005f, 10.0f, 5.0f};
        PIDParam gyro_squat = {1.5f, 0.0f, 0.005f, 10.0f, 4.5f};

        // PIDParam speed_stand = {0.0f, 0.0f, 0.0f, 0.0f, 0.50f};
        // PIDParam speed_squat = {0.0f, 0.0f, 0.0f, 0.0f, 0.45f};

        // PIDParam angle_stand = {0.0f, 0.0f, 0.0f, 0.0f, 2.5f};
        // PIDParam angle_squat = {0.0f, 0.0f, 0.0f, 0.0f, 2.2f};

        // PIDParam gyro_stand = {0.0f, 0.0f, 0.0f, 10.0f, 5.0f};
        // PIDParam gyro_squat = {0.0f, 0.0f, 0.0f, 10.0f, 4.5f};

        pid_speed_ = bbot_real::PIDController(speed_stand.p, speed_stand.i, speed_stand.d,
                                              speed_stand.ramp, speed_stand.limit);
        pid_angle_ = bbot_real::PIDController(angle_stand.p, angle_stand.i, angle_stand.d,
                                              angle_stand.ramp, angle_stand.limit);
        pid_gyro_ = bbot_real::PIDController(gyro_stand.p, gyro_stand.i, gyro_stand.d,
                                             gyro_stand.ramp, gyro_stand.limit);

        // ---- 平衡参数 ----
        balance_offset_ = 0.0;
        cmd_sign_ = -1.0;
        max_cmd_x_ = 5.0;
        max_safe_pitch_ = 1.20;

        walk_speed_ = 0.3;
        turn_speed_ = 0.5;
        speed_ramp_time_ = 1.0;

        L_MIN_ = 0.25;
        L_MAX_ = 0.50;
        current_height_ = L_MAX_;
        target_height_ = L_MAX_;
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0;

        // ---- 腿部控制模式 ----
        this->declare_parameter<std::string>("leg_control_mode", "servo");
        leg_mode_ = this->get_parameter("leg_control_mode").as_string();
        RCLCPP_INFO(this->get_logger(), "腿部控制模式: %s", leg_mode_.c_str());

        // ---- 锁腿模式（固定关节角度，跳过IK）----
        this->declare_parameter<bool>("fix_legs", false);
        this->declare_parameter<double>("hip_angle_deg", 20.0);
        this->declare_parameter<double>("knee_angle_deg", -20.0);

        fix_legs_ = this->get_parameter("fix_legs").as_bool();
        hip_angle_fixed_ = this->get_parameter("hip_angle_deg").as_double() * M_PI / 180.0;
        knee_angle_fixed_ = this->get_parameter("knee_angle_deg").as_double() * M_PI / 180.0;

        if (fix_legs_)
            RCLCPP_INFO(this->get_logger(), "锁腿模式: hip=%.1f° knee=%.1f°",
                        this->get_parameter("hip_angle_deg").as_double(),
                        this->get_parameter("knee_angle_deg").as_double());

        // 力位混合控制参数（仅 hybrid 模式使用）
        leg_kp_ = 30.0f;
        leg_kd_ = 1.0f;

        // ---- CAN总线 ----
        can_ = std::make_shared<bbot_real::CanInterface>();
        this->declare_parameter<std::string>("can_interface", "can0");
        std::string can_if = this->get_parameter("can_interface").as_string();
        try
        {
            can_->open(can_if);
            RCLCPP_INFO(this->get_logger(), "CAN接口 '%s' 打开成功", can_if.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "CAN打开失败 '%s': %s（将以无CAN模式运行）",
                        can_if.c_str(), e.what());
        }

        // ---- 关节电机初始化 ----
        this->declare_parameter<int>("motor_left_hip_id", 1);
        this->declare_parameter<int>("motor_left_knee_id", 2);
        this->declare_parameter<int>("motor_right_hip_id", 3);
        this->declare_parameter<int>("motor_right_knee_id", 4);

        motor_left_hip_.init(can_, this->get_parameter("motor_left_hip_id").as_int(), 75.0);
        motor_left_knee_.init(can_, this->get_parameter("motor_left_knee_id").as_int(), 60.0);
        motor_right_hip_.init(can_, this->get_parameter("motor_right_hip_id").as_int(), 75.0);
        motor_right_knee_.init(can_, this->get_parameter("motor_right_knee_id").as_int(), 60.0);

        // 使能所有关节电机
        motor_left_hip_.enable();
        motor_left_knee_.enable();
        motor_right_hip_.enable();
        motor_right_knee_.enable();
        joints_enabled_ = true;

        // ---- 轮毂电机（ZLAC CANopen双轴驱动器）----
        this->declare_parameter<int>("wheel_node_id", 5);
        wheel_node_id_ = this->get_parameter("wheel_node_id").as_int();
        wheel_.init(can_, wheel_node_id_);
        if (can_->is_open() && !wheel_.enable())
            RCLCPP_WARN(this->get_logger(), "轮毂电机使能失败，请检查ZLAC驱动器");
        RCLCPP_INFO(this->get_logger(), "轮毂电机 ZLAC node=%d", wheel_node_id_);

        // ---- 轮毂物理参数 ----
        this->declare_parameter<double>("wheel_radius", 0.07); // 默认 70mm
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        RCLCPP_INFO(this->get_logger(), "轮毂半径: %.3f m", wheel_radius_);

        // ---- 订阅 ----
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), 10,
            std::bind(&PIDBalanceController::imu_callback, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input", 10,
            std::bind(&PIDBalanceController::joy_callback, this, std::placeholders::_1));

        // ---- 发布（调试/回退用）----
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        // ---- 控制定时器 200Hz ----
        timer_ = this->create_wall_timer(5ms, std::bind(&PIDBalanceController::control_loop, this));
        last_time_ = this->now();

        // 初始化数据日志
        std::string log_dir = "/home/robot/bbot_real/src/bbot_real/src/data_logs/";
        try
        {
            std::filesystem::create_directories(log_dir);
            log_angle_.open(log_dir + "angle_data.txt");
            log_target_angle_.open(log_dir + "target_angle_data.txt");
            log_timestamp_angle_.open(log_dir + "timestamp_angle.txt");
            log_timestamp_target_angle_.open(log_dir + "timestamp_target_angle.txt");

            log_speed_.open(log_dir + "speed_data.txt");
            log_target_speed_.open(log_dir + "target_speed_data.txt");
            log_timestamp_speed_.open(log_dir + "timestamp_speed.txt");
            log_timestamp_target_speed_.open(log_dir + "timestamp_target_speed.txt");

            log_gyro_.open(log_dir + "gyro_data.txt");
            log_target_gyro_.open(log_dir + "target_gyro_data.txt");
            log_timestamp_gyro_.open(log_dir + "timestamp_gyro.txt");
            log_timestamp_target_gyro_.open(log_dir + "timestamp_target_gyro.txt");
            logging_enabled_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "数据日志文件打开失败: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "PID平衡控制器启动完成（遥控+位置控制模式）");
    }

    ~PIDBalanceController()
    {
        RCLCPP_INFO(this->get_logger(), "控制器正在退出，释放电机...");

        // 1. 先将轮子输出力矩清零
        wheel_.set_torque(0, 0);
        std::this_thread::sleep_for(10ms);

        // 2. 发送急停及去使能报文
        wheel_.emergency_stop();
        motor_left_hip_.disable();
        motor_left_knee_.disable();
        motor_right_hip_.disable();
        motor_right_knee_.disable();

        // 3. 延时 100ms 留出缓冲发送时间，解决退出后抱死问题
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        can_->close();

        if (log_angle_.is_open())
            log_angle_.close();
        if (log_target_angle_.is_open())
            log_target_angle_.close();
        if (log_timestamp_angle_.is_open())
            log_timestamp_angle_.close();
        if (log_timestamp_target_angle_.is_open())
            log_timestamp_target_angle_.close();
        if (log_speed_.is_open())
            log_speed_.close();
        if (log_target_speed_.is_open())
            log_target_speed_.close();
        if (log_timestamp_speed_.is_open())
            log_timestamp_speed_.close();
        if (log_timestamp_target_speed_.is_open())
            log_timestamp_target_speed_.close();
        if (log_gyro_.is_open())
            log_gyro_.close();
        if (log_target_gyro_.is_open())
            log_target_gyro_.close();
        if (log_timestamp_gyro_.is_open())
            log_timestamp_gyro_.close();
        if (log_timestamp_target_gyro_.is_open())
            log_timestamp_target_gyro_.close();
    }

private:
    //   遥控器回调
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 3 || msg->buttons.size() < 3)
            return;

        double ly_roll = msg->axes[0];  // 转向
        double lx_pitch = msg->axes[1]; // 前进/后退
        double aux1 = msg->axes[2];     // 腿高度
        int aux2_btn = msg->buttons[2]; // 急停按钮

        // 急停
        if (aux2_btn == 1)
        {
            is_emergency_stopped_ = true;

            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;

            pid_speed_.reset();
            pid_angle_.reset();
            pid_gyro_.reset();
            return;
        }

        // 速度控制
        target_speed_const_ = (std::abs(lx_pitch) > 0.05) ? lx_pitch * walk_speed_ : 0.0;
        // 转向控制
        target_yaw_rate_ = (std::abs(ly_roll) > 0.05) ? -ly_roll * turn_speed_ : 0.0;
        // 腿高度控制
        target_height_ = L_MIN_ + ((aux1 + 1.0) / 2.0) * (L_MAX_ - L_MIN_);
    }

    //   IMU回调
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 只在收到第一帧数据时打印一次
        RCLCPP_INFO_ONCE(this->get_logger(), "已成功接收到第一帧 IMU 数据！");

        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        pitch_ = pitch;
        pitch_rate_raw_ = msg->angular_velocity.y;

        if (!pitch_rate_filter_init_)
        {
            pitch_rate_filt_ = pitch_rate_raw_;
            pitch_rate_filter_init_ = true;
        }
        else
        {
            pitch_rate_filt_ = low_pass_filter(pitch_rate_raw_, pitch_rate_filt_, pitch_rate_alpha_);
        }
        pitch_rate_ = pitch_rate_filt_;
        imu_received_ = true;
    }

    //   控制主循环 200Hz
    void control_loop()
    {
        if (!imu_received_)
            return;

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0001)
            dt = 0.005;
        if (dt > 0.05)
            dt = 0.005;

        // 安全停机
        if (std::abs(pitch_) > max_safe_pitch_ || is_emergency_stopped_)
        {
            wheel_.emergency_stop();

            publish_cmd(0.0, 0.0);

            motor_left_hip_.disable();
            motor_left_knee_.disable();
            motor_right_hip_.disable();
            motor_right_knee_.disable();

            joints_enabled_ = false; // 标记关节已失能

            pid_speed_.reset();
            pid_angle_.reset();
            pid_gyro_.reset();

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "急停...");
            return;
        }
        else
        {
            // 机器人处于安全俯仰角范围内，且关节未使能时，自动重新使能并锁定腿部
            if (!joints_enabled_)
            {
                RCLCPP_INFO(this->get_logger(), "进入安全角度，正在重新锁定/使能全关节电机...");
                motor_left_hip_.enable();
                std::this_thread::sleep_for(10ms);
                motor_left_knee_.enable();
                std::this_thread::sleep_for(10ms);
                motor_right_hip_.enable();
                std::this_thread::sleep_for(10ms);
                motor_right_knee_.enable();
                std::this_thread::sleep_for(10ms);
                joints_enabled_ = true;
            }
        }

        // 以 50Hz 的频率 (每 4 个周期一次) 通过 SDO 读取实际轮速，避免阻塞 200Hz 核心控制
        if (speed_read_counter_++ % 4 == 0)
        {
            double left_rpm = 0.0;
            double right_rpm = 0.0;
            if (wheel_.read_motor_rpms(left_rpm, right_rpm))
            {
                // 将 RPM 转换为 m/s: RPM * 2 * pi / 60 * R
                double left_mps = (left_rpm * 2.0 * M_PI / 60.0) * wheel_radius_;
                double right_mps = (right_rpm * 2.0 * M_PI / 60.0) * wheel_radius_;

                // 右轮反向安装，做差求均值
                double current_x_dot = (left_mps - right_mps) / 2.0;

                // 使用低通滤波器平滑反馈速度
                x_dot_ = low_pass_filter(current_x_dot, x_dot_, 0.20);
            }
        }

        if (!fix_legs_)
        {
            update_leg_height(dt);
            interpolate_pid_gains();
        }

        // 速度斜坡
        double ramp_step = dt / speed_ramp_time_;
        if (target_speed_smoothed_ < target_speed_const_)
            target_speed_smoothed_ = std::min(target_speed_smoothed_ + ramp_step, target_speed_const_);
        else if (target_speed_smoothed_ > target_speed_const_)
            target_speed_smoothed_ = std::max(target_speed_smoothed_ - ramp_step, target_speed_const_);
        double target_speed = target_speed_smoothed_;

        // ===== 三级PID级联 =====
        // 1. 速度环 → 目标倾角
        double vel_error = target_speed - x_dot_;
        double target_pitch = pid_speed_(vel_error, dt);
        target_pitch = clamp_value(target_pitch, -0.25, 0.25);
        target_pitch_filtered_ = low_pass_filter(target_pitch, target_pitch_filtered_, 0.35);
        target_pitch_filtered_ += balance_offset_;

        // 2. 角度环 → 目标角速度
        double angle_error = target_pitch_filtered_ - pitch_;
        double target_pitch_rate = pid_angle_(angle_error, dt);
        target_pitch_rate = clamp_value(target_pitch_rate, -3.2, 3.2);

        // 3. 角速度环 → 轮速指令
        double gyro_error = target_pitch_rate - pitch_rate_;
        double cmd_raw = pid_gyro_(gyro_error, dt);

        double cmd_x = clamp_value(cmd_raw * cmd_sign_, -max_cmd_x_, max_cmd_x_);

        // 输出
        publish_cmd(cmd_x, target_yaw_rate_);
        send_wheel_can(cmd_x, target_yaw_rate_);
        send_leg_can();

        // 正常控制周期结束后记录俯仰角、速度及角速度的实际值与期望值
        if (logging_enabled_)
        {
            double t = now.seconds();
            log_angle_ << pitch_ << "\n";
            log_target_angle_ << target_pitch_filtered_ << "\n";
            log_timestamp_angle_ << t << "\n";
            log_timestamp_target_angle_ << t << "\n";

            log_speed_ << x_dot_ << "\n";
            log_target_speed_ << target_speed << "\n";
            log_timestamp_speed_ << t << "\n";
            log_timestamp_target_speed_ << t << "\n";

            log_gyro_ << pitch_rate_ << "\n";
            log_target_gyro_ << target_pitch_rate << "\n";
            log_timestamp_gyro_ << t << "\n";
            log_timestamp_target_gyro_ << t << "\n";
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "[PID] pitch=%.3f vel=%.2f cmd=%.2f h=%.3f",
                             pitch_, x_dot_, cmd_x, current_height_);
    }

    //   腿部高度
    void update_leg_height(double dt)
    {
        double step = leg_transition_speed_ * dt;
        if (current_height_ > target_height_)
            current_height_ = std::max(current_height_ - step, target_height_);
        else if (current_height_ < target_height_)
            current_height_ = std::min(current_height_ + step, target_height_);
        current_height_ = clamp_value(current_height_, L_MIN_, L_MAX_);
    }

    double height_ratio()
    {
        return clamp_value((current_height_ - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
    }

    //   PID增益插值
    void interpolate_pid_gains()
    {
        double r = height_ratio();
        pid_speed_.P = lerp(0.25f, 0.30f, r);
        pid_speed_.I = lerp(0.010f, 0.015f, r);
        pid_speed_.D = lerp(0.002f, 0.005f, r);
        pid_speed_.limit = lerp(0.45f, 0.50f, r);

        pid_angle_.P = lerp(6.0f, 7.0f, r);
        pid_angle_.D = lerp(0.03f, 0.05f, r);
        pid_angle_.limit = lerp(2.2f, 2.5f, r);

        pid_gyro_.P = lerp(1.5f, 2.2f, r);
        pid_gyro_.D = lerp(0.005f, 0.005f, r);
        pid_gyro_.limit = lerp(4.5f, 5.0f, r);
    }

    //   腿部CAN发送（位置控制）
    void send_leg_can()
    {
        double hip_angle, knee_angle;

        if (fix_legs_)
        {
            // 锁腿模式：跳过 IK，直接使用固定角度
            hip_angle = hip_angle_fixed_;
            knee_angle = knee_angle_fixed_;
        }
        else
        {
            // 正常模式：通过 IK 逆解计算关节角度
            double x_off = lerp(0.032, 0.015, height_ratio());
            bbot_real::IKSolution ik = kinematics_.inverse_kinematics(current_height_, 0.0, x_off);

            hip_angle = -ik.theta_hip;
            knee_angle = -ik.theta_knee;

            // 增加对 NaN 值的安全过滤保护
            if (std::isnan(hip_angle) || std::isnan(knee_angle))
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                      "逆解计算输出 NaN！请检查高度极限 (h=%.3f, x_off=%.3f)", current_height_, x_off);
                return; // 放弃本次发送，避免电机软掉或报错
            }
        }

        // ROS发布（调试用）
        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {hip_angle, knee_angle, hip_angle, knee_angle};
        leg_pub_->publish(leg_cmd);

        if (leg_mode_ == "servo")
        {
            // 纯伺服位置控制（模式0x01）：只发角度，电机内部PID
            double hip_deg = hip_angle * 180.0 / M_PI;
            double knee_deg = knee_angle * 180.0 / M_PI;
            motor_left_hip_.set_servo_position(hip_deg);
            motor_left_knee_.set_servo_position(knee_deg);
            motor_right_hip_.set_servo_position(-hip_deg);
            motor_right_knee_.set_servo_position(-knee_deg);
        }
        else // hybrid
        {
            // 力位混合控制（模式0x00）：位置指令 + 重力补偿前馈
            double hip_ff = 0.0, knee_ff = 0.0;
            if (!fix_legs_)
            {
                // 正常模式：基于 IK 结果计算重力补偿
                double x_off = lerp(0.032, 0.015, height_ratio());
                bbot_real::IKSolution ik = kinematics_.inverse_kinematics(current_height_, 0.0, x_off);
                bbot_real::JointTorques torques = kinematics_.compute_gravity_torques(
                    0.0, ik.theta_hip, ik.theta_knee);
                hip_ff = torques.hip_torque / 2.0;
                knee_ff = torques.knee_torque / 2.0;
            }
            // 锁腿模式下前馈为 0，纯位置保持

            motor_left_hip_.set_position(hip_angle, 0.0, hip_ff, leg_kp_, leg_kd_);
            motor_left_knee_.set_position(knee_angle, 0.0, knee_ff, leg_kp_, leg_kd_);
            motor_right_hip_.set_position(-hip_angle, 0.0, hip_ff, leg_kp_, leg_kd_);
            motor_right_knee_.set_position(-knee_angle, 0.0, knee_ff, leg_kp_, leg_kd_);
        }
    }

    void send_wheel_can(double cmd_x, double yaw_rate)
    {
        int16_t base_torque = static_cast<int16_t>(cmd_x * 1000.0);
        int16_t diff_torque = static_cast<int16_t>(yaw_rate * 500.0); // 500为转向增益，需调试

        wheel_.set_torque(base_torque + diff_torque, -base_torque + diff_torque);
    }

    void publish_cmd(double vx, double vz)
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = vx;
        cmd.twist.angular.z = vz;
        cmd_pub_->publish(cmd);
    }

    // 成员变量

    // CAN与电机
    std::shared_ptr<bbot_real::CanInterface> can_;
    bbot_real::JointMotorDriver motor_left_hip_, motor_left_knee_;
    bbot_real::JointMotorDriver motor_right_hip_, motor_right_knee_;
    bbot_real::WheelMotorDriver wheel_;
    int wheel_node_id_;
    bbot_real::Kinematics kinematics_;

    // PID
    bbot_real::PIDController pid_speed_, pid_angle_, pid_gyro_;

    // 订阅/发布
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // IMU状态
    bool imu_received_ = false;
    double pitch_ = 0.0, pitch_rate_ = 0.0;
    double pitch_rate_raw_ = 0.0, pitch_rate_filt_ = 0.0;
    bool pitch_rate_filter_init_ = false;
    double pitch_rate_alpha_ = 0.20;

    // 轮速
    double x_dot_ = 0.0;
    int speed_read_counter_ = 0; // 类内私有计数器，取代原有的 static 局部变量

    // 轮子物理参数
    double wheel_radius_;

    // 平衡参数
    double balance_offset_, cmd_sign_ = -1.0, max_cmd_x_, max_safe_pitch_;
    double target_pitch_filtered_ = 0.0;

    // 遥控指令
    double target_speed_const_ = 0.0, target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_, turn_speed_, speed_ramp_time_;

    // 腿部高度
    double L_MIN_, L_MAX_, current_height_, target_height_, leg_transition_speed_;

    // 腿部控制
    std::string leg_mode_;  // "servo" 或 "hybrid"
    float leg_kp_, leg_kd_; // 仅 hybrid 模式使用

    // 锁腿模式
    bool fix_legs_ = false;
    double hip_angle_fixed_ = 0.0;  // rad
    double knee_angle_fixed_ = 0.0; // rad

    bool is_emergency_stopped_ = false;

    // 数据日志文件流
    std::ofstream log_angle_, log_target_angle_, log_timestamp_angle_, log_timestamp_target_angle_;
    std::ofstream log_speed_, log_target_speed_, log_timestamp_speed_, log_timestamp_target_speed_;
    std::ofstream log_gyro_, log_target_gyro_, log_timestamp_gyro_, log_timestamp_target_gyro_;
    bool logging_enabled_ = false;

    bool joints_enabled_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDBalanceController>());
    rclcpp::shutdown();
    return 0;
}