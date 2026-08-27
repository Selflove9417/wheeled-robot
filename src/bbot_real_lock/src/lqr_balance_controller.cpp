#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "bbot_real/kinematics.hpp"
#include "bbot_real/can_interface.hpp"
#include "bbot_real/joint_motor_driver.hpp"
#include "bbot_real/wheel_motor_driver.hpp"

#include <fstream>
#include <filesystem>

using namespace std::chrono_literals;

// ==================== 工具函数 ====================

static double clamp_value(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

static double low_pass_filter(double new_val, double old_val, double alpha)
{
    return alpha * new_val + (1.0 - alpha) * old_val;
}

static double lerp(double a, double b, double t) { return a + (b - a) * t; }

// ==================== LQR增益结构 ====================

struct LQRGain
{
    double k_x, k_x_dot, k_theta, k_theta_dot;
};

// ==================== 主控制器 ====================

class LQRBalanceController : public rclcpp::Node
{
public:
    LQRBalanceController()
        : Node("lqr_balance_controller")
    {
        // ---- LQR增益（下蹲 / 站立）----
        gain_low_ = {-20.00, -30.40, -131.98, -25.46};
        gain_high_ = {-25.00, -30.78, -157.82, -36.45};
        current_gain_ = gain_high_;

        // ---- 平衡参数 ----
        balance_offset_ = 0.005;
        cmd_scale_ = 0.1;
        cmd_sign_ = 1.0;
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

        // ---- 关节电机 ----
        this->declare_parameter<int>("motor_left_hip_id", 1);
        this->declare_parameter<int>("motor_left_knee_id", 2);
        this->declare_parameter<int>("motor_right_hip_id", 3);
        this->declare_parameter<int>("motor_right_knee_id", 4);

        motor_left_hip_.init(can_, this->get_parameter("motor_left_hip_id").as_int(), 75.0);
        motor_left_knee_.init(can_, this->get_parameter("motor_left_knee_id").as_int(), 60.0);
        motor_right_hip_.init(can_, this->get_parameter("motor_right_hip_id").as_int(), 75.0);
        motor_right_knee_.init(can_, this->get_parameter("motor_right_knee_id").as_int(), 60.0);

        // 初始进行安全配置，不在这里一次性使能，避免冲突与误触发
        joints_enabled_ = false;

        // ---- 轮毂电机（ZLAC CANopen双轴驱动器）----
        this->declare_parameter<int>("wheel_node_id", 5);
        wheel_node_id_ = this->get_parameter("wheel_node_id").as_int();
        wheel_.init(can_, wheel_node_id_);
        if (can_->is_open() && !wheel_.enable())
            RCLCPP_WARN(this->get_logger(), "轮毂电机使能失败，请检查ZLAC驱动器");
        RCLCPP_INFO(this->get_logger(), "轮毂电机 ZLAC node=%d", wheel_node_id_);

        // ---- 轮毂物理参数 ----
        this->declare_parameter<double>("wheel_radius", 0.075); // LQR轮半径参数化，默认75mm
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        RCLCPP_INFO(this->get_logger(), "轮毂半径: %.3f m", wheel_radius_);

        // ---- 订阅 ----
        // 修改默认话题名称为 /imu/data，匹配您的实际环境
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        std::string target_imu_topic = this->get_parameter("imu_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "正在订阅 IMU 话题: %s", target_imu_topic.c_str());

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            target_imu_topic, 10,
            std::bind(&LQRBalanceController::imu_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input", 10,
            std::bind(&LQRBalanceController::joy_callback, this, std::placeholders::_1));

        // ---- 发布 ----
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        // ---- 控制定时器 200Hz ----
        timer_ = this->create_wall_timer(5ms, std::bind(&LQRBalanceController::control_loop, this));
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

        RCLCPP_INFO(this->get_logger(), "LQR平衡控制器启动完成（遥控+位置控制模式）");
    }

    ~LQRBalanceController()
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
    // ==================== 遥控器回调 ====================
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 3 || msg->buttons.size() < 3)
            return;
        double ly_roll = msg->axes[0];
        double lx_pitch = msg->axes[1];
        double aux1 = msg->axes[2];
        int aux2_btn = msg->buttons[2];

        if (aux2_btn == 1) // 急停
        {
            is_emergency_stopped_ = true;
            target_speed_const_ = 0.0;
            target_yaw_rate_ = 0.0;
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            return;
        }

        target_speed_const_ = (std::abs(lx_pitch) > 0.05) ? lx_pitch * walk_speed_ : 0.0;
        target_yaw_rate_ = (std::abs(ly_roll) > 0.05) ? -ly_roll * turn_speed_ : 0.0;
        target_height_ = L_MIN_ + ((aux1 + 1.0) / 2.0) * (L_MAX_ - L_MIN_);
    }

    // ==================== IMU回调 ====================
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
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

    // ==================== 控制主循环 200Hz ====================
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

        // ---- 1. 安全停机与遥控急停 ----
        if (std::abs(pitch_) > max_safe_pitch_ || is_emergency_stopped_)
        {
            wheel_.emergency_stop();
            publish_cmd(0.0, 0.0);

            motor_left_hip_.disable();
            motor_left_knee_.disable();
            motor_right_hip_.disable();
            motor_right_knee_.disable();

            vel_integral_ = 0.0;
            joints_enabled_ = false; // 重置关节使能标志位
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[安全/急停] 全车进入释放状态...");
            return;
        }
        else
        {
            // ---- 2. 自动使能/锁死状态机 ----
            // 当机器人处于安全姿态范围时，自动重新使能并硬化关节
            if (!joints_enabled_)
            {
                RCLCPP_INFO(this->get_logger(), "进入安全角度，自动使能关节电机锁定高度...");
                motor_left_hip_.enable();
                std::this_thread::sleep_for(15ms);
                motor_left_knee_.enable();
                std::this_thread::sleep_for(15ms);
                motor_right_hip_.enable();
                std::this_thread::sleep_for(15ms);
                motor_right_knee_.enable();
                std::this_thread::sleep_for(15ms);
                joints_enabled_ = true;
            }
        }

        // ---- 3. 正常状态控制逻辑 ----

        // 以 50Hz 的频率 (每 4 个周期一次) 通过 SDO 直接读取物理轮毂电机的实际转速
        if (speed_read_counter_++ % 4 == 0)
        {
            double left_rpm = 0.0;
            double right_rpm = 0.0;
            if (wheel_.read_motor_rpms(left_rpm, right_rpm))
            {
                // 将 RPM 转换为 m/s: RPM * 2 * pi / 60 * R
                double left_mps = (left_rpm * 2.0 * M_PI / 60.0) * wheel_radius_;
                double right_mps = (right_rpm * 2.0 * M_PI / 60.0) * wheel_radius_;

                // 考虑右侧轮反向安装特性，计算机器人实际物理前进线速度
                double current_x_dot = (left_mps - right_mps) / 2.0;

                // 低通滤波平滑轮速
                x_dot_ = low_pass_filter(current_x_dot, x_dot_, 0.20);
            }
        }

        // 位移状态积分
        x_ += x_dot_ * dt;

        update_leg_height(dt);
        if (!fix_legs_)
            interpolate_lqr_gain();

        // 速度斜坡
        double ramp_step = dt / speed_ramp_time_;
        if (target_speed_smoothed_ < target_speed_const_)
            target_speed_smoothed_ = std::min(target_speed_smoothed_ + ramp_step, target_speed_const_);
        else if (target_speed_smoothed_ > target_speed_const_)
            target_speed_smoothed_ = std::max(target_speed_smoothed_ - ramp_step, target_speed_const_);
        double target_speed = target_speed_smoothed_;

        // 位置跟踪
        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005)
        {
            if (was_moving_)
            {
                target_x_ = x_;
                was_moving_ = false;
            }
        }
        else
        {
            target_x_ += target_speed * dt;
            was_moving_ = true;
        }

        // ===== LQR控制核心计算 =====
        double pos_error = x_ - target_x_;
        double vel_error = x_dot_ - target_speed;
        double gyro_val = pitch_rate_;
        double u_pitch = 0.0;
        double dynamic_target_pitch = balance_offset_;

        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005)
        {
            // 模式1: 静止 — 全状态LQR
            double theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(current_gain_.k_x * pos_error +
                        current_gain_.k_x_dot * vel_error +
                        current_gain_.k_theta * theta_error +
                        current_gain_.k_theta_dot * gyro_val);
            vel_integral_ = 0.0;
        }
        else
        {
            // 模式2: 运动 — 外环PI速度 + 内环LQR角度
            double vel_error_v = target_speed - x_dot_;
            vel_integral_ += vel_error_v * dt;
            vel_integral_ = clamp_value(vel_integral_, -0.5, 0.5);
            dynamic_target_pitch = balance_offset_ + (0.25 * vel_error_v + 0.05 * vel_integral_);
            dynamic_target_pitch = clamp_value(dynamic_target_pitch, -0.2, 0.2);
            double theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(current_gain_.k_theta * theta_error +
                        current_gain_.k_theta_dot * gyro_val);
        }

        double cmd_x = clamp_value(u_pitch * cmd_scale_ * cmd_sign_ + target_speed, -max_cmd_x_, max_cmd_x_);

        // 输出与分发
        publish_cmd(cmd_x, target_yaw_rate_);
        send_wheel_can(cmd_x, target_yaw_rate_);
        send_leg_can();

        // 正常控制周期结束后记录状态空间实际值与动态平衡期望值
        if (logging_enabled_)
        {
            double t = now.seconds();
            log_angle_ << pitch_ << "\n";
            log_target_angle_ << dynamic_target_pitch << "\n";
            log_timestamp_angle_ << t << "\n";
            log_timestamp_target_angle_ << t << "\n";

            log_speed_ << x_dot_ << "\n";
            log_target_speed_ << target_speed << "\n";
            log_timestamp_speed_ << t << "\n";
            log_timestamp_target_speed_ << t << "\n";

            log_gyro_ << pitch_rate_ << "\n";
            log_target_gyro_ << 0.0 << "\n";
            log_timestamp_gyro_ << t << "\n";
            log_timestamp_target_gyro_ << t << "\n";
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "[LQR] x=%.3f v=%.2f pitch=%.2f u=%.2f cmd=%.2f h=%.3f Kθ=%.1f",
                             x_, x_dot_, pitch_, u_pitch, cmd_x, current_height_, current_gain_.k_theta);
    }

    // ==================== 腿部高度 ====================
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

    void interpolate_lqr_gain()
    {
        double r = height_ratio();
        current_gain_.k_x = lerp(gain_low_.k_x, gain_high_.k_x, r);
        current_gain_.k_x_dot = lerp(gain_low_.k_x_dot, gain_high_.k_x_dot, r);
        current_gain_.k_theta = lerp(gain_low_.k_theta, gain_high_.k_theta, r);
        current_gain_.k_theta_dot = lerp(gain_low_.k_theta_dot, gain_high_.k_theta_dot, r);
    }

    // ==================== 腿部CAN发送 ====================
    void send_leg_can()
    {
        double hip_angle, knee_angle;

        if (fix_legs_)
        {
            // 锁腿模式：跳过 IK，直接使用固定角度
            hip_angle  = hip_angle_fixed_;
            knee_angle = knee_angle_fixed_;
        }
        else
        {
            // 正常模式：通过 IK 逆解计算关节角度
            double x_off = lerp(0.032, 0.015, height_ratio());
            bbot_real::IKSolution ik = kinematics_.inverse_kinematics(current_height_, 0.0, x_off);

            hip_angle = -ik.theta_hip;
            knee_angle = -ik.theta_knee;

            // 安全保护：防止逆解计算由于边界状态返回 NaN 导致电机报错失去力矩
            if (std::isnan(hip_angle) || std::isnan(knee_angle))
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                      "[IK] 逆解计算输出 NaN 越界！已过滤。h=%.3f, x_off=%.3f", current_height_, x_off);
                return;
            }
        }

        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {hip_angle, knee_angle, hip_angle, knee_angle};
        leg_pub_->publish(leg_cmd);

        if (leg_mode_ == "servo")
        {
            // 纯伺服位置控制（模式0x01）
            double hip_deg = hip_angle * 180.0 / M_PI;
            double knee_deg = knee_angle * 180.0 / M_PI;
            motor_left_hip_.set_servo_position(hip_deg);
            motor_left_knee_.set_servo_position(knee_deg);
            motor_right_hip_.set_servo_position(-hip_deg);
            motor_right_knee_.set_servo_position(-knee_deg);
        }
        else // hybrid
        {
            // 力位混合控制（模式0x00）：位置 + 重力补偿前馈
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

            motor_left_hip_.set_position(hip_angle, 0.0, hip_ff, leg_kp_, leg_kd_);
            motor_left_knee_.set_position(knee_angle, 0.0, knee_ff, leg_kp_, leg_kd_);

            motor_right_hip_.set_position(-hip_angle, 0.0, hip_ff, leg_kp_, leg_kd_);
            motor_right_knee_.set_position(-knee_angle, 0.0, knee_ff, leg_kp_, leg_kd_);
        }
    }

    void send_wheel_can(double cmd_x, double yaw_rate)
    {
        int16_t base_torque = static_cast<int16_t>(cmd_x * 1000.0);
        int16_t diff_torque = static_cast<int16_t>(yaw_rate * 500.0); // 500.0为差速转向增益

        // 差扭混合控制发送至双轴轮毂电机驱动器
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

    // ==================== 成员变量 ====================

    // CAN与电机
    std::shared_ptr<bbot_real::CanInterface> can_;
    bbot_real::JointMotorDriver motor_left_hip_, motor_left_knee_;
    bbot_real::JointMotorDriver motor_right_hip_, motor_right_knee_;
    bbot_real::WheelMotorDriver wheel_;
    int wheel_node_id_;
    bbot_real::Kinematics kinematics_;

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
    double pitch_rate_alpha_ = 0.10;

    // 轮部状态反馈
    double x_ = 0.0;     // 物理累计线位移
    double x_dot_ = 0.0; // 物理过滤线速度
    int speed_read_counter_ = 0;

    // LQR增益
    LQRGain gain_low_, gain_high_, current_gain_;

    // 平衡参数
    double balance_offset_, cmd_scale_, cmd_sign_, wheel_radius_, max_cmd_x_, max_safe_pitch_;

    // 遥控指令
    double target_speed_const_ = 0.0, target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_, turn_speed_, speed_ramp_time_;
    double target_x_ = 0.0;
    bool was_moving_ = false;

    // 腿部高度
    double L_MIN_, L_MAX_, current_height_, target_height_, leg_transition_speed_;

    // 速度积分
    double vel_integral_ = 0.0;

    // 腿部控制
    std::string leg_mode_;
    float leg_kp_, leg_kd_;
    bool joints_enabled_ = false; // 关节电机自适应状态标志位

    // 锁腿模式
    bool fix_legs_ = false;
    double hip_angle_fixed_ = 0.0;   // rad
    double knee_angle_fixed_ = 0.0;  // rad

    // 急停状态
    bool is_emergency_stopped_ = false;

    // 数据日志文件流
    std::ofstream log_angle_, log_target_angle_, log_timestamp_angle_, log_timestamp_target_angle_;
    std::ofstream log_speed_, log_target_speed_, log_timestamp_speed_, log_timestamp_target_speed_;
    std::ofstream log_gyro_, log_target_gyro_, log_timestamp_gyro_, log_timestamp_target_gyro_;
    bool logging_enabled_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRBalanceController>());
    rclcpp::shutdown();
    return 0;
}