#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>

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

using namespace std::chrono_literals;

// 常用数学与滤波工具函数
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

// LQR增益参数结构
struct LQRGain
{
    double k_x, k_x_dot, k_theta, k_theta_dot;
};

class LQRBalanceController : public rclcpp::Node
{
public:
    LQRBalanceController()
        : Node("lqr_balance_controller")
    {
        // LQR 增益配置（下蹲 / 站立）
        gain_low_ = {-6.1624, -46.8436, -197.6985, -46.8109};
        gain_high_ = {-6.3650, -48.5719, -227.1942, -57.6391};
        current_gain_ = gain_high_;

        // 平衡与重心参数
        // 平衡参数
        balance_offset_min_ = -3.5 * M_PI / 180.0; // 蹲伏时的平衡角
        balance_offset_max_ = -0.8 * M_PI / 180.0; // 站立时的平衡角

        balance_offset_ = 0.0; // 当前插值后的动态偏置
        balance_offset_auto_ = balance_offset_;
        ki_vel_trim_ = 0.0;
        cmd_scale_ = 1.0;
        cmd_sign_ = -1.0;
        max_cmd_x_ = 10.0;
        max_safe_pitch_ = 0.40;

        walk_speed_ = 0.3;
        turn_speed_ = 0.5;
        speed_ramp_time_ = 1.0;

        const auto &robot_params = kinematics_.params();

        L_MIN_ = robot_params.L_MIN;
        L_MAX_ = robot_params.L_MAX;

        target_height_ = L_MIN_;
        current_height_ = target_height_;
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0;

        // Roll 差动平衡补偿参数
        roll_kp_ = 0.35;
        roll_ki_ = 0.08;
        roll_kd_ = 0.015;

        // roll_kp_ = 0.0;
        // roll_ki_ = 0.0;
        // roll_kd_ = 0.0;

        roll_target_ = 0.5 * 3.14 / 180.0;
        roll_offset_ = 0.0 * 3.14 / 180.0;
        roll_sign_ = 1.0;
        max_delta_h_ = 0.04;

        RCLCPP_INFO(this->get_logger(), "Roll补偿配置: Kp=%.2f Ki=%.3f Kd=%.3f offset=%.3f sign=%.1f max_dh=%.3f",
                    roll_kp_, roll_ki_, roll_kd_, roll_offset_, roll_sign_, max_delta_h_);

        // 位置积分微调物理重心平衡角参数
        ki_pos_angle_ = 0.0;
        max_pos_trim_angle_ = 0.035;
        RCLCPP_INFO(this->get_logger(), "位置积分微调角度配置: Ki_pos_angle=%.4f max_trim=%.3f rad",
                    ki_pos_angle_, max_pos_trim_angle_);

        // 初始化上一帧有效关节角度（防止初始为0导致突变）
        bbot_real::IKSolution init_ik = kinematics_.inverse_kinematics(current_height_, 0.0, 0.0);
        last_valid_hip_l_ = -init_ik.theta_hip;
        last_valid_knee_l_ = -init_ik.theta_knee;
        last_valid_hip_r_ = -init_ik.theta_hip;
        last_valid_knee_r_ = -init_ik.theta_knee;

        // 腿部控制模式
        leg_mode_ = "servo";
        RCLCPP_INFO(this->get_logger(), "腿部控制模式: %s", leg_mode_.c_str());

        leg_kp_ = 30.0f;
        leg_kd_ = 1.0f;

        // CAN 总线初始化
        can_ = std::make_shared<bbot_real::CanInterface>();
        std::string can_if = "can0";
        try
        {
            can_->open(can_if);
            RCLCPP_INFO(this->get_logger(), "CAN接口 '%s' 打开成功", can_if.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "CAN打开失败 '%s': %s（以无CAN模式运行）", can_if.c_str(), e.what());
        }

        // 关节电机初始化
        motor_left_hip_.init(
            can_, 1, robot_params.hip_torque_max);

        motor_left_knee_.init(
            can_, 2, robot_params.knee_torque_max);

        motor_right_hip_.init(
            can_, 3, robot_params.hip_torque_max);

        motor_right_knee_.init(
            can_, 4, robot_params.knee_torque_max);

        motor_left_hip_.enable();
        motor_left_knee_.enable();
        motor_right_hip_.enable();
        motor_right_knee_.enable();
        joints_enabled_ = true;

        wheel_node_id_ = 5;
        wheel_.init(can_, wheel_node_id_);

        if (can_->is_open() && !wheel_.enable())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "轮毂电机使能失败，请检查驱动器");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "轮毂电机 node=%d",
            wheel_node_id_);

        wheel_radius_ = robot_params.wheel_radius;
        max_wheel_speed_ = 5.0;

        // 话题订阅与发布
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&LQRBalanceController::imu_callback, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input", 10,
            std::bind(&LQRBalanceController::joy_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        // 控制定时器 (200Hz)
        timer_ = this->create_wall_timer(5ms, std::bind(&LQRBalanceController::control_loop, this));
        last_time_ = this->now();

        // 数据日志初始化
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

            log_left_current_.open(log_dir + "left_current_data.txt");
            log_right_current_.open(log_dir + "right_current_data.txt");
            log_timestamp_left_current_.open(log_dir + "timestamp_left_current.txt");
            log_timestamp_right_current_.open(log_dir + "timestamp_right_current.txt");

            log_hip_left_torque_.open(log_dir + "hip_left_torque.txt");
            log_knee_left_torque_.open(log_dir + "knee_left_torque.txt");
            log_hip_right_torque_.open(log_dir + "hip_right_torque.txt");
            log_knee_right_torque_.open(log_dir + "knee_right_torque.txt");
            logging_enabled_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "数据日志文件打开失败: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "LQR平衡控制器启动完成");
    }

    ~LQRBalanceController()
    {
        wheel_.emergency_stop();
        motor_left_hip_.disable();
        motor_left_knee_.disable();
        motor_right_hip_.disable();
        motor_right_knee_.disable();
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
        if (log_left_current_.is_open())
            log_left_current_.close();
        if (log_right_current_.is_open())
            log_right_current_.close();
        if (log_timestamp_left_current_.is_open())
            log_timestamp_left_current_.close();
        if (log_timestamp_right_current_.is_open())
            log_timestamp_right_current_.close();
        if (log_hip_left_torque_.is_open())
            log_hip_left_torque_.close();
        if (log_knee_left_torque_.is_open())
            log_knee_left_torque_.close();
        if (log_hip_right_torque_.is_open())
            log_hip_right_torque_.close();
        if (log_knee_right_torque_.is_open())
            log_knee_right_torque_.close();
    }

private:
    void query_motor_torque()
    {
        uint8_t data[2] = {0xE0, 0x03};
        can_->send(1, data, 2);
        can_->send(2, data, 2);
        can_->send(3, data, 2);
        can_->send(4, data, 2);
    }

    void query_motor_torque_constant()
    {
        uint8_t data[2] = {0xE0, 0x16};
        can_->send(1, data, 2);
        can_->send(2, data, 2);
        can_->send(3, data, 2);
        can_->send(4, data, 2);
    }

    void read_motor_feedback()
    {
        uint32_t can_id;
        uint8_t data[8];
        while (true)
        {
            int len = can_->recv(can_id, data, 8);
            if (len <= 0)
                break;

            if (can_id == 1u)
                motor_left_hip_.parse_feedback(data, len);
            else if (can_id == 2u)
                motor_left_knee_.parse_feedback(data, len);
            else if (can_id == 3u)
                motor_right_hip_.parse_feedback(data, len);
            else if (can_id == 4u)
                motor_right_knee_.parse_feedback(data, len);
            else if (can_id == (0x180u + wheel_node_id_))
            {
                uint32_t val = data[0] |
                               (static_cast<uint32_t>(data[1]) << 8) |
                               (static_cast<uint32_t>(data[2]) << 16) |
                               (static_cast<uint32_t>(data[3]) << 24);
                int16_t left_raw = static_cast<int16_t>(val & 0xFFFF);
                int16_t right_raw = static_cast<int16_t>((val >> 16) & 0xFFFF);

                double left_rpm = left_raw * 0.1;
                double right_rpm = right_raw * 0.1;
                double left_mps = (left_rpm * 2.0 * M_PI / 60.0) * wheel_radius_;
                double right_mps = (right_rpm * 2.0 * M_PI / 60.0) * wheel_radius_;

                double max_abs_wheel_mps = std::max(std::abs(left_mps), std::abs(right_mps));
                if (max_abs_wheel_mps > max_wheel_speed_)
                {
                    wheel_over_speed_ = true;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "轮速超限 %.2f m/s，触发急停", max_abs_wheel_mps);
                }

                double current_x_dot = (left_mps - right_mps) / 2.0;
                x_dot_ = low_pass_filter(current_x_dot, x_dot_, 0.50);
            }
        }
    }

    // ==================== 遥控器回调 ====================
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
            target_x_ = x_;
            was_moving_ = false;
            vel_integral_ = 0.0;
            pos_integral_ = 0.0;
            pos_trim_angle_ = 0.0;
            return;
        }

        if (is_emergency_stopped_)
        {
            is_emergency_stopped_ = false;
            wheel_over_speed_ = false;
            RCLCPP_INFO(this->get_logger(), "急停按钮已释放，等待安全角度后重新使能电机");
        }

        // 死区处理 (死区阈值 0.05，死区外线性归一化平滑过渡)
        const double deadzone = 0.05;
        // 速度控制 (前进/后退)
        if (std::abs(lx_pitch) > deadzone)
        {
            double sign = (lx_pitch > 0.0) ? 1.0 : -1.0;
            double scaled = (std::abs(lx_pitch) - deadzone) / (1.0 - deadzone);
            target_speed_const_ = sign * scaled * walk_speed_;
        }
        else
        {
            target_speed_const_ = 0.0;
        }

        // 转向控制 (左转/右转)
        if (std::abs(ly_roll) > deadzone)
        {
            double sign = (ly_roll > 0.0) ? 1.0 : -1.0;
            double scaled = (std::abs(ly_roll) - deadzone) / (1.0 - deadzone);
            target_yaw_rate_ = -sign * scaled * turn_speed_;
        }
        else
        {
            target_yaw_rate_ = 0.0;
        }

        // 腿高度控制 (aux1: -1.0 ~ +1.0 线性映射到 L_MIN_ ~ L_MAX_)
        double height_cmd = L_MIN_ + ((aux1 + 1.0) / 2.0) * (L_MAX_ - L_MIN_);
        target_height_ = clamp_value(height_cmd, L_MIN_, L_MAX_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "已成功接收到第一帧 IMU 数据！");

        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        pitch_ = pitch;
        pitch_rate_raw_ = msg->angular_velocity.y;

        // Roll 状态提取与低通滤波
        roll_ = roll;
        roll_rate_raw_ = msg->angular_velocity.x;

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

        if (!roll_rate_filter_init_)
        {
            roll_rate_filt_ = roll_rate_raw_;
            roll_rate_filter_init_ = true;
        }
        else
        {
            roll_rate_filt_ = low_pass_filter(roll_rate_raw_, roll_rate_filt_, roll_rate_alpha_);
        }
        roll_rate_ = roll_rate_filt_;

        imu_received_ = true;
    }

    // 主控制循环
    void control_loop()
    {
        if (!imu_received_)
            return;

        torque_query_tick_++;
        if (torque_query_tick_ % 4 == 0)
            query_motor_torque();
        if (torque_query_tick_ % 100 == 0)
            query_motor_torque_constant();

        read_motor_feedback();

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0001 || dt > 0.05)
            dt = 0.005;

        loop_tick_++;

        startup_elapsed_ += dt;
        if (startup_elapsed_ > leg_startup_ramp_time_)
            startup_elapsed_ = leg_startup_ramp_time_;

        // 站立完全展开后锁定当前位置为自平衡目标原点
        if (!standup_done_)
        {
            target_x_ = x_; // 站稳前，目标位置紧跟当前位置，不产生预置位置误差
            pos_integral_ = 0.0;
            if (startup_elapsed_ >= leg_startup_ramp_time_)
            {
                standup_done_ = true;
                RCLCPP_INFO(this->get_logger(), "机器人站立就绪，锁定目标位置 target_x = %.3f", target_x_);
            }
        }

        // 安全停机与自动重使能
        if (std::abs(pitch_) > max_safe_pitch_ || is_emergency_stopped_ || wheel_over_speed_)
        {
            wheel_.emergency_stop();
            publish_cmd(0.0, 0.0);

            motor_left_hip_.disable();
            motor_left_knee_.disable();
            motor_right_hip_.disable();
            motor_right_knee_.disable();

            joints_enabled_ = false;
            vel_integral_ = 0.0;
            pos_integral_ = 0.0;
            roll_integral_ = 0.0;
            pos_trim_angle_ = 0.0;
            startup_elapsed_ = 0.0;
            standup_done_ = false;

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "急停触发...");
            return;
        }
        else if (!joints_enabled_)
        {
            RCLCPP_INFO(this->get_logger(), "角度恢复安全，重新使能全电机...");
            x_ = 0.0;
            target_x_ = 0.0;
            vel_integral_ = 0.0;
            pos_integral_ = 0.0;
            roll_integral_ = 0.0;
            pos_trim_angle_ = 0.0;
            wheel_over_speed_ = false; // 清除超速标志
            balance_offset_auto_ = balance_offset_;

            if (!wheel_.is_enabled() && !wheel_.enable())
                RCLCPP_WARN(this->get_logger(), "轮毂电机重新使能失败");
            motor_left_hip_.enable();
            motor_left_knee_.enable();
            motor_right_hip_.enable();
            motor_right_knee_.enable();
            joints_enabled_ = true;
        }

        // // ================== 重心零偏在线自适应 (适度增益) ==================
        // if (target_speed_const_ == 0.0 && standup_done_)
        // {
        //     // 适度步长在线寻优重心，杜绝长期漂移
        //     balance_offset_auto_ += x_dot_ * ki_vel_trim_ * dt;
        //     balance_offset_auto_ = clamp_value(balance_offset_auto_,
        //                                        balance_offset_ - 0.04,
        //                                        balance_offset_ + 0.04);
        // }
        // else
        // {
        //     balance_offset_auto_ = balance_offset_;
        // }

        balance_offset_auto_ = balance_offset_;

        double dynamic_target_pitch = balance_offset_auto_;

        // 里程计积分
        x_ += x_dot_ * dt;

        update_leg_height(dt);
        interpolate_lqr_gain();

        // 速度规划平滑过渡
        double ramp_step = dt / speed_ramp_time_;
        if (target_speed_smoothed_ < target_speed_const_)
            target_speed_smoothed_ = std::min(target_speed_smoothed_ + ramp_step, target_speed_const_);
        else if (target_speed_smoothed_ > target_speed_const_)
            target_speed_smoothed_ = std::max(target_speed_smoothed_ - ramp_step, target_speed_const_);
        double target_speed = target_speed_smoothed_;

        // 运动状态与目标点维护
        if (target_speed_const_ == 0.0 && std::abs(target_speed) < 0.005)
        {
            if (was_moving_)
            {
                target_x_ = x_;
                pos_integral_ = 0.0;
                was_moving_ = false;
            }
        }
        else
        {
            target_x_ += target_speed * dt;
            was_moving_ = true;
        }

        double pos_error = x_ - target_x_;
        double vel_error = -x_dot_ + 0.0;
        double gyro_val = pitch_rate_;
        double u_pitch = 0.0;

        double r = height_ratio();

        balance_offset_ = lerp(balance_offset_min_, balance_offset_max_, r);

        if (target_speed_const_ == 0.0 && std::abs(target_speed_smoothed_) < 0.005)
        {
            // ================== 标准全状态 LQR (静止自平衡) ==================
            if (standup_done_)
            {
                // 位置积分微调目标俯仰角（车往前偏 pos_error > 0 时向后微仰，车往后退 pos_error < 0 时向前微俯）
                pos_trim_angle_ -= ki_pos_angle_ * vel_error * dt;
                pos_trim_angle_ = clamp_value(pos_trim_angle_, -max_pos_trim_angle_, max_pos_trim_angle_);
            }
            else
            {
                pos_trim_angle_ = 0.0;
            }

            dynamic_target_pitch = balance_offset_ + pos_trim_angle_;
            double theta_error = pitch_ - dynamic_target_pitch;

            // 纯全状态 LQR 反馈控制（由目标角度积分彻底消除稳态位置误差）
            u_pitch = -(current_gain_.k_x * pos_error +
                        current_gain_.k_x_dot * vel_error +
                        current_gain_.k_theta * theta_error +
                        current_gain_.k_theta_dot * gyro_val);
            vel_integral_ = 0.0;
            pos_integral_ = 0.0;
        }
        else
        {
            // ================== 运动模式：速度外环 + 姿态 LQR ==================
            pos_trim_angle_ = 0.0; // 运动时清零微调量

            double vel_error_v = target_speed_smoothed_ - x_dot_;
            vel_integral_ += vel_error_v * dt;
            vel_integral_ = clamp_value(vel_integral_, -0.5, 0.5);

            dynamic_target_pitch = balance_offset_ + (0.15 * vel_error_v + 0.03 * vel_integral_);
            dynamic_target_pitch = clamp_value(dynamic_target_pitch, -0.2, 0.2);

            double theta_error = pitch_ - dynamic_target_pitch;
            u_pitch = -(current_gain_.k_theta * theta_error +
                        current_gain_.k_theta_dot * gyro_val);
            pos_integral_ = 0.0;
        }

        // 最终力矩输出与平滑下发
        double total_torque = clamp_value(u_pitch * cmd_scale_ * cmd_sign_, -max_cmd_x_, max_cmd_x_);

        publish_cmd(total_torque, target_yaw_rate_);
        send_wheel_can(total_torque, target_yaw_rate_);
        send_leg_can(dt);

        // 运行数据记录
        if (logging_enabled_)
        {
            double t = now.nanoseconds() * 1e-9;
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

            log_left_current_ << left_cmd_ma_ << "\n";
            log_right_current_ << right_cmd_ma_ << "\n";
            log_timestamp_left_current_ << t << "\n";
            log_timestamp_right_current_ << t << "\n";

            log_hip_left_torque_ << motor_left_hip_.torque_feedback() << "\n";
            log_knee_left_torque_ << motor_left_knee_.torque_feedback() << "\n";
            log_hip_right_torque_ << motor_right_hip_.torque_feedback() << "\n";
            log_knee_right_torque_ << motor_right_knee_.torque_feedback() << "\n";
        }

        double roll_err_deg = (roll_target_ + roll_offset_ - roll_) * 180.0 / M_PI;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 150,
                             "[LQR] pit=%.2f°(targ=%.2f° trim=%.2f°) torq=%.2f | roll=%.2f° dh=%.1fmm h=%.3f",
                             pitch_ * 180.0 / M_PI, dynamic_target_pitch * 180.0 / M_PI, pos_trim_angle_ * 180.0 / M_PI,
                             total_torque, roll_ * 180.0 / M_PI, last_delta_h_ * 1000.0, current_height_);
    }

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

    // 腿部关节驱动与 Roll 姿态差动高度补偿
    void send_leg_can(double dt)
    {
        // 目标 Roll 叠加机械零偏补偿 roll_offset_
        double effective_target_roll = roll_target_ + roll_offset_;
        double roll_error = effective_target_roll - roll_;

        // 仅在站立就绪且正常使能时累积积分，防止开机/摔倒积分饱和
        if (standup_done_ && joints_enabled_)
        {
            roll_integral_ += roll_error * dt;
            roll_integral_ = clamp_value(roll_integral_, -roll_integral_limit_, roll_integral_limit_);
        }
        else
        {
            roll_integral_ = 0.0;
        }

        // 计算差动高度补偿量（PI+D，带方向符号系数 roll_sign_）
        double raw_delta_h = (roll_kp_ * roll_error + roll_ki_ * roll_integral_ - roll_kd_ * roll_rate_) * roll_sign_;
        double delta_h = clamp_value(raw_delta_h, -max_delta_h_, max_delta_h_);
        last_delta_h_ = delta_h;

        // 左右腿高度分配（保留 0.005m 的安全几何裕量，防止 acos(>1) 奇异）
        double max_safe_h = L_MAX_ - 0.005;
        double h_left = clamp_value(current_height_ + delta_h, L_MIN_, max_safe_h);
        double h_right = clamp_value(current_height_ - delta_h, L_MIN_, max_safe_h);
        last_h_left_ = h_left;
        last_h_right_ = h_right;

        double ratio_l = clamp_value((h_left - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        double ratio_r = clamp_value((h_right - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        double x_off_l = lerp(0.078, 0.075, ratio_l);
        double x_off_r = lerp(0.068, 0.065, ratio_r);

        // 逆运动学求解
        bbot_real::IKSolution ik_l = kinematics_.inverse_kinematics(h_left, 0.0, x_off_l);
        bbot_real::IKSolution ik_r = kinematics_.inverse_kinematics(h_right, 0.0, x_off_r);

        // ================= 安全检查：若正常则更新，若出现 NaN 则保持上一帧有效值 =================
        if (!std::isnan(ik_l.theta_hip) && !std::isnan(ik_l.theta_knee))
        {
            last_valid_hip_l_ = -ik_l.theta_hip;
            last_valid_knee_l_ = -ik_l.theta_knee;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "左腿逆解越界 NaN (hl=%.3f, x_off=%.3f)，保持上一帧姿态！", h_left, x_off_l);
        }

        if (!std::isnan(ik_r.theta_hip) && !std::isnan(ik_r.theta_knee))
        {
            last_valid_hip_r_ = -ik_r.theta_hip;
            last_valid_knee_r_ = -ik_r.theta_knee;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "右腿逆解越界 NaN (hr=%.3f, x_off=%.3f)，保持上一帧姿态！", h_right, x_off_r);
        }

        // 使用经过安全过滤的有效角度
        double hip_l = last_valid_hip_l_;
        double knee_l = last_valid_knee_l_;
        double hip_r = last_valid_hip_r_;
        double knee_r = last_valid_knee_r_;

        // ROS 话题发布
        std_msgs::msg::Float64MultiArray leg_cmd;
        leg_cmd.data = {hip_l, knee_l, hip_r, knee_r};
        leg_pub_->publish(leg_cmd);

        double startup_ramp = clamp_value(startup_elapsed_ / leg_startup_ramp_time_, 0.0, 1.0);
        uint16_t servo_speed = static_cast<uint16_t>(lerp(100.0, 500.0, startup_ramp));
        uint16_t servo_current = static_cast<uint16_t>(lerp(200.0, 1000.0, startup_ramp));
        float startup_kp = static_cast<float>(lerp(5.0, leg_kp_, startup_ramp));
        float startup_kd = static_cast<float>(lerp(0.2, leg_kd_, startup_ramp));

        // ================= 周期性下发 CAN，永远不提前 return =================
        if (leg_mode_ == "servo")
        {
            double hip_deg_l = hip_l * 180.0 / M_PI;
            double knee_deg_l = knee_l * 180.0 / M_PI;
            double hip_deg_r = hip_r * 180.0 / M_PI;
            double knee_deg_r = knee_r * 180.0 / M_PI;

            motor_left_hip_.set_servo_position(hip_deg_l, servo_speed, servo_current);
            motor_left_knee_.set_servo_position(knee_deg_l, servo_speed, servo_current);
            motor_right_hip_.set_servo_position(-hip_deg_r, servo_speed, servo_current);
            motor_right_knee_.set_servo_position(-knee_deg_r, servo_speed, servo_current);
        }
        else
        {
            // Hybrid 力位混合模式：使用安全记忆角度计算重力补偿
            bbot_real::JointTorques torques_l = kinematics_.compute_gravity_torques(
                0.0, -hip_l, -knee_l);
            bbot_real::JointTorques torques_r = kinematics_.compute_gravity_torques(
                0.0, -hip_r, -knee_r);

            motor_left_hip_.set_position(hip_l, 0.0, torques_l.hip_torque / 2.0, startup_kp, startup_kd);
            motor_left_knee_.set_position(knee_l, 0.0, torques_l.knee_torque / 2.0, startup_kp, startup_kd);
            motor_right_hip_.set_position(-hip_r, 0.0, -torques_r.hip_torque / 2.0, startup_kp, startup_kd);
            motor_right_knee_.set_position(-knee_r, 0.0, -torques_r.knee_torque / 2.0, startup_kp, startup_kd);
        }
    }

    // 轮毂力矩下发（平滑连续控制）
    void send_wheel_can(double total_torque, double yaw_rate)
    {
        double single_wheel_torque_nm = total_torque / 2.0;
        int16_t base_torque = static_cast<int16_t>(single_wheel_torque_nm * 1000.0);
        int16_t diff_torque = static_cast<int16_t>(yaw_rate * 500.0);

        int16_t left_cmd = clamp_value(base_torque + diff_torque, -7000, 7000);
        int16_t right_cmd = clamp_value(-base_torque + diff_torque, -7000, 7000);

        left_cmd_ma_ = left_cmd;
        right_cmd_ma_ = right_cmd;

        wheel_.set_torque(left_cmd, right_cmd);
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

    // CAN 与执行器
    std::shared_ptr<bbot_real::CanInterface> can_;
    bbot_real::JointMotorDriver motor_left_hip_, motor_left_knee_;
    bbot_real::JointMotorDriver motor_right_hip_, motor_right_knee_;
    bbot_real::WheelMotorDriver wheel_;
    int wheel_node_id_;
    bbot_real::Kinematics kinematics_;

    // ROS 话题通信
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // IMU 传感器状态
    bool imu_received_ = false;
    double pitch_ = 0.0, pitch_rate_ = 0.0;
    double pitch_rate_raw_ = 0.0, pitch_rate_filt_ = 0.0;
    bool pitch_rate_filter_init_ = false;
    double pitch_rate_alpha_ = 0.80;

    // Roll 状态与补偿参数
    double roll_ = 0.0, roll_rate_ = 0.0;
    double roll_rate_raw_ = 0.0, roll_rate_filt_ = 0.0;
    bool roll_rate_filter_init_ = false;
    double roll_rate_alpha_ = 0.80;

    double roll_target_ = 0.0;
    double roll_offset_ = 0.0 * 3.14 / 180.0; // 机械零偏手动/参数补偿 (rad)
    double roll_sign_ = -1.0;                 // 补偿方向极性切换系数 (+1.0 或 -1.0)
    double roll_kp_ = 0.35;                   // 比例增益 (提升至 0.35)
    double roll_ki_ = 0.08;                   // 积分增益 (消除稳态静差)
    double roll_kd_ = 0.015;                  // 微分阻尼增益
    double roll_integral_ = 0.0;
    double roll_integral_limit_ = 0.20; // 积分饱和限幅 (rad*s)
    double max_delta_h_ = 0.04;         // 最大高度差 (m)
    double last_delta_h_ = 0.0;
    double last_h_left_ = 0.36;
    double last_h_right_ = 0.36;

    // 记忆上一次有效关节角度（防止 NaN 导致看门狗超时）
    double last_valid_hip_l_ = 0.0;
    double last_valid_knee_l_ = 0.0;
    double last_valid_hip_r_ = 0.0;
    double last_valid_knee_r_ = 0.0;

    // 里程计状态
    double x_ = 0.0;
    double x_dot_ = 0.0;
    double wheel_radius_;
    double max_wheel_speed_;

    // LQR 增益
    LQRGain gain_low_, gain_high_, current_gain_;

    // 平衡与定点积分参数
    // 平衡参数
    double balance_offset_min_ = -3.0 * M_PI / 180.0; // 蹲伏时的平衡角
    double balance_offset_max_ = -2.0 * M_PI / 180.0; // 站立时的平衡角
    double balance_offset_ = 0.0;                     // 当前插值后的动态偏置

    double balance_offset_auto_;
    double k_i_pos_ = -0.8;
    double pos_integral_ = 0.0;
    bool standup_done_ = false;

    // 位置积分微调物理重心平衡角
    double ki_pos_angle_ = 0.0;         // 积分增益 (rad/(m*s))
    double max_pos_trim_angle_ = 0.035; // 最大微调幅度 (约 ±2.0 度)
    double pos_trim_angle_ = 0.0;       // 实时角度微调量 (rad)

    double ki_vel_trim_ = 0.02;

    double cmd_scale_, cmd_sign_ = 1.0, max_cmd_x_, max_safe_pitch_;
    unsigned int loop_tick_ = 0;

    // 运动指令
    double target_speed_const_ = 0.0, target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_, turn_speed_, speed_ramp_time_;
    double target_x_ = 0.0;
    bool was_moving_ = false;

    // 腿部高度与启动斜坡
    double L_MIN_, L_MAX_, current_height_, target_height_, leg_transition_speed_;
    double leg_startup_ramp_time_ = 5.0;
    double startup_elapsed_ = 0.0;

    double vel_integral_ = 0.0;

    // 腿部控制模式
    std::string leg_mode_;
    float leg_kp_, leg_kd_;

    bool is_emergency_stopped_ = false;
    bool wheel_over_speed_ = false;

    // 日志文件流
    std::ofstream log_angle_, log_target_angle_, log_timestamp_angle_, log_timestamp_target_angle_;
    std::ofstream log_speed_, log_target_speed_, log_timestamp_speed_, log_timestamp_target_speed_;
    std::ofstream log_gyro_, log_target_gyro_, log_timestamp_gyro_, log_timestamp_target_gyro_;
    std::ofstream log_left_current_, log_right_current_, log_timestamp_left_current_, log_timestamp_right_current_;
    std::ofstream log_hip_left_torque_, log_knee_left_torque_, log_hip_right_torque_, log_knee_right_torque_;
    bool logging_enabled_ = false;

    double left_cmd_ma_ = 0.0;
    double right_cmd_ma_ = 0.0;

    bool joints_enabled_ = false;
    uint32_t torque_query_tick_ = 0;
}; // 类结束

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQRBalanceController>());
    rclcpp::shutdown();
    return 0;
}