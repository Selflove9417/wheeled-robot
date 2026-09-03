#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

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

using namespace std::chrono_literals;

// 工具函数
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

// PID参数结构
struct PIDParam
{
    float p, i, d, ramp, limit;
};

// 主控制器
class PIDBalanceController : public rclcpp::Node
{
public:
    PIDBalanceController()
        : Node("pid_balance_controller")
    {
        PIDParam speed_stand = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        PIDParam speed_squat = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        PIDParam angle_stand = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        PIDParam angle_squat = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        PIDParam gyro_stand = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        PIDParam gyro_squat = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        pid_speed_ = bbot_real::PIDController(speed_stand.p, speed_stand.i, speed_stand.d,
                                              speed_stand.ramp, speed_stand.limit);
        pid_angle_ = bbot_real::PIDController(angle_stand.p, angle_stand.i, angle_stand.d,
                                              angle_stand.ramp, angle_stand.limit);
        pid_gyro_ = bbot_real::PIDController(gyro_stand.p, gyro_stand.i, gyro_stand.d,
                                             gyro_stand.ramp, gyro_stand.limit);

        // 平衡参数
        balance_offset_min_ = -3.9 * M_PI / 180.0; // 蹲伏时的平衡角
        balance_offset_max_ = -2.0 * M_PI / 180.0; // 站立时的平衡角

        cmd_sign_ = 1.0;
        max_cmd_x_ = 10.0;
        max_safe_pitch_ = 0.40; // 22.9°

        walk_speed_ = 0.3;
        turn_speed_ = 0.5;
        speed_ramp_time_ = 1.0;

        const auto &robot_params = kinematics_.params();

        L_MIN_ = robot_params.L_MIN;
        L_MAX_ = robot_params.L_MAX;

        target_height_ = L_MIN_;
        current_height_ = target_height_;
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0;

        // Roll 差动找平控制：低带宽 PD + 死区 + 输出限速
        // 目标不是快速姿态控制，而是缓慢消除机身左右倾斜，避免升降时左右腿高频抖动。
        roll_kp_ = 0.08;
        roll_ki_ = 0.0;
        roll_kd_ = 0.003;
        roll_offset_ = -1.5 * M_PI / 180.0;

        // IMU Roll=0 作为水平目标；若 IMU 安装有固定零偏，只调 roll_offset_。
        roll_target_ = 0.0 * M_PI / 180.0;

        roll_sign_ = 1.0;
        max_delta_h_ = 0.015;       // 单侧最大差动腿长 15 mm
        roll_delta_h_rate_ = 0.015; // 差动腿长最大变化速度 15 mm/s
        roll_deadband_ = 0.30 * M_PI / 180.0;
        roll_move_gain_scale_ = 0.25;

        RCLCPP_INFO(
            this->get_logger(),
            "Roll找平配置: Kp=%.3f Kd=%.4f deadband=%.2fdeg rate=%.1fmm/s max_dh=%.1fmm",
            roll_kp_, roll_kd_, roll_deadband_ * 180.0 / M_PI,
            roll_delta_h_rate_ * 1000.0, max_delta_h_ * 1000.0);

        // 初始化上一帧有效关节角度（防止初始为0导致突变）
        bbot_real::IKSolution init_ik = kinematics_.inverse_kinematics(current_height_, 0.0, 0.0);
        last_valid_hip_l_ = -init_ik.theta_hip;
        last_valid_knee_l_ = -init_ik.theta_knee;
        last_valid_hip_r_ = -init_ik.theta_hip;
        last_valid_knee_r_ = -init_ik.theta_knee;

        // 腿部控制模式
        leg_mode_ = "servo";
        RCLCPP_INFO(this->get_logger(), "腿部控制模式: %s", leg_mode_.c_str());

        // 力位混合控制参数（仅 hybrid 模式使用）
        leg_kp_ = 30.0f;
        leg_kd_ = 1.0f;

        // CAN总线
        can_ = std::make_shared<bbot_real::CanInterface>();
        std::string can_if = "can0";
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

        // 关节电机初始化
        motor_left_hip_.init(can_, 1, robot_params.hip_torque_max);
        motor_left_knee_.init(can_, 2, robot_params.knee_torque_max);
        motor_right_hip_.init(can_, 3, robot_params.hip_torque_max);
        motor_right_knee_.init(can_, 4, robot_params.knee_torque_max);

        // 使能所有关节电机
        motor_left_hip_.enable();
        motor_left_knee_.enable();
        motor_right_hip_.enable();
        motor_right_knee_.enable();

        // 轮毂电机（ZLAC CANopen双轴驱动器）
        wheel_node_id_ = 5;
        wheel_.init(can_, wheel_node_id_);
        if (can_->is_open() && !wheel_.enable())
            RCLCPP_WARN(this->get_logger(), "轮毂电机使能失败，请检查ZLAC驱动器");
        RCLCPP_INFO(this->get_logger(), "轮毂电机 ZLAC node=%d", wheel_node_id_);

        // 轮毂物理参数
        wheel_radius_ = robot_params.wheel_radius;
        RCLCPP_INFO(this->get_logger(), "轮毂半径: %.3f m", wheel_radius_);

        // 轮速超限保护阈值
        max_wheel_speed_ = 10.0;
        RCLCPP_INFO(this->get_logger(), "轮速超限保护阈值: %.3f m/s", max_wheel_speed_);

        // 订阅与发布
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&PIDBalanceController::imu_callback, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/rc_input", 10,
            std::bind(&PIDBalanceController::joy_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

        telemetry_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/bbot/telemetry", 10);

        // 控制定时器 200Hz
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

            log_left_current_.open(log_dir + "left_current_data.txt");
            log_right_current_.open(log_dir + "right_current_data.txt");
            log_hip_left_torque_.open(log_dir + "hip_left_torque.txt");
            log_knee_left_torque_.open(log_dir + "knee_left_torque.txt");
            log_hip_right_torque_.open(log_dir + "hip_right_torque.txt");
            log_knee_right_torque_.open(log_dir + "knee_right_torque.txt");

            // ==================== 关节温度日志 ====================

            log_hip_left_motor_temp_.open(
                log_dir + "hip_left_motor_temp.txt");
            log_hip_left_mos_temp_.open(
                log_dir + "hip_left_mos_temp.txt");

            log_knee_left_motor_temp_.open(
                log_dir + "knee_left_motor_temp.txt");
            log_knee_left_mos_temp_.open(
                log_dir + "knee_left_mos_temp.txt");

            log_hip_right_motor_temp_.open(
                log_dir + "hip_right_motor_temp.txt");
            log_hip_right_mos_temp_.open(
                log_dir + "hip_right_mos_temp.txt");

            log_knee_right_motor_temp_.open(
                log_dir + "knee_right_motor_temp.txt");
            log_knee_right_mos_temp_.open(
                log_dir + "knee_right_mos_temp.txt");

            log_timestamp_joint_temp_.open(
                log_dir + "timestamp_joint_temp.txt");

            log_timestamp_left_current_.open(log_dir + "timestamp_left_current.txt");
            log_timestamp_right_current_.open(log_dir + "timestamp_right_current.txt");
            logging_enabled_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "数据日志文件打开失败: %s", e.what());
        }

        // ==================== 键盘控制初始化 ====================
        setup_keyboard();

        RCLCPP_INFO(
            this->get_logger(),
            "键盘高度控制已启用：↑ 升高，↓ 降低");

        RCLCPP_INFO(this->get_logger(), "PID平衡控制器启动完成（遥控+位置控制模式）");
    }

    ~PIDBalanceController()
    {
        restore_keyboard();

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

        if (log_hip_left_motor_temp_.is_open())
            log_hip_left_motor_temp_.close();

        if (log_hip_left_mos_temp_.is_open())
            log_hip_left_mos_temp_.close();

        if (log_knee_left_motor_temp_.is_open())
            log_knee_left_motor_temp_.close();

        if (log_knee_left_mos_temp_.is_open())
            log_knee_left_mos_temp_.close();

        if (log_hip_right_motor_temp_.is_open())
            log_hip_right_motor_temp_.close();

        if (log_hip_right_mos_temp_.is_open())
            log_hip_right_mos_temp_.close();

        if (log_knee_right_motor_temp_.is_open())
            log_knee_right_motor_temp_.close();

        if (log_knee_right_mos_temp_.is_open())
            log_knee_right_mos_temp_.close();

        if (log_timestamp_joint_temp_.is_open())
            log_timestamp_joint_temp_.close();
    }

private:
    // ============================================================
    // ENCOS Servo位置模式：发送带 ACK 的位置控制指令
    //
    // 与 JointMotorDriver::set_servo_position() 的编码完全一致，
    // 唯一区别是允许设置最低 2 bit 的 ack。
    //
    // ack:
    //   0 -> 不返回
    //   1 -> 返回报文类型 1
    //   2 -> 返回报文类型 2
    //   3 -> 返回报文类型 3
    // ============================================================
    bool send_servo_position_with_ack(
        uint16_t motor_id,
        double pos_deg,
        uint16_t spd,
        uint16_t cur,
        uint8_t ack)
    {
        if (!can_ || !can_->is_open())
            return false;

        ack &= 0x03;

        union
        {
            float f;
            uint8_t b[4];
        } conv;

        conv.f = static_cast<float>(pos_deg);

        uint8_t data[8];

        data[0] = 0x20 | (conv.b[3] >> 3);
        data[1] = (conv.b[3] << 5) | (conv.b[2] >> 3);
        data[2] = (conv.b[2] << 5) | (conv.b[1] >> 3);
        data[3] = (conv.b[1] << 5) | (conv.b[0] >> 3);
        data[4] = (conv.b[0] << 5) | (spd >> 10);
        data[5] = (spd & 0x3FC) >> 2;
        data[6] = ((spd & 0x03) << 6) | (cur >> 6);

        // 原来的 JointMotorDriver 这里最低两位固定为 0。
        // 现在最低两位用于 ACK。
        data[7] =
            static_cast<uint8_t>(
                ((cur & 0x3F) << 2) |
                (ack & 0x03));

        return can_->send(motor_id, data, 8);
    }

    // ============================================================
    // 解析 ENCOS 问答模式返回报文类型 1
    //
    // Byte0[7:5] = frame type
    // Byte0[4:0] = error code
    // Byte6       = motor temperature * 2 + 50
    // Byte7       = MOS temperature   * 2 + 50
    // ============================================================
    void parse_joint_temperature_feedback(
        uint32_t can_id,
        const uint8_t *data,
        int len)
    {
        if (len < 8)
            return;

        const uint8_t frame_type =
            static_cast<uint8_t>((data[0] >> 5) & 0x07);

        if (frame_type != 1)
            return;

        const uint8_t error_code =
            static_cast<uint8_t>(data[0] & 0x1F);

        const double motor_temp =
            (static_cast<int>(data[6]) - 50) / 2.0;

        const double mos_temp =
            (static_cast<int>(data[7]) - 50) / 2.0;

        if (can_id == 1u)
        {
            temp_left_hip_motor_ = motor_temp;
            temp_left_hip_mos_ = mos_temp;
            error_left_hip_ = error_code;
        }
        else if (can_id == 2u)
        {
            temp_left_knee_motor_ = motor_temp;
            temp_left_knee_mos_ = mos_temp;
            error_left_knee_ = error_code;
        }
        else if (can_id == 3u)
        {
            temp_right_hip_motor_ = motor_temp;
            temp_right_hip_mos_ = mos_temp;
            error_right_hip_ = error_code;
        }
        else if (can_id == 4u)
        {
            temp_right_knee_motor_ = motor_temp;
            temp_right_knee_mos_ = mos_temp;
            error_right_knee_ = error_code;
        }

        if (error_code != 0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "关节电机 ID=%u 报错: error_code=%u, Motor=%.1f°C MOS=%.1f°C",
                static_cast<unsigned int>(can_id),
                static_cast<unsigned int>(error_code),
                motor_temp,
                mos_temp);
        }
    }
    void query_motor_torque()
    {
        uint8_t data[2];
        data[0] = 0xE0;
        data[1] = 0x03;
        can_->send(1, data, 2);
        can_->send(2, data, 2);
        can_->send(3, data, 2);
        can_->send(4, data, 2);
    }

    void query_motor_torque_constant()
    {
        uint8_t data[2];
        data[0] = 0xE0;
        data[1] = 0x16;
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
            {
                parse_joint_temperature_feedback(can_id, data, len);
                motor_left_hip_.parse_feedback(data, len);
            }
            else if (can_id == 2u)
            {
                parse_joint_temperature_feedback(can_id, data, len);
                motor_left_knee_.parse_feedback(data, len);
            }
            else if (can_id == 3u)
            {
                parse_joint_temperature_feedback(can_id, data, len);
                motor_right_hip_.parse_feedback(data, len);
            }
            else if (can_id == 4u)
            {
                parse_joint_temperature_feedback(can_id, data, len);
                motor_right_knee_.parse_feedback(data, len);
            }
            else if (can_id == (0x180u + wheel_node_id_))
            {
                uint32_t val =
                    data[0] |
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

            pid_speed_.reset();
            pid_angle_.reset();
            pid_gyro_.reset();
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

        // 对高度指令进行低通滤波，减少遥控器抖动
        const double height_alpha = 0.15; // 滤波系数，越小越平滑但响应越慢
        target_height_ = low_pass_filter(height_cmd, target_height_, height_alpha);
        target_height_ = clamp_value(target_height_, L_MIN_, L_MAX_);
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

        // Roll 角低通滤波：避免姿态估计的小噪声直接变成左右腿差动高度
        if (!roll_filter_init_)
        {
            roll_filt_ = roll;
            roll_filter_init_ = true;
        }
        else
        {
            roll_filt_ = low_pass_filter(roll, roll_filt_, roll_alpha_);
        }
        roll_ = roll_filt_;

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

    // ============================================================
    // 键盘输入初始化
    // ============================================================
    void setup_keyboard()
    {
        if (!isatty(STDIN_FILENO))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "stdin 不是终端，键盘高度控制不可用");
            keyboard_enabled_ = false;
            return;
        }

        if (tcgetattr(STDIN_FILENO, &original_terminal_settings_) != 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "读取终端设置失败，键盘高度控制不可用");
            keyboard_enabled_ = false;
            return;
        }

        struct termios new_settings = original_terminal_settings_;

        // 关闭规范模式和回显
        new_settings.c_lflag &= ~(ICANON | ECHO);

        // read() 立即返回
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_settings) != 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "设置终端模式失败，键盘高度控制不可用");
            keyboard_enabled_ = false;
            return;
        }

        original_stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);

        if (original_stdin_flags_ < 0)
        {
            tcsetattr(
                STDIN_FILENO,
                TCSANOW,
                &original_terminal_settings_);

            RCLCPP_WARN(
                this->get_logger(),
                "读取 stdin flags 失败，键盘高度控制不可用");

            keyboard_enabled_ = false;
            return;
        }

        if (fcntl(
                STDIN_FILENO,
                F_SETFL,
                original_stdin_flags_ | O_NONBLOCK) != 0)
        {
            tcsetattr(
                STDIN_FILENO,
                TCSANOW,
                &original_terminal_settings_);

            RCLCPP_WARN(
                this->get_logger(),
                "设置 stdin 非阻塞模式失败，键盘高度控制不可用");

            keyboard_enabled_ = false;
            return;
        }

        keyboard_enabled_ = true;
    }

    // ============================================================
    // 恢复键盘终端设置
    // ============================================================
    void restore_keyboard()
    {
        if (!keyboard_enabled_)
            return;

        tcsetattr(
            STDIN_FILENO,
            TCSANOW,
            &original_terminal_settings_);

        if (original_stdin_flags_ >= 0)
        {
            fcntl(
                STDIN_FILENO,
                F_SETFL,
                original_stdin_flags_);
        }

        keyboard_enabled_ = false;
    }

    // ============================================================
    // 键盘控制
    //
    // ↑ : 增加腿长 / 机身升高
    // ↓ : 减小腿长 / 机身降低
    // ============================================================
    void process_keyboard_input()
    {
        if (!keyboard_enabled_)
            return;

        char buffer[32];

        ssize_t n = read(
            STDIN_FILENO,
            buffer,
            sizeof(buffer));

        if (n <= 0)
            return;

        for (ssize_t i = 0; i < n; ++i)
        {
            // 检测方向键转义序列：
            //
            // ↑ : ESC [ A
            // ↓ : ESC [ B
            //
            if (buffer[i] == '\x1B')
            {
                if (i + 2 >= n)
                    continue;

                if (buffer[i + 1] != '[')
                    continue;

                // ==================== ↑ 上方向键 ====================
                if (buffer[i + 2] == 'A')
                {
                    target_height_ += keyboard_height_step_;

                    target_height_ = clamp_value(
                        target_height_,
                        L_MIN_,
                        L_MAX_);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "↑ 机身升高: target_height=%.4f m",
                        target_height_);
                }

                // ==================== ↓ 下方向键 ====================
                else if (buffer[i + 2] == 'B')
                {
                    target_height_ -= keyboard_height_step_;

                    target_height_ = clamp_value(
                        target_height_,
                        L_MIN_,
                        L_MAX_);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "↓ 机身降低: target_height=%.4f m",
                        target_height_);
                }

                i += 2;
            }
        }
    }
    void control_loop()
    {
        // 键盘输入独立于 IMU 处理
        process_keyboard_input();

        if (!imu_received_)
            return;

        // 每20ms查询一次关节电流，每500ms刷新一次扭矩系数
        torque_query_tick_++;
        if (torque_query_tick_ % 4 == 0)
            query_motor_torque();
        if (torque_query_tick_ % 100 == 0)
            query_motor_torque_constant();

        // 读取并分拣关节反馈 + 轮速TPDO
        read_motor_feedback();

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0001)
            dt = 0.005;
        if (dt > 0.05)
            dt = 0.005;

        loop_tick_++;
        const double dt_gyro = dt;
        double dt_angle = 0.0;
        double dt_speed = 0.0;
        if (loop_tick_ % 2 == 0)
            dt_angle = dt * 2.0;
        if (loop_tick_ % 4 == 0)
            dt_speed = dt * 4.0;

        startup_elapsed_ += dt;
        if (startup_elapsed_ > leg_startup_ramp_time_)
            startup_elapsed_ = leg_startup_ramp_time_;

        if (!standup_done_)
        {
            if (startup_elapsed_ >= leg_startup_ramp_time_)
            {
                standup_done_ = true;
                RCLCPP_INFO(this->get_logger(), "机器人站立就绪 (PID Controller)");
            }
        }

        // 安全停机
        if (std::abs(pitch_) > max_safe_pitch_ || is_emergency_stopped_ || wheel_over_speed_)
        {
            wheel_.emergency_stop();
            publish_cmd(0.0, 0.0);

            motor_left_hip_.disable();
            motor_left_knee_.disable();
            motor_right_hip_.disable();
            motor_right_knee_.disable();

            joints_enabled_ = false;

            pid_speed_.reset();
            pid_angle_.reset();
            pid_gyro_.reset();

            roll_integral_ = 0.0;
            roll_delta_h_ = 0.0;
            startup_elapsed_ = 0.0;
            standup_done_ = false;

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "急停...");
            return;
        }
        else
        {
            if (!joints_enabled_)
            {
                RCLCPP_INFO(this->get_logger(), "进入安全角度，正在重新使能轮毂与全关节电机...");
                if (!wheel_.is_enabled() && !wheel_.enable())
                    RCLCPP_WARN(this->get_logger(), "轮毂电机重新使能失败");
                motor_left_hip_.enable();
                motor_left_knee_.enable();
                motor_right_hip_.enable();
                motor_right_knee_.enable();
                joints_enabled_ = true;
                roll_integral_ = 0.0;
                roll_delta_h_ = 0.0;
                wheel_over_speed_ = false;
            }
        }

        update_leg_height(dt);
        interpolate_pid_gains();

        // 三级PID级联控制
        // if (dt_speed > 0.0)
        {
            double dt_speed = dt; // 取消 50Hz 分频，消除外环延迟

            double ramp_step = dt_speed / speed_ramp_time_;
            if (target_speed_smoothed_ < target_speed_const_)
                target_speed_smoothed_ = std::min(target_speed_smoothed_ + ramp_step, target_speed_const_);
            else if (target_speed_smoothed_ > target_speed_const_)
                target_speed_smoothed_ = std::max(target_speed_smoothed_ - ramp_step, target_speed_const_);

            double vel_error = -target_speed_smoothed_ + x_dot_;
            double target_pitch = pid_speed_(vel_error, dt_speed);
            target_pitch = clamp_value(target_pitch, -0.25, 0.25);

            double raw_target = target_pitch + balance_offset_;
            target_pitch_filtered_ = low_pass_filter(raw_target, target_pitch_filtered_, target_pitch_alpha_);
        }

        // if (dt_angle > 0.0)
        {
            double dt_angle = dt;
            double angle_error = target_pitch_filtered_ - pitch_;
            target_pitch_rate_ = pid_angle_(angle_error, dt_angle);
            target_pitch_rate_ = clamp_value(target_pitch_rate_, -3.2, 3.2);
        }

        double gyro_error = target_pitch_rate_ - pitch_rate_;
        double cmd_raw = pid_gyro_(gyro_error, dt_gyro);
        double cmd_x = clamp_value(cmd_raw * cmd_sign_, -max_cmd_x_, max_cmd_x_);

        publish_cmd(cmd_x, target_yaw_rate_);
        send_wheel_can(cmd_x, target_yaw_rate_);
        send_leg_can(dt);

        std_msgs::msg::Float64MultiArray telem_msg;
        telem_msg.data = {
            x_dot_,                              // [0] 实际前进速度 (m/s)
            target_speed_smoothed_,              // [1] 目标速度 (m/s)
            pitch_,                              // [2] 实际俯仰角 Pitch (rad)
            target_pitch_filtered_,              // [3] 目标俯仰角 (rad)
            pitch_rate_,                         // [4] 实际角速度 Pitch Rate (rad/s)
            target_pitch_rate_,                  // [5] 目标角速度 (rad/s)
            left_cmd_ma_,                        // [6] 左轮电机电流 (mA)
            -right_cmd_ma_,                      // [7] 右轮电机电流(取反) (mA)
            motor_left_hip_.torque_feedback(),   // [8] 关节1: 左Hip力矩 (Nm)
            motor_left_knee_.torque_feedback(),  // [9] 关节2: 左Knee力矩 (Nm)
            motor_right_hip_.torque_feedback(),  // [10] 关节3: 右Hip力矩 (Nm)
            -motor_right_knee_.torque_feedback() // [11] 关节4: 右Knee力矩(取反) (Nm)
        };

        telemetry_pub_->publish(telem_msg);

        if (logging_enabled_)
        {
            double t = now.nanoseconds() * 1e-9;
            log_angle_ << pitch_ << "\n";
            log_target_angle_ << target_pitch_filtered_ << "\n";
            log_timestamp_angle_ << t << "\n";
            log_timestamp_target_angle_ << t << "\n";

            log_speed_ << x_dot_ << "\n";
            log_target_speed_ << target_speed_smoothed_ << "\n";
            log_timestamp_speed_ << t << "\n";
            log_timestamp_target_speed_ << t << "\n";

            log_gyro_ << pitch_rate_ << "\n";
            log_target_gyro_ << target_pitch_rate_ << "\n";
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

            // ==================== 温度日志 ====================

            log_hip_left_motor_temp_
                << temp_left_hip_motor_ << "\n";

            log_hip_left_mos_temp_
                << temp_left_hip_mos_ << "\n";

            log_knee_left_motor_temp_
                << temp_left_knee_motor_ << "\n";

            log_knee_left_mos_temp_
                << temp_left_knee_mos_ << "\n";

            log_hip_right_motor_temp_
                << temp_right_hip_motor_ << "\n";

            log_hip_right_mos_temp_
                << temp_right_hip_mos_ << "\n";

            log_knee_right_motor_temp_
                << temp_right_knee_motor_ << "\n";

            log_knee_right_mos_temp_
                << temp_right_knee_mos_ << "\n";

            log_timestamp_joint_temp_
                << t << "\n";
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "[PID] pitch=%.3f vel=%.2f cmd=%.2f | roll=%.2f° dh=%.1fmm h=%.3f",
                             pitch_, x_dot_, cmd_x, roll_ * 180.0 / M_PI, last_delta_h_ * 1000.0, current_height_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[TEMP] "
            "LH %.1f/%.1f C | "
            "LK %.1f/%.1f C | "
            "RH %.1f/%.1f C | "
            "RK %.1f/%.1f C",
            temp_left_hip_motor_,
            temp_left_hip_mos_,
            temp_left_knee_motor_,
            temp_left_knee_mos_,
            temp_right_hip_motor_,
            temp_right_hip_mos_,
            temp_right_knee_motor_,
            temp_right_knee_mos_);
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

    void interpolate_pid_gains()
    {
        // 动态根据当前腿高比例进行增益插值
        double r = height_ratio();

        balance_offset_ = lerp(balance_offset_min_, balance_offset_max_, r);

        pid_speed_.P = lerp(0.10f, 0.15f, r);
        pid_speed_.I = lerp(0.01f, 0.05f, r);
        pid_speed_.D = lerp(0.0f, 0.0f, r);
        pid_speed_.limit = lerp(0.45f, 0.50f, r);

        pid_angle_.P = lerp(5.0f, 6.0f, r);
        pid_angle_.I = lerp(0.0f, 0.0f, r);
        pid_angle_.D = lerp(0.0f, 0.05f, r);
        pid_angle_.limit = lerp(3.0f, 5.0f, r);

        pid_gyro_.P = lerp(25.0f, 30.0f, r);
        pid_gyro_.I = lerp(0.0f, 0.0f, r);
        pid_gyro_.D = lerp(0.0f, 0.0f, r);
        pid_gyro_.limit = lerp(10.0f, 10.0f, r);

        // pid_speed_.P = lerp(0.0f, 0.0f, r);
        // pid_speed_.I = lerp(0.0f, 0.0f, r);
        // pid_speed_.D = lerp(0.0f, 0.0f, r);
        // pid_speed_.limit = lerp(0.45f, 0.50f, r);

        // pid_angle_.P = lerp(0.0f, 0.0f, r);
        // pid_angle_.I = lerp(0.0f, 0.0f, r);
        // pid_angle_.D = lerp(0.0f, 0.0f, r);
        // pid_angle_.limit = lerp(0.0f, 5.0f, r);

        // pid_gyro_.P = lerp(0.0f, 0.0f, r);
        // pid_gyro_.I = lerp(0.0f, 0.0f, r);
        // pid_gyro_.D = lerp(0.0f, 0.0f, r);
        // pid_gyro_.limit = lerp(0.0f, 10.0f, r);
    }

    // 腿部关节驱动与 Roll 姿态差动高度补偿
    void send_leg_can(double dt)
    {
        // ============================================================
        // Roll 低带宽找平环
        //
        // current_height_：只负责整体升降
        // roll_delta_h_ ：只负责左右腿差动找平
        //
        // 左腿 = H + dh
        // 右腿 = H - dh
        // ============================================================

        const double effective_target_roll = roll_target_ + roll_offset_;
        double roll_error = effective_target_roll - roll_;

        // 1) Roll 死区：小误差不动作，避免 IMU 噪声造成左右腿来回修正
        if (std::abs(roll_error) <= roll_deadband_)
        {
            roll_error = 0.0;
        }
        else if (roll_error > 0.0)
        {
            roll_error -= roll_deadband_;
        }
        else
        {
            roll_error += roll_deadband_;
        }

        // 2) 升降过程中降低 Roll 环增益，避免整体高度控制和差动高度控制互相抢动作
        const bool height_moving =
            std::abs(target_height_ - current_height_) > 0.002;

        const double roll_gain_scale =
            height_moving ? roll_move_gain_scale_ : 1.0;

        // 3) 只在站立完成并且关节已使能后启用 Roll 找平
        double target_delta_h = 0.0;
        if (standup_done_ && joints_enabled_)
        {
            // 暂时使用 PD，不使用积分。
            // D 项使用已经滤波后的 roll_rate_。
            const double raw_delta_h =
                (roll_kp_ * roll_error - roll_kd_ * roll_rate_) * roll_sign_ * roll_gain_scale;

            target_delta_h = clamp_value(
                raw_delta_h,
                -max_delta_h_,
                max_delta_h_);
        }

        // 4) 差动腿长输出限速：这是抑制升降抖动的关键
        const double max_delta_step = roll_delta_h_rate_ * dt;

        if (roll_delta_h_ < target_delta_h)
        {
            roll_delta_h_ = std::min(
                roll_delta_h_ + max_delta_step,
                target_delta_h);
        }
        else if (roll_delta_h_ > target_delta_h)
        {
            roll_delta_h_ = std::max(
                roll_delta_h_ - max_delta_step,
                target_delta_h);
        }

        // 5) 几何边界约束。
        // 正常范围内，左右腿严格保持 H±dh 对称。
        // 接近 L_MIN/L_MAX 时，为了仍然保留 Roll 找平能力，
        // 只把“中心高度”平滑地向可行域内移动必要的距离。
        // 由于 dh 本身已经限速，因此边界处也不会突然升降。
        const double max_safe_h = L_MAX_ - 0.005;

        double delta_h = clamp_value(
            roll_delta_h_,
            -max_delta_h_,
            max_delta_h_);

        // 确保 center_h ± |dh| 都在合法腿长范围内。
        const double abs_delta_h = std::abs(delta_h);
        const double center_min = L_MIN_ + abs_delta_h;
        const double center_max = max_safe_h - abs_delta_h;

        // max_delta_h_ 远小于腿长工作区间，因此正常情况下 center_min < center_max。
        const double center_h = clamp_value(
            current_height_,
            center_min,
            center_max);

        const double h_left = center_h + delta_h;
        const double h_right = center_h - delta_h;

        last_delta_h_ = delta_h;
        last_h_left_ = h_left;
        last_h_right_ = h_right;

        double ratio_l = clamp_value((h_left - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        double ratio_r = clamp_value((h_right - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        double x_off_l = lerp(0.080, 0.077, ratio_l);
        double x_off_r = lerp(0.070, 0.067, ratio_r);

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

            // ====================================================
            // 温度反馈请求
            //
            // 控制周期 5ms = 200Hz
            // 20 个周期请求一次反馈：
            //
            // 20 × 5ms = 100ms
            //
            // 即每个关节温度约 10Hz 更新一次。
            // 平时仍然使用原来的 JointMotorDriver，
            // 只有请求反馈这一帧使用 ack=1。
            // ====================================================
            temperature_request_tick_++;

            const bool request_temperature =
                (temperature_request_tick_ % 20 == 0);

            if (request_temperature)
            {
                // ack = 1 -> 返回报文类型1
                send_servo_position_with_ack(
                    1,
                    hip_deg_l,
                    servo_speed,
                    servo_current,
                    1);

                send_servo_position_with_ack(
                    2,
                    knee_deg_l,
                    servo_speed,
                    servo_current,
                    1);

                send_servo_position_with_ack(
                    3,
                    -hip_deg_r,
                    servo_speed,
                    servo_current,
                    1);

                send_servo_position_with_ack(
                    4,
                    -knee_deg_r,
                    servo_speed,
                    servo_current,
                    1);
            }
            else
            {
                // 其余周期完全保持原来的 Servo 控制方式
                motor_left_hip_.set_servo_position(
                    hip_deg_l,
                    servo_speed,
                    servo_current);

                motor_left_knee_.set_servo_position(
                    knee_deg_l,
                    servo_speed,
                    servo_current);

                motor_right_hip_.set_servo_position(
                    -hip_deg_r,
                    servo_speed,
                    servo_current);

                motor_right_knee_.set_servo_position(
                    -knee_deg_r,
                    servo_speed,
                    servo_current);
            }
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

    void send_wheel_can(double cmd_x, double yaw_rate)
    {
        int16_t base_torque = static_cast<int16_t>(cmd_x * 1000.0);
        int16_t diff_torque = static_cast<int16_t>(yaw_rate * 500.0);

        int16_t left_ma = base_torque + diff_torque;
        int16_t right_ma = -base_torque + diff_torque;

        left_cmd_ma_ = left_ma;
        right_cmd_ma_ = right_ma;

        wheel_.set_torque(left_ma, right_ma);
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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr telemetry_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // IMU状态
    bool imu_received_ = false;
    double pitch_ = 0.0, pitch_rate_ = 0.0;
    double pitch_rate_raw_ = 0.0, pitch_rate_filt_ = 0.0;
    bool pitch_rate_filter_init_ = false;
    double pitch_rate_alpha_ = 0.80;

    // Roll 状态与低带宽找平参数
    double roll_ = 0.0, roll_rate_ = 0.0;

    // Roll角低通
    double roll_filt_ = 0.0;
    bool roll_filter_init_ = false;
    double roll_alpha_ = 0.08;

    // Roll角速度低通。low_pass_filter() 中 alpha 越小越平滑。
    double roll_rate_raw_ = 0.0, roll_rate_filt_ = 0.0;
    bool roll_rate_filter_init_ = false;
    double roll_rate_alpha_ = 0.15;

    double roll_target_ = 0.0;
    double roll_offset_ = 0.0;   // IMU/机械固定零偏补偿 (rad)
    double roll_sign_ = 1.0;     // 补偿方向极性 (+1.0 或 -1.0)
    double roll_kp_ = 0.08;      // Roll -> 差动腿长 P
    double roll_ki_ = 0.0;       // 当前不启用积分
    double roll_kd_ = 0.003;     // Roll角速度阻尼
    double roll_integral_ = 0.0; // 保留变量兼容原有安全复位逻辑
    double roll_integral_limit_ = 0.20;

    double roll_deadband_ = 0.30 * M_PI / 180.0;
    double roll_move_gain_scale_ = 0.25; // 升降时仅保留25% Roll修正
    double roll_delta_h_rate_ = 0.015;   // 差动腿长最大变化速度 (m/s)
    double roll_delta_h_ = 0.0;          // 实际平滑输出的差动腿长 (m)
    double max_delta_h_ = 0.015;         // 单侧最大差动腿长 (m)

    double last_delta_h_ = 0.0;
    double last_h_left_ = 0.36;
    double last_h_right_ = 0.36;

    // 记忆上一次有效关节角度（防止 NaN 导致看门狗超时）
    double last_valid_hip_l_ = 0.0;
    double last_valid_knee_l_ = 0.0;
    double last_valid_hip_r_ = 0.0;
    double last_valid_knee_r_ = 0.0;

    // 轮速
    double x_dot_ = 0.0;

    // 轮子物理参数
    double wheel_radius_;
    double max_wheel_speed_;

    // 平衡参数
    double balance_offset_min_ = -3.0 * M_PI / 180.0; // 蹲伏时的平衡角
    double balance_offset_max_ = -2.0 * M_PI / 180.0; // 站立时的平衡角
    double balance_offset_ = 0.0;                     // 当前插值后的动态偏置

    double cmd_sign_ = 1.0, max_cmd_x_, max_safe_pitch_;
    double target_pitch_filtered_ = 0.0;

    double target_pitch_rate_ = 0.0;
    unsigned int loop_tick_ = 0;
    double target_pitch_alpha_ = 0.90;

    // 遥控指令
    double target_speed_const_ = 0.0, target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_, turn_speed_, speed_ramp_time_;

    // 腿部高度
    double L_MIN_, L_MAX_, current_height_, target_height_, leg_transition_speed_;
    double leg_startup_ramp_time_ = 5.0;
    double startup_elapsed_ = 0.0;
    bool standup_done_ = false;

    // ==================== 键盘高度控制 ====================

    // 每次按 ↑ / ↓ 改变的目标高度
    double keyboard_height_step_ = 0.005; // 5 mm

    // 终端原始配置
    struct termios original_terminal_settings_;

    // stdin 原始 flags
    int original_stdin_flags_ = -1;

    // 键盘功能是否成功初始化
    bool keyboard_enabled_ = false;

    // 腿部控制
    std::string leg_mode_;
    float leg_kp_, leg_kd_;

    bool is_emergency_stopped_ = false;
    bool wheel_over_speed_ = false;

    // 数据日志文件流
    std::ofstream log_angle_, log_target_angle_, log_timestamp_angle_, log_timestamp_target_angle_;
    std::ofstream log_speed_, log_target_speed_, log_timestamp_speed_, log_timestamp_target_speed_;
    std::ofstream log_gyro_, log_target_gyro_, log_timestamp_gyro_, log_timestamp_target_gyro_;
    std::ofstream log_left_current_, log_right_current_, log_timestamp_left_current_, log_timestamp_right_current_;
    std::ofstream log_hip_left_torque_;
    std::ofstream log_knee_left_torque_;
    std::ofstream log_hip_right_torque_;
    std::ofstream log_knee_right_torque_;

    // 关节温度日志
    std::ofstream log_hip_left_motor_temp_;
    std::ofstream log_hip_left_mos_temp_;

    std::ofstream log_knee_left_motor_temp_;
    std::ofstream log_knee_left_mos_temp_;

    std::ofstream log_hip_right_motor_temp_;
    std::ofstream log_hip_right_mos_temp_;

    std::ofstream log_knee_right_motor_temp_;
    std::ofstream log_knee_right_mos_temp_;

    std::ofstream log_timestamp_joint_temp_;

    bool logging_enabled_ = false;

    double left_cmd_ma_ = 0.0;
    double right_cmd_ma_ = 0.0;

    bool joints_enabled_ = false;
    uint32_t torque_query_tick_ = 0;

    // Servo 类型1反馈请求计数器
    uint32_t temperature_request_tick_ = 0;

    // ==================== 关节温度 ====================
    // Motor = 线圈/电机温度
    // MOS   = 驱动MOS温度

    double temp_left_hip_motor_ = 0.0;
    double temp_left_hip_mos_ = 0.0;

    double temp_left_knee_motor_ = 0.0;
    double temp_left_knee_mos_ = 0.0;

    double temp_right_hip_motor_ = 0.0;
    double temp_right_hip_mos_ = 0.0;

    double temp_right_knee_motor_ = 0.0;
    double temp_right_knee_mos_ = 0.0;

    // 电机返回错误码
    uint8_t error_left_hip_ = 0;
    uint8_t error_left_knee_ = 0;
    uint8_t error_right_hip_ = 0;
    uint8_t error_right_knee_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDBalanceController>());
    rclcpp::shutdown();
    return 0;
}