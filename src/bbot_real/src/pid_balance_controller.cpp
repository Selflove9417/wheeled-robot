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
        balance_offset_ = 0.008;
        cmd_sign_ = 1.0;
        max_cmd_x_ = 10.0;
        max_safe_pitch_ = 0.40; // 22.9°

        walk_speed_ = 0.3;
        turn_speed_ = 0.5;
        speed_ramp_time_ = 1.0;

        L_MIN_ = 0.30;
        L_MAX_ = 0.40;
        current_height_ = L_MAX_;
        target_height_ = L_MAX_;
        leg_transition_speed_ = (L_MAX_ - L_MIN_) / 4.0;

        // 初始化上一帧有效关节角度（防止初始为0导致突变）
        bbot_real::IKSolution init_ik = kinematics_.inverse_kinematics(L_MAX_, 0.0, 0.0);
        last_valid_hip_l_ = -init_ik.theta_hip;
        last_valid_knee_l_ = -init_ik.theta_knee;
        last_valid_hip_r_ = -init_ik.theta_hip;
        last_valid_knee_r_ = -init_ik.theta_knee;

        // 腿部控制模式
        this->declare_parameter<std::string>("leg_control_mode", "servo");
        leg_mode_ = this->get_parameter("leg_control_mode").as_string();
        RCLCPP_INFO(this->get_logger(), "腿部控制模式: %s", leg_mode_.c_str());

        // 力位混合控制参数（仅 hybrid 模式使用）
        leg_kp_ = 30.0f;
        leg_kd_ = 1.0f;

        // CAN总线
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

        // 关节电机初始化
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

        // 轮毂电机（ZLAC CANopen双轴驱动器）
        this->declare_parameter<int>("wheel_node_id", 5);
        wheel_node_id_ = this->get_parameter("wheel_node_id").as_int();
        wheel_.init(can_, wheel_node_id_);
        if (can_->is_open() && !wheel_.enable())
            RCLCPP_WARN(this->get_logger(), "轮毂电机使能失败，请检查ZLAC驱动器");
        RCLCPP_INFO(this->get_logger(), "轮毂电机 ZLAC node=%d", wheel_node_id_);

        // 轮毂物理参数
        this->declare_parameter<double>("wheel_radius", 0.07); // 默认 70mm
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        RCLCPP_INFO(this->get_logger(), "轮毂半径: %.3f m", wheel_radius_);

        // 轮速超限保护阈值
        this->declare_parameter<double>("max_wheel_speed", 3.0);
        max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();
        RCLCPP_INFO(this->get_logger(), "轮速超限保护阈值: %.3f m/s", max_wheel_speed_);

        // 订阅与发布
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), 10,
            std::bind(&PIDBalanceController::imu_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10);
        leg_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands", 10);

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

            log_timestamp_left_current_.open(log_dir + "timestamp_left_current.txt");
            log_timestamp_right_current_.open(log_dir + "timestamp_right_current.txt");
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
                motor_left_hip_.parse_feedback(data, len);
            }
            else if (can_id == 2u)
            {
                motor_left_knee_.parse_feedback(data, len);
            }
            else if (can_id == 3u)
            {
                motor_right_hip_.parse_feedback(data, len);
            }
            else if (can_id == 4u)
            {
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

    void control_loop()
    {
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

            startup_elapsed_ = 0.0;

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

            double vel_error = target_speed_smoothed_ - x_dot_;
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
        send_leg_can();

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
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "[PID] pitch=%.3f vel=%.2f cmd=%.2f h=%.3f",
                             pitch_, x_dot_, cmd_x, current_height_);
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

        pid_speed_.P = lerp(0.0f, 0.13f, r);
        pid_speed_.I = lerp(0.0f, 0.0f, r);
        pid_speed_.D = lerp(0.0f, 0.0f, r);
        pid_speed_.limit = lerp(0.45f, 0.50f, r);

        pid_angle_.P = lerp(0.0f, 5.0f, r);
        pid_angle_.I = lerp(0.0f, 0.0f, r);
        pid_angle_.D = lerp(0.0f, 0.0f, r);
        pid_angle_.limit = lerp(0.0f, 5.0f, r);

        pid_gyro_.P = lerp(0.0f, 30.0f, r);
        pid_gyro_.I = lerp(0.0f, 0.0f, r);
        pid_gyro_.D = lerp(0.0f, 0.0f, r);
        pid_gyro_.limit = lerp(0.0f, 10.0f, r);
    }

    void send_leg_can()
    {
        // Roll 平衡补偿高度差计算
        double roll_error = roll_target_ - roll_;
        double delta_h = roll_kp_ * roll_error - roll_kd_ * roll_rate_;
        delta_h = clamp_value(delta_h, -max_delta_h_, max_delta_h_);

        // 左右腿高度分配（保留 0.015m 的安全几何裕量，防止 acos(>1) 导致 NaN）
        double max_safe_h = L_MAX_ - 0.015;
        double h_left = clamp_value(current_height_ - delta_h, L_MIN_, max_safe_h);
        double h_right = clamp_value(current_height_ + delta_h, L_MIN_, max_safe_h);

        double ratio_l = clamp_value((h_left - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        double ratio_r = clamp_value((h_right - L_MIN_) / (L_MAX_ - L_MIN_), 0.0, 1.0);
        double x_off_l = lerp(0.0, 0.065, ratio_l);
        double x_off_r = lerp(0.0, 0.055, ratio_r);

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
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // IMU状态
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
    double roll_kp_ = 0.15;
    double roll_kd_ = 0.01;
    double max_delta_h_ = 0.04;

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
    double balance_offset_, cmd_sign_ = 1.0, max_cmd_x_, max_safe_pitch_;
    double target_pitch_filtered_ = 0.0;
    double target_pitch_rate_ = 0.0;
    unsigned int loop_tick_ = 0;
    double target_pitch_alpha_ = 0.80;

    // 遥控指令
    double target_speed_const_ = 0.0, target_speed_smoothed_ = 0.0;
    double target_yaw_rate_ = 0.0;
    double walk_speed_, turn_speed_, speed_ramp_time_;

    // 腿部高度
    double L_MIN_, L_MAX_, current_height_, target_height_, leg_transition_speed_;
    double leg_startup_ramp_time_ = 5.0;
    double startup_elapsed_ = 0.0;

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
    bool logging_enabled_ = false;

    double left_cmd_ma_ = 0.0;
    double right_cmd_ma_ = 0.0;

    bool joints_enabled_ = false;
    uint32_t torque_query_tick_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDBalanceController>());
    rclcpp::shutdown();
    return 0;
}