#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "bbot_real/can_interface.hpp"
#include "bbot_real/joint_motor_driver.hpp"

using namespace std::chrono_literals;

namespace bbot_real
{

    class DualLegTestNode : public rclcpp::Node
    {
    public:
        DualLegTestNode()
            : Node("dual_leg_test_node"), run_time_(0.0)
        {
            // ---- 核心控制参数 ----
            this->declare_parameter<std::string>("can_interface", "can0");
            this->declare_parameter<std::string>("leg_control_mode", "servo"); // "servo" 或 "hybrid"

            // 4 个电机的 ID 参数
            this->declare_parameter<int>("motor_left_hip_id", 1);
            this->declare_parameter<int>("motor_left_knee_id", 2);
            this->declare_parameter<int>("motor_right_hip_id", 3);
            this->declare_parameter<int>("motor_right_knee_id", 4);

            std::string can_if = this->get_parameter("can_interface").as_string();
            leg_mode_ = this->get_parameter("leg_control_mode").as_string();

            int l_hip_id = this->get_parameter("motor_left_hip_id").as_int();
            int l_knee_id = this->get_parameter("motor_left_knee_id").as_int();
            int r_hip_id = this->get_parameter("motor_right_hip_id").as_int();
            int r_knee_id = this->get_parameter("motor_right_knee_id").as_int();

            // ---- 频率配置 ----
            double control_frequency = 20.0; // 目标频率，单位：Hz
            dt_ = 1.0 / control_frequency;   // 计算得到准确的积分步长 dt
            auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(dt_ * 1000.0));

            RCLCPP_INFO(this->get_logger(), "=== 双腿 CAN 测试节点启动 ===");
            RCLCPP_INFO(this->get_logger(), "控制频率: %.1f Hz (周期: %ld ms)", control_frequency, period_ms.count());
            RCLCPP_INFO(this->get_logger(), "控制模式: %s", leg_mode_.c_str());
            RCLCPP_INFO(this->get_logger(), "左腿 ID -> 髋: %d, 膝: %d", l_hip_id, l_knee_id);
            RCLCPP_INFO(this->get_logger(), "右腿 ID -> 髋: %d, 膝: %d", r_hip_id, r_knee_id);

            // Hybrid 模式下的默认测试增益
            leg_kp_ = 20.0f;
            leg_kd_ = 0.8f;

            // ---- 初始化 CAN 总线 ----
            can_ = std::make_shared<bbot_real::CanInterface>();
            try
            {
                can_->open(can_if);
                RCLCPP_INFO(this->get_logger(), "CAN 接口 '%s' 打开成功", can_if.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(this->get_logger(), "CAN 打开失败 '%s': %s", can_if.c_str(), e.what());
                return;
            }

            // ---- 初始化并使能 4 个关节电机 ----
            motor_left_hip_.init(can_, l_hip_id, 75.0);
            motor_left_knee_.init(can_, l_knee_id, 60.0);
            motor_right_hip_.init(can_, r_hip_id, 75.0);
            motor_right_knee_.init(can_, r_knee_id, 60.0);

            RCLCPP_INFO(this->get_logger(), "正在发送全关节电机使能指令...");
            motor_left_hip_.enable();
            motor_left_knee_.enable();
            motor_right_hip_.enable();
            motor_right_knee_.enable();

            // ---- 调试发布器 ----
            cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/dual_leg_test/commands", 10);

            // ---- 控制定时器 ----
            timer_ = this->create_wall_timer(period_ms, std::bind(&DualLegTestNode::control_loop, this));

            RCLCPP_INFO(this->get_logger(), "测试就绪：双腿将开始做交替摆动。");
        }

        ~DualLegTestNode()
        {
            RCLCPP_INFO(this->get_logger(), "正在断开所有电机使能并关闭 CAN...");
            motor_left_hip_.disable();
            motor_left_knee_.disable();
            motor_right_hip_.disable();
            motor_right_knee_.disable();
            if (can_)
            {
                can_->close();
            }
        }

    private:
        void control_loop()
        {
            // 使用与定时器周期严格同步的 dt_
            run_time_ += dt_;

            // 从10度到30度来回摆动
            // 10度转换为弧度
            double amplitude = 10.0 * M_PI / 180.0;
            // 基础偏移为20度，转换为弧度
            double offset = 20.0 * M_PI / 180.0;

            // sin 范围 [-1, 1]，整体角度范围即 [offset - amplitude, offset + amplitude] -> [10°, 30°]
            // 左腿
            // double l_hip_angle = offset + amplitude * std::sin(2.0 * run_time_);
            // double l_knee_angle = -l_hip_angle;

            // // 右腿
            // double r_hip_angle = -l_hip_angle;
            // double r_knee_angle = l_hip_angle;

            // 左腿
            double l_hip_angle = offset;
            double l_knee_angle = -offset;

            // 右腿
            double r_hip_angle = -offset;
            double r_knee_angle = offset;

            // 发布期望角度 (弧度)
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {l_hip_angle, l_knee_angle, r_hip_angle, r_knee_angle};
            cmd_pub_->publish(msg);

            // ---- 分模式发送 CAN 指令 ----
            if (leg_mode_ == "servo")
            {
                // 位置控制
                // 伺服模式：单位转换为度(deg)
                double l_hip_deg = l_hip_angle * 180.0 / M_PI;
                double l_knee_deg = l_knee_angle * 180.0 / M_PI;
                double r_hip_deg = r_hip_angle * 180.0 / M_PI;
                double r_knee_deg = r_knee_angle * 180.0 / M_PI;

                motor_left_hip_.set_servo_position(l_hip_deg);
                motor_left_knee_.set_servo_position(l_knee_deg);
                motor_right_hip_.set_servo_position(r_hip_deg);
                motor_right_knee_.set_servo_position(r_knee_deg);
            }
            else
            {
                // 力位混合模式：直接使用弧度值
                double ff_zero = 0.0;

                motor_left_hip_.set_position(l_hip_angle, 0.0, ff_zero, leg_kp_, leg_kd_);
                motor_left_knee_.set_position(l_knee_angle, 0.0, ff_zero, leg_kp_, leg_kd_);
                motor_right_hip_.set_position(r_hip_angle, 0.0, ff_zero, leg_kp_, leg_kd_);
                motor_right_knee_.set_position(r_knee_angle, 0.0, ff_zero, leg_kp_, leg_kd_);
            }

            // 限制打印频率
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[测试中] 左腿(Hip:%.2f°, Knee:%.2f°) | 右腿(Hip:%.2f°, Knee:%.2f°)",
                                 l_hip_angle * 180.0 / M_PI, l_knee_angle * 180.0 / M_PI,
                                 r_hip_angle * 180.0 / M_PI, r_knee_angle * 180.0 / M_PI);
        }

        // 成员变量
        std::shared_ptr<bbot_real::CanInterface> can_;
        bbot_real::JointMotorDriver motor_left_hip_, motor_left_knee_;
        bbot_real::JointMotorDriver motor_right_hip_, motor_right_knee_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string leg_mode_;
        float leg_kp_;
        float leg_kd_;
        double run_time_;
        double dt_; // 存储计算得到的步长
    };

} // namespace bbot_real

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bbot_real::DualLegTestNode>());
    rclcpp::shutdown();
    return 0;
}