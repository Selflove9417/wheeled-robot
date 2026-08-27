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

    class SingleLegTestNode : public rclcpp::Node
    {
    public:
        SingleLegTestNode()
            : Node("single_leg_test_node"), run_time_(0.0)
        {
            // ---- 核心控制参数 ----
            this->declare_parameter<std::string>("can_interface", "can0");
            this->declare_parameter<std::string>("leg_control_mode", "servo"); // "servo" 或 "hybrid"
            this->declare_parameter<int>("hip_motor_id", 1);
            this->declare_parameter<int>("knee_motor_id", 2);

            std::string can_if = this->get_parameter("can_interface").as_string();
            leg_mode_ = this->get_parameter("leg_control_mode").as_string();
            int hip_id = this->get_parameter("hip_motor_id").as_int();
            int knee_id = this->get_parameter("knee_motor_id").as_int();

            RCLCPP_INFO(this->get_logger(), "=== 单腿 CAN 测试节点启动 ===");
            RCLCPP_INFO(this->get_logger(), "控制模式: %s", leg_mode_.c_str());
            RCLCPP_INFO(this->get_logger(), "目标电机 ID -> 髋关节: %d, 膝关节: %d", hip_id, knee_id);

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

            // ---- 初始化并使能单腿关节电机 ----
            motor_hip_.init(can_, hip_id, 75.0);
            motor_knee_.init(can_, knee_id, 60.0);

            RCLCPP_INFO(this->get_logger(), "正在发送电机使能指令...");
            motor_hip_.enable();
            motor_knee_.enable();

            // ---- 调试发布器 ----
            cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/single_leg_test/commands", 10);

            // ---- 控制定时器 20Hz (50ms) ----
            timer_ = this->create_wall_timer(50ms, std::bind(&SingleLegTestNode::control_loop, this));

            RCLCPP_INFO(this->get_logger(), "测试就绪：单腿将开始缓慢做正弦摆动。");
        }

        ~SingleLegTestNode()
        {
            RCLCPP_INFO(this->get_logger(), "正在断开电机使能并关闭 CAN...");
            motor_hip_.disable();
            motor_knee_.disable();
            if (can_)
            {
                can_->close();
            }
        }

    private:
        void control_loop()
        {
            const double dt = 0.01; // 100Hz
            run_time_ += dt;

            // 添加一个静态计数器
            static int loop_count = 0;
            loop_count++;

            // 每 5 个循环才发送一次（50ms 间隔）
            if (loop_count % 5 != 0)
            {
                return;
            }

            // ---- 构造安全的测试正弦轨迹 ----
            // 髋关节在 0° 到 +15° 之间摆动，膝关节在 0° 到 -15° 之间摆动，周期约 3.14 秒
            double amplitude = 0.1309;                     // 15° 对应的弧度值 (15 * π / 180 ≈ 0.2618 rad，半振幅为 0.1309)
            double base = 1.0 + std::sin(2.0 * run_time_); // 范围 [0, 2]

            double hip_angle = -amplitude * base;
            double knee_angle = amplitude * base;

            // 发布当前期望角度
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {hip_angle, knee_angle};
            cmd_pub_->publish(msg);

            // ---- 分模式发送 CAN 指令 ----
            if (leg_mode_ == "servo")
            {
                // 纯伺服位置控制（模式 0x01）：接收单位为度(deg)
                double hip_deg = hip_angle * 180.0 / M_PI;
                double knee_deg = knee_angle * 180.0 / M_PI;

                motor_hip_.set_servo_position(hip_deg);
                motor_knee_.set_servo_position(knee_deg);
            }
            else
            {
                // 力位混合控制（模式 0x00）：接收单位为弧度(rad)
                double hip_ff = 0.0;
                double knee_ff = 0.0;

                motor_hip_.set_position(hip_angle, 0.0, hip_ff, leg_kp_, leg_kd_);
                usleep(1000);
                motor_knee_.set_position(knee_angle, 0.0, knee_ff, leg_kp_, leg_kd_);
                usleep(1000);
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[测试中] 运行时间: %.2fs | 目标弧度 -> 髋关节: %.3f, 膝关节: %.3f",
                                 run_time_, hip_angle, knee_angle);
        }

        // 成员变量
        std::shared_ptr<bbot_real::CanInterface> can_;
        bbot_real::JointMotorDriver motor_hip_;
        bbot_real::JointMotorDriver motor_knee_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string leg_mode_;
        float leg_kp_;
        float leg_kd_;
        double run_time_;
    };

} // namespace bbot_real

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bbot_real::SingleLegTestNode>());
    rclcpp::shutdown();
    return 0;
}
