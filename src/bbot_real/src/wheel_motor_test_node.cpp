#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "bbot_real/can_interface.hpp"
#include "bbot_real/wheel_motor_driver.hpp"

using namespace std::chrono_literals;

namespace bbot_real
{

    class WheelMotorTestNode : public rclcpp::Node
    {
    public:
        WheelMotorTestNode()
            : Node("wheel_motor_test_node"), run_time_(0.0)
        {
            // ---- 核心控制参数声明 ----
            this->declare_parameter<std::string>("can_interface", "can0");
            this->declare_parameter<int>("wheel_motor_node_id", 5); // ZLAC8015D 驱动器 ID

            std::string can_if = this->get_parameter("can_interface").as_string();
            int node_id = this->get_parameter("wheel_motor_node_id").as_int();

            RCLCPP_INFO(this->get_logger(), "=== 轮毂电机力矩测试节点启动 ===");
            RCLCPP_INFO(this->get_logger(), "CAN网络接口: %s", can_if.c_str());
            RCLCPP_INFO(this->get_logger(), "目标驱动器 ID: %d", node_id);

            // ---- 初始化公共抽象 CAN 总线 ----
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

            // ---- 初始化并绑定驱动实例 ----
            if (!wheel_driver_.init(can_, node_id))
            {
                RCLCPP_FATAL(this->get_logger(), "驱动实例初始化失败，请检查 CAN 指针或状态！");
                return;
            }

            // ---- 执行转矩模式使能序列 ----
            RCLCPP_INFO(this->get_logger(), "正在配置驱动器转矩模式并执行控制字使能握手...");
            if (wheel_driver_.enable())
            {
                RCLCPP_INFO(this->get_logger(), "轮毂电机使能完全成功！");
            }
            else
            {
                RCLCPP_FATAL(this->get_logger(), "轮毂电机使能失败或握手超时！");
                return;
            }

            // ---- 调试发布器 ----
            cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_motor_test/commands", 10);

            // ---- 控制定时器 200Hz (5ms)，匹配平衡环路主频 ----
            timer_ = this->create_wall_timer(20ms, std::bind(&WheelMotorTestNode::control_loop, this));

            RCLCPP_INFO(this->get_logger(), "测试就绪：轮毂电机开始做正弦力矩晃动。请确保车体已被完全架空！");
        }

        ~WheelMotorTestNode()
        {
            RCLCPP_WARN(this->get_logger(), "测试节点正在退出，下发紧急停车命令...");
            // 安全退出：切断电流输出并打回预操作状态，防止电机残留缓冲区数据持续飞车
            wheel_driver_.emergency_stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (can_)
            {
                can_->close();
            }
        }

    private:
        void control_loop()
        {
            const double dt = 0.02; // 50Hz
            run_time_ += dt;

            // ---- 提高控制电流信号 (单位: mA) ----
            double omega = 6.28; // 1Hz 的晃动频率

            // 3A ~ 5A 的电流足以克服轮毂电机的电刷和减速箱摩擦力，你会明显看到/感到轮子在正反剧烈摆动
            double target_current_ma = 4000.0 * std::sin(omega * run_time_);

            // 镜像配置
            int16_t t_left = static_cast<int16_t>(target_current_ma);
            int16_t t_right = static_cast<int16_t>(-target_current_ma);

            // 发布期望电流
            std_msgs::msg::Float64MultiArray msg;
            msg.data = {static_cast<double>(t_left), static_cast<double>(t_right)};
            cmd_pub_->publish(msg);

            // 下发高频实时 PDO 力矩指令
            wheel_driver_.set_torque(t_left, t_right);

            // 打印日志
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[增大电流测试中] 当前输出电流 -> 左轮: %d mA | 右轮: %d mA", t_left, t_right);

            // // 恒定向前的力矩电流值 (单位: mA)
            // const int16_t forward_current = 2000;
            // int16_t t_left = forward_current;
            // int16_t t_right = forward_current;

            // // 发布期望电流
            // std_msgs::msg::Float64MultiArray msg;
            // msg.data = {static_cast<double>(t_left), static_cast<double>(t_right)};
            // cmd_pub_->publish(msg);

            // // 下发力矩指令
            // wheel_driver_.set_torque(t_left, t_right);

            // // 打印日志
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            //                      "恒向前力矩: 左轮 %d mA, 右轮 %d mA", t_left, t_right);
        }

        // 成员变量
        std::shared_ptr<bbot_real::CanInterface> can_;
        bbot_real::WheelMotorDriver wheel_driver_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        double run_time_;
    };

} // namespace bbot_real

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bbot_real::WheelMotorTestNode>());
    rclcpp::shutdown();
    return 0;
}