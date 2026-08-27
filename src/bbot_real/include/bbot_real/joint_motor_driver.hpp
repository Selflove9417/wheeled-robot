#ifndef BBOT_REAL__JOINT_MOTOR_DRIVER_HPP_
#define BBOT_REAL__JOINT_MOTOR_DRIVER_HPP_

#include <memory>
#include <cstdint>
#include "bbot_real/can_interface.hpp"

namespace bbot_real
{
    struct JointMotorConfig
    {
        uint16_t can_id{0};
        double torque_limit{30.0};
        bool inverted{false};
    };

    class JointMotorDriver
    {
    public:
        JointMotorDriver();
        ~JointMotorDriver();

        bool init(
            std::shared_ptr<CanInterface> can,
            uint16_t motor_id,
            double torque_limit = 30.0,
            bool inverted = false);

        bool set_position(
            double pos_rad,
            double spd_rad_s = 0.0,
            double tor_nm = 0.0,
            float kp = 0.0f,
            float kd = 0.0f);

        // 为 spd 和 cur 设置默认值（默认速度 500，默认阈值电流 1000），兼容单参数调用
        bool set_servo_position(
            double pos_deg,
            uint16_t spd = 500,
            uint16_t cur = 1000);

        bool enable();
        bool disable();
        bool set_zero_position();

        bool update_feedback();
        void parse_feedback(const uint8_t *data, uint8_t len);

        // === 反馈数据读取接口 ===
        double current_feedback() const { return current_feedback_; }
        double power_feedback() const { return power_feedback_; }
        double torque_feedback() const { return torque_feedback_; }
        double torque_constant() const { return torque_constant_; }
        double last_pos() const { return last_pos_; }

    private:
        int float_to_uint(double x, double x_min, double x_max, int bits);
        bool send_broadcast_cmd(uint8_t cmd);
        bool send_frame(const uint8_t *data, uint8_t len);

        std::shared_ptr<CanInterface> can_{nullptr};
        JointMotorConfig config_;

        double last_pos_{0.0};

        // 状态与反馈数据
        double current_feedback_{0.0};
        double power_feedback_{0.0};
        double torque_feedback_{0.0};
        double torque_constant_{1.4};
    };

} // namespace bbot_real

#endif // BBOT_REAL__JOINT_MOTOR_DRIVER_HPP_