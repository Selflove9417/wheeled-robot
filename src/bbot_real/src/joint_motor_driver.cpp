#include "bbot_real/joint_motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace bbot_real
{

    // 协议参数范围
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;

    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    static constexpr float POS_MIN = -12.5f;
    static constexpr float POS_MAX = 12.5f;

    static constexpr float SPD_MIN = -18.0f;
    static constexpr float SPD_MAX = 18.0f;

    static constexpr float T_MIN = -30.0f;
    static constexpr float T_MAX = 30.0f;

    JointMotorDriver::JointMotorDriver() = default;

    JointMotorDriver::~JointMotorDriver() = default;

    bool JointMotorDriver::init(
        std::shared_ptr<CanInterface> can,
        uint16_t motor_id,
        double torque_limit,
        bool inverted)
    {
        if (!can || !can->is_open())
            return false;

        can_ = can;

        config_.can_id = motor_id;
        config_.torque_limit = torque_limit;
        config_.inverted = inverted;

        torque_constant_ = 1.4;
        current_feedback_ = 0.0;
        power_feedback_ = 0.0;
        torque_feedback_ = 0.0;

        return true;
    }

    int JointMotorDriver::float_to_uint(
        double x,
        double x_min,
        double x_max,
        int bits)
    {
        double span = x_max - x_min;
        double offset = x_min;

        return static_cast<int>(
            (x - offset) *
            ((1 << bits) - 1) /
            span);
    }

    bool JointMotorDriver::set_position(
        double pos_rad,
        double spd_rad_s,
        double tor_nm,
        float kp,
        float kd)
    {
        if (!can_)
            return false;

        if (config_.inverted)
        {
            pos_rad = -pos_rad;
            spd_rad_s = -spd_rad_s;
            tor_nm = -tor_nm;
        }

        pos_rad = std::clamp(
            pos_rad,
            static_cast<double>(POS_MIN),
            static_cast<double>(POS_MAX));

        spd_rad_s = std::clamp(
            spd_rad_s,
            static_cast<double>(SPD_MIN),
            static_cast<double>(SPD_MAX));

        tor_nm = std::clamp(
            tor_nm,
            -config_.torque_limit,
            config_.torque_limit);

        last_pos_ = pos_rad;

        int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
        int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
        int pos_int = float_to_uint(pos_rad, POS_MIN, POS_MAX, 16);
        int spd_int = float_to_uint(spd_rad_s, SPD_MIN, SPD_MAX, 12);
        int tor_int = float_to_uint(tor_nm, T_MIN, T_MAX, 12);

        uint8_t data[8];

        data[0] = 0x00 | (kp_int >> 7);
        data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8);
        data[2] = kd_int & 0xFF;
        data[3] = pos_int >> 8;
        data[4] = pos_int & 0xFF;
        data[5] = spd_int >> 4;
        data[6] = ((spd_int & 0x0F) << 4) | (tor_int >> 8);
        data[7] = tor_int & 0xFF;

        return send_frame(data, 8);
    }

    bool JointMotorDriver::set_servo_position(
        double pos_deg,
        uint16_t spd,
        uint16_t cur)
    {
        if (!can_)
            return false;

        if (config_.inverted)
            pos_deg = -pos_deg;

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
        data[7] = ((cur & 0x3F) << 2);

        last_pos_ = pos_deg * M_PI / 180.0;

        return send_frame(data, 8);
    }

    bool JointMotorDriver::enable()
    {
        return send_broadcast_cmd(0x01);
    }

    bool JointMotorDriver::disable()
    {
        if (!can_)
            return false;

        uint8_t data[3] = {0x69, 0x00, 0x00};
        return send_frame(data, 3);
    }

    bool JointMotorDriver::set_zero_position()
    {
        return send_broadcast_cmd(0x03);
    }

    bool JointMotorDriver::send_broadcast_cmd(uint8_t cmd)
    {
        if (!can_)
            return false;

        uint8_t data[4] = {
            static_cast<uint8_t>((config_.can_id >> 8) & 0xff),
            static_cast<uint8_t>(config_.can_id & 0xff),
            0x00,
            cmd};

        return can_->send(0x7FF, data, 4);
    }

    bool JointMotorDriver::send_frame(const uint8_t *data, uint8_t len)
    {
        if (!can_)
            return false;

        return can_->send(config_.can_id, data, len);
    }

    bool JointMotorDriver::update_feedback()
    {
        if (!can_)
            return false;

        uint32_t can_id;
        uint8_t data[8];

        int len = can_->recv(can_id, data, 8);

        if (len <= 0)
            return false;

        // 过滤其它电机反馈
        if (can_id != config_.can_id)
            return false;

        parse_feedback(data, static_cast<uint8_t>(len));

        return true;
    }

    void JointMotorDriver::parse_feedback(
        const uint8_t *data,
        uint8_t len)
    {
        if (len < 2)
            return;

        uint8_t frame_type = (data[0] >> 5) & 0x07;

        // 只处理报文类型 5 (问答查询返回报文)
        if (frame_type != 5)
            return;

        uint8_t query_id = data[1];

        /*
         * 查询 3: 返回实际相电流 (float32, 大端)
         */
        if (query_id == 3 && len >= 6)
        {
            float current;
            uint32_t bits =
                (static_cast<uint32_t>(data[2]) << 24) |
                (static_cast<uint32_t>(data[3]) << 16) |
                (static_cast<uint32_t>(data[4]) << 8) |
                static_cast<uint32_t>(data[5]);
            std::memcpy(&current, &bits, sizeof(current));

            current_feedback_ = current;
            torque_feedback_ = current_feedback_ * torque_constant_;
        }
        /*
         * 查询 4: 返回实际功率 (float32, 大端，单位: W) -> 【新增】
         */
        else if (query_id == 4 && len >= 6)
        {
            float power;
            uint32_t bits =
                (static_cast<uint32_t>(data[2]) << 24) |
                (static_cast<uint32_t>(data[3]) << 16) |
                (static_cast<uint32_t>(data[4]) << 8) |
                static_cast<uint32_t>(data[5]);
            std::memcpy(&power, &bits, sizeof(power));

            power_feedback_ = power;
        }
        /*
         * 查询 22: 返回转矩常数 Kt (uint16, 比例 100)
         */
        else if (query_id == 22 && len >= 4)
        {
            uint16_t kt =
                (static_cast<uint16_t>(data[2]) << 8) |
                data[3];

            torque_constant_ = static_cast<double>(kt) / 100.0;
            torque_feedback_ = current_feedback_ * torque_constant_;
        }
    }

} // namespace bbot_real