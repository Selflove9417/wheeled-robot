#include "bbot_real/joint_motor_driver.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace bbot_real
{

// 协议参数范围
static constexpr float KP_MIN = 0.0f,   KP_MAX = 500.0f;
static constexpr float KD_MIN = 0.0f,   KD_MAX = 5.0f;
static constexpr float POS_MIN = -12.5f, POS_MAX = 12.5f;
static constexpr float SPD_MIN = -18.0f, SPD_MAX = 18.0f;
static constexpr float T_MIN = -30.0f,   T_MAX = 30.0f;

JointMotorDriver::JointMotorDriver() = default;
JointMotorDriver::~JointMotorDriver() = default;

bool JointMotorDriver::init(std::shared_ptr<CanInterface> can, uint16_t motor_id,
                             double torque_limit, bool inverted)
{
    if (!can || !can->is_open()) return false;
    can_ = can;
    config_.can_id = motor_id;
    config_.torque_limit = torque_limit;
    config_.inverted = inverted;
    return true;
}

// 浮点数量化为整数
int JointMotorDriver::float_to_uint(double x, double x_min, double x_max, int bits)
{
    double span = x_max - x_min;
    double offset = x_min;
    return static_cast<int>((x - offset) * ((1 << bits) - 1) / span);
}

// 力位混控模式
bool JointMotorDriver::set_position(double pos_rad, double spd_rad_s, double tor_nm,
                                     float kp, float kd)
{
    if (!can_) return false;

    if (config_.inverted)
    {
        pos_rad = -pos_rad;
        spd_rad_s = -spd_rad_s;
        tor_nm = -tor_nm;
    }

    pos_rad   = std::clamp(pos_rad,   static_cast<double>(POS_MIN), static_cast<double>(POS_MAX));
    spd_rad_s = std::clamp(spd_rad_s, static_cast<double>(SPD_MIN), static_cast<double>(SPD_MAX));
    tor_nm    = std::clamp(tor_nm, -config_.torque_limit, config_.torque_limit);
    last_pos_ = pos_rad;

    int kp_int  = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int  = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    int pos_int = float_to_uint(pos_rad, POS_MIN, POS_MAX, 16);
    int spd_int = float_to_uint(spd_rad_s, SPD_MIN, SPD_MAX, 12);
    int tor_int = float_to_uint(tor_nm, T_MIN, T_MAX, 12);

    // 模式0x00: 级联PID位置控制（力位混合）
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

// 位置控制模式（伺服模式）
bool JointMotorDriver::set_servo_position(double pos_deg, uint16_t spd, uint16_t cur)
{
    if (!can_) return false;

    if (config_.inverted) pos_deg = -pos_deg;

    // float32 → 4字节
    union { float f; uint8_t b[4]; } conv;
    conv.f = static_cast<float>(pos_deg);

    uint8_t ack = 0;  // 不应答

    // 模式0x01: 伺服位置控制
    uint8_t data[8];
    data[0] = 0x20 | (conv.b[3] >> 3);
    data[1] = (conv.b[3] << 5) | (conv.b[2] >> 3);
    data[2] = (conv.b[2] << 5) | (conv.b[1] >> 3);
    data[3] = (conv.b[1] << 5) | (conv.b[0] >> 3);
    data[4] = (conv.b[0] << 5) | (spd >> 10);
    data[5] = (spd & 0x3FC) >> 2;
    data[6] = ((spd & 0x03) << 6) | (cur >> 6);
    data[7] = ((cur & 0x3F) << 2) | ack;

    last_pos_ = pos_deg * M_PI / 180.0;  // 存弧度用于调试
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

    // 对应手册 9.1.4 变阻尼制动模式示例指令：0x69 0x00 0x00
    uint8_t data[3] = {0x69, 0x00, 0x00};

    return send_frame(data, 3);
}

bool JointMotorDriver::set_zero_position()
{
    return send_broadcast_cmd(0x03);
}

bool JointMotorDriver::send_broadcast_cmd(uint8_t cmd)
{
    if (!can_) return false;
    // 广播帧: [motor_id高8, motor_id低8, 0x00, cmd]
    uint8_t data[4] = {
        static_cast<uint8_t>((config_.can_id >> 8) & 0xFF),
        static_cast<uint8_t>(config_.can_id & 0xFF),
        0x00,
        cmd
    };
    return can_->send(0x7FF, data, 4);
}

bool JointMotorDriver::send_frame(const uint8_t * data, uint8_t len)
{
    if (!can_) return false;
    return can_->send(config_.can_id, data, len);
}

}  // namespace bbot_real
