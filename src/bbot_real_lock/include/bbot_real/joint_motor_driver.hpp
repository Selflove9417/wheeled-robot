#ifndef BBOT_REAL_JOINT_MOTOR_DRIVER_HPP
#define BBOT_REAL_JOINT_MOTOR_DRIVER_HPP

#include <cstdint>
#include <memory>
#include "bbot_real/can_interface.hpp"

namespace bbot_real
{

/**
 * @brief RV系列关节伺服电机驱动（膝关节/髋关节）。
 *
 * 使用级联PID位置控制模式（CAN协议模式0x00），
 * 支持位置指令 + 速度前馈 + 扭矩前馈（重力补偿）。
 *
 * CAN帧格式（模式0x00，8字节）:
 *   字节0: [模式(3b)|KP高5b]
 *   字节1: [KP低7b|KD高1b]     KP范围 0~500, KD范围 0~5
 *   字节2: [KD低8b]
 *   字节3: [位置高8b]            位置范围 -12.5~12.5 rad (16位)
 *   字节4: [位置低8b]
 *   字节5: [速度高8b]            速度范围 -18~18 rad/s (12位)
 *   字节6: [速度低4b|扭矩高4b]   扭矩范围 -30~30 Nm (12位)
 *   字节7: [扭矩低8b]
 *
 * 参考: /home/ubuntu2404/CAN/can.c  send_motor_ctrl_cmd()
 */
class JointMotorDriver
{
public:
    struct MotorConfig
    {
        uint16_t can_id = 0;        // CAN ID (1~127)
        double torque_limit = 60.0; // 扭矩限制 (Nm)
        bool inverted = false;      // 是否反转方向
    };

    JointMotorDriver();
    ~JointMotorDriver();

    /// 绑定已打开的CAN接口
    bool init(std::shared_ptr<CanInterface> can, uint16_t motor_id,
              double torque_limit = 60.0, bool inverted = false);

    /**
     * @brief 模式0x00：级联PID位置控制（力位混合）
     * @param pos_rad   目标位置 (rad)
     * @param spd_rad_s 速度前馈 (rad/s)
     * @param tor_nm    扭矩前馈 (Nm)，用于重力补偿
     * @param kp        位置环KP (0~500)
     * @param kd        位置环KD (0~5)
     */
    bool set_position(double pos_rad, double spd_rad_s, double tor_nm,
                      float kp = 30.0f, float kd = 1.0f);

    /**
     * @brief 模式0x01：纯伺服位置控制（无重力补偿，电机内部PID）
     * @param pos_deg   目标位置 (度)
     * @param spd       速度限制（原始值×10，如500=50RPM）
     * @param cur       电流限制（原始值×10，如100=10A）
     */
    bool set_servo_position(double pos_deg, uint16_t spd = 500, uint16_t cur = 1000);

    /// 电机使能（广播ID 0x7FF, cmd=0x01）
    bool enable();

    /// 电机关闭（模式0x03 + 控制状态2 = 变阻尼制动, 发送到电机CAN ID）
    bool disable();

    /// 将当前位置设为零点（广播ID 0x7FF, cmd=0x03）
    bool set_zero_position();

    uint16_t motor_id() const { return config_.can_id; }
    double last_position() const { return last_pos_; }

private:
    MotorConfig config_;
    std::shared_ptr<CanInterface> can_;
    double last_pos_ = 0.0;

    bool send_broadcast_cmd(uint8_t cmd);
    bool send_frame(const uint8_t * data, uint8_t len);

    // 浮点数压缩为协议整数
    static int float_to_uint(double x, double x_min, double x_max, int bits);
};

}  // namespace bbot_real

#endif  // BBOT_REAL_JOINT_MOTOR_DRIVER_HPP
