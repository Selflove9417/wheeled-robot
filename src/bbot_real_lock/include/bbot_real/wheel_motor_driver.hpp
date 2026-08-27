#ifndef BBOT_REAL_WHEEL_MOTOR_DRIVER_HPP
#define BBOT_REAL_WHEEL_MOTOR_DRIVER_HPP

#include <cstdint>
#include <memory>
#include "bbot_real/can_interface.hpp"

namespace bbot_real
{

    /**
     * @brief ZLAC双轴轮毂电机驱动（CANopen/CiA402协议）。
     *
     * ZLAC为双轴驱动器，一个CAN节点同时控制左右两个轮毂电机。
     *
     * CAN ID分配:
     *   NMT:     0x000
     *   SDO发送:  0x600 + node_id
     *   SDO接收:  0x580 + node_id
     *   扭矩PDO:  0x200 + node_id (4字节: 左int16 + 右int16, mA)
     *
     * 参考: /home/ubuntu2404/CANopen/zlac_can_interface.cpp
     */
    class WheelMotorDriver
    {
    public:
        WheelMotorDriver();
        ~WheelMotorDriver();

        /// 绑定CAN接口
        bool init(std::shared_ptr<CanInterface> can, uint8_t node_id);

        /**
         * @brief 执行完整的CANopen使能序列。
         * NMT预操作 → 故障复位 → 配置扭矩模式 → 映射RPDO → 设看门狗
         * → NMT启动 → CiA402状态机(Shutdown→SwitchOn→Enable)
         * @return 是否成功进入Operation Enabled状态
         */
        bool enable();

        /// 紧急停止（发送NMT预操作，电机自由停车）
        bool emergency_stop();

        /// 发送左右轮扭矩指令 (mA)，范围 ±30000
        bool set_torque(int16_t left_ma, int16_t right_ma);

        /// 读取CiA402状态字 (0x6041)
        uint32_t read_status();

        /// 读取故障码 (0x603F)
        uint32_t read_fault_code();

        uint8_t node_id() const { return node_id_; }
        bool is_enabled() const { return enabled_; }

        /**
         * @brief 读取双轮电机的实际转速
         * @param out_left_rpm 左轮实际转速输出 (RPM)
         * @param out_right_rpm 右轮实际转速输出 (RPM)
         * @return 是否读取成功
         */
        bool read_motor_rpms(double &out_left_rpm, double &out_right_rpm);

    private:
        std::shared_ptr<CanInterface> can_;
        uint8_t node_id_ = 5;
        bool enabled_ = false;

        /// 发送标准CAN帧（使用CANopen ID）
        bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t len);

        /// SDO写入 (expedited transfer, 1/2/4字节)
        bool write_sdo(uint16_t index, uint8_t subindex, uint32_t data, uint8_t data_len);

        /// SDO读取
        bool read_sdo(uint16_t index, uint8_t subindex, uint32_t &out_data);

        /// NMT命令
        void send_nmt(uint8_t command);

        /// 等待状态字低4位达到期望值
        bool wait_status(uint16_t expected_low4, int timeout_ms);
    };

} // namespace bbot_real

#endif // BBOT_REAL_WHEEL_MOTOR_DRIVER_HPP
