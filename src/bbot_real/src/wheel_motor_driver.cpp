#include "bbot_real/wheel_motor_driver.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

namespace bbot_real
{

    WheelMotorDriver::WheelMotorDriver() = default;
    WheelMotorDriver::~WheelMotorDriver() = default;

    bool WheelMotorDriver::init(std::shared_ptr<CanInterface> can, uint8_t node_id)
    {
        if (!can || !can->is_open())
            return false;
        can_ = can;
        node_id_ = node_id;
        return true;
    }

    // ==================== CANopen 协议层 ====================

    bool WheelMotorDriver::send_frame(uint32_t can_id, const uint8_t *data, uint8_t len)
    {
        if (!can_)
            return false;
        return can_->send(can_id, data, len);
    }

    void WheelMotorDriver::send_nmt(uint8_t command)
    {
        uint8_t data[2] = {command, node_id_};
        send_frame(0x000, data, 2);
    }

    bool WheelMotorDriver::write_sdo(uint16_t index, uint8_t subindex, uint32_t data, uint8_t data_len)
    {
        uint8_t tx[8] = {};
        tx[1] = index & 0xFF;
        tx[2] = (index >> 8) & 0xFF;
        tx[3] = subindex;

        // Expedited SDO: 1/2/4字节
        if (data_len == 1)
            tx[0] = 0x2F;
        else if (data_len == 2)
            tx[0] = 0x2B;
        else if (data_len == 4)
            tx[0] = 0x23;
        else
            return false;

        for (int i = 0; i < data_len; ++i)
            tx[4 + i] = (data >> (8 * i)) & 0xFF;

        if (!send_frame(0x600 + node_id_, tx, 8))
            return false;

        // 等待从站应答 (0x580 + node_id)
        uint32_t rx_id;
        uint8_t rx[8];
        auto start = std::chrono::steady_clock::now();
        while (true)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
            if (elapsed >= 100000)
                return false; // 100ms超时

            int n = can_->recv(rx_id, rx, 8);
            if (n > 0)
            {
                if (rx_id == (uint32_t)(0x580 + node_id_))
                {
                    if (rx[0] == 0x60 && rx[1] == tx[1] && rx[2] == tx[2] && rx[3] == subindex)
                        return true;
                    if (rx[0] == 0x80) // Abort
                    {
                        uint32_t abort = rx[4] | (rx[5] << 8) | (rx[6] << 16) | (rx[7] << 24);
                        std::cerr << "[ZLAC SDO] 拒绝! Index=0x" << std::hex << index
                                  << " Abort=0x" << abort << std::dec << std::endl;
                        return false;
                    }
                }
                continue;
            }

            // 只有在完全没有数据可读的时候，才适当挂起线程释放内核
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    bool WheelMotorDriver::read_sdo(uint16_t index, uint8_t subindex, uint32_t &out_data)
    {
        uint8_t tx[8] = {};
        tx[0] = 0x40; // SDO upload request
        tx[1] = index & 0xFF;
        tx[2] = (index >> 8) & 0xFF;
        tx[3] = subindex;

        if (!send_frame(0x600 + node_id_, tx, 8))
            return false;

        uint32_t rx_id;
        uint8_t rx[8];
        auto start = std::chrono::steady_clock::now();
        while (true)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
            if (elapsed >= 100000)
                return false;

            int n = can_->recv(rx_id, rx, 8);
            if (n > 0)
            {
                if (rx_id == (uint32_t)(0x580 + node_id_))
                {
                    if ((rx[0] == 0x4F || rx[0] == 0x4B || rx[0] == 0x43) &&
                        rx[1] == tx[1] && rx[2] == tx[2] && rx[3] == subindex)
                    {
                        out_data = rx[4] | (rx[5] << 8) | (rx[6] << 16) | (rx[7] << 24);
                        return true;
                    }
                }
                continue; // 同上，命中非目标帧继续排空总线缓冲
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    bool WheelMotorDriver::wait_status(uint16_t expected_low4, int timeout_ms)
    {
        int loops = timeout_ms / 5;
        while (loops-- > 0)
        {
            uint32_t status = 0;
            if (read_sdo(0x6041, 0x00, status))
            {
                uint16_t left = status & 0xFFFF;
                uint16_t right = (status >> 16) & 0xFFFF;
                if ((left & 0x0F) == expected_low4 && (right & 0x0F) == expected_low4)
                    return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    // ==================== CiA402 状态机 ====================

    bool WheelMotorDriver::enable()
    {
        if (!can_)
            return false;

        std::cout << "[ZLAC] 节点" << (int)node_id_ << " 开始使能序列..." << std::endl;

        // 1. NMT预操作
        send_nmt(0x80);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 2. 故障复位
        uint32_t status = 0;
        if (read_sdo(0x6041, 0x00, status))
        {
            if ((status & 0x0008) || ((status >> 16) & 0x0008))
            {
                std::cout << "[ZLAC] 检测到故障，尝试复位..." << std::endl;
                write_sdo(0x6040, 0x00, 0x0080, 2); // Fault reset
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // 3. 设定扭矩模式 (0x6060 = 4)
        if (!write_sdo(0x6060, 0x00, 0x04, 1))
        {
            std::cerr << "[ZLAC] 设置扭矩模式失败" << std::endl;
            return false;
        }
        std::cout << "[ZLAC] 扭矩模式已设置" << std::endl;

        // 4. 映射RPDO0 → 0x6071:03
        write_sdo(0x1600, 0x00, 0x00, 1);       // 禁用映射
        write_sdo(0x1600, 0x01, 0x60710320, 4); // 映射0x6071 sub3, 32位
        write_sdo(0x1400, 0x02, 0xFF, 1);       // 异步触发
        write_sdo(0x1600, 0x00, 0x01, 1);       // 使能映射(1个对象)

        // 5. 配置 TPDO0 主动上报实际速度 (0x606C:03, 20ms)
        if (!enable_speed_tpdo())
            std::cerr << "[ZLAC] TPDO0速度反馈配置失败，轮速将不会更新" << std::endl;

        // 6. 看门狗 200ms (0x2000)
        write_sdo(0x2000, 0x00, 200, 2);

        write_sdo(0x2026, 0x03, 2, 2);

        // 释放外部抱闸：0x2030:07 = B0(左轮)，0x2030:08 = B1(右轮)
        if (!write_sdo(0x2030, 0x07, 1, 2) || !write_sdo(0x2030, 0x08, 1, 2))
            std::cerr << "[ZLAC] 抱闸释放失败，轮子可能仍处于锁死状态" << std::endl;

        // 7. NMT启动
        send_nmt(0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 8. CiA402状态机: Shutdown → Switch On → Enable Operation
        write_sdo(0x6040, 0x00, 0x0006, 2);
        if (!wait_status(0x01, 200))
        {
            std::cerr << "[ZLAC] Shutdown失败" << std::endl;
            return false;
        }
        std::cout << "[ZLAC] Shutdown OK" << std::endl;

        write_sdo(0x6040, 0x00, 0x0007, 2);
        if (!wait_status(0x03, 200))
        {
            std::cerr << "[ZLAC] SwitchOn失败" << std::endl;
            return false;
        }
        std::cout << "[ZLAC] SwitchOn OK" << std::endl;

        write_sdo(0x6040, 0x00, 0x000F, 2);
        if (!wait_status(0x07, 200))
        {
            std::cerr << "[ZLAC] Enable失败" << std::endl;
            return false;
        }
        std::cout << "[ZLAC] Operation Enabled" << std::endl;

        enabled_ = true;
        return true;
    }

    bool WheelMotorDriver::emergency_stop()
    {
        enabled_ = false;

        // 先清空 RPDO 扭矩缓冲，避免阻塞式 SDO 失败时残留力矩
        uint8_t zero_data[8] = {};
        send_frame(0x200 + node_id_, zero_data, 8);

        // 进入 NMT 预操作
        send_nmt(0x80);

        // 再通过 SDO 进入 Shutdown，清掉驱动内部状态
        write_sdo(0x6040, 0x00, 0x0006, 2);
        return true;
    }

    // 实时控制

    bool WheelMotorDriver::set_torque(int16_t left_ma, int16_t right_ma)
    {
        if (!can_ || !enabled_)
            return false;

        left_ma = std::clamp(left_ma, static_cast<int16_t>(-30000), static_cast<int16_t>(30000));
        right_ma = std::clamp(right_ma, static_cast<int16_t>(-30000), static_cast<int16_t>(30000));

        // 采用标准8字节全清空数组，后4字节默认赋0x00
        uint8_t data[8] = {};
        data[0] = left_ma & 0xFF;
        data[1] = (left_ma >> 8) & 0xFF;
        data[2] = right_ma & 0xFF;
        data[3] = (right_ma >> 8) & 0xFF;

        return send_frame(0x200 + node_id_, data, 8);
    }

    uint32_t WheelMotorDriver::read_status()
    {
        uint32_t status = 0;
        read_sdo(0x6041, 0x00, status);
        return status;
    }

    uint32_t WheelMotorDriver::read_fault_code()
    {
        uint32_t code = 0;
        read_sdo(0x603F, 0x00, code);
        return code;
    }

    bool WheelMotorDriver::read_motor_rpms(double &out_left_rpm, double &out_right_rpm)
    {
        uint32_t val = 0;
        // 读取 0x606C 子索引 0x03 (左右实际速度组合)
        if (!read_sdo(0x606C, 0x03, val))
        {
            return false;
        }

        // 解析低16位 (左轮) 和高16位 (右轮) 的 16位有符号整数 (I16)
        int16_t left_raw = static_cast<int16_t>(val & 0xFFFF);
        int16_t right_raw = static_cast<int16_t>((val >> 16) & 0xFFFF);

        // 寄存器单位为 0.1 RPM，需乘以 0.1 转换为实际 RPM
        out_left_rpm = left_raw * 0.1;
        out_right_rpm = right_raw * 0.1;
        return true;
    }

    bool WheelMotorDriver::enable_speed_tpdo()
    {
        if (!can_)
            return false;

        // 6.1.1: 清空 TPDO0 映射
        if (!write_sdo(0x1A00, 0x00, 0x00, 1))
            return false;

        // 6.1.2: 将 0x606C:03 映射至 TPDO0 映射 1，32 位
        if (!write_sdo(0x1A00, 0x01, 0x606C0320, 4))
            return false;

        // 6.1.3: 定时器触发 (255)
        if (!write_sdo(0x1800, 0x02, 0xFF, 1))
            return false;

        // 6.1.4: 定时器 20ms，单位 0.5ms，因此写入 40
        if (!write_sdo(0x1800, 0x05, 40, 2))
            return false;

        // 6.1.5: 开启 1 个 TPDO0 映射
        if (!write_sdo(0x1A00, 0x00, 0x01, 1))
            return false;

        // 手册 6.1 还提供 0x2010=1 保存到 EEPROM；
        // 这里每次启动都重新配置，不写 EEPROM，避免永久改变驱动配置。
        return true;
    }

    bool WheelMotorDriver::read_motor_speed_tpdo(double &out_left_rpm, double &out_right_rpm)
    {
        if (!can_)
            return false;

        uint32_t rx_id = 0;
        uint8_t rx[8] = {};
        int n = can_->recv(rx_id, rx, 8);
        if (n <= 0)
            return false;

        // TPDO0 COB-ID 默认是 0x180 + node_id
        if (rx_id != (uint32_t)(0x180 + node_id_))
            return false;

        uint32_t val = rx[0] | (rx[1] << 8) | (rx[2] << 16) | (rx[3] << 24);
        int16_t left_raw = static_cast<int16_t>(val & 0xFFFF);
        int16_t right_raw = static_cast<int16_t>((val >> 16) & 0xFFFF);

        // 寄存器单位为 0.1 RPM，需乘以 0.1 转换为实际 RPM
        out_left_rpm = left_raw * 0.1;
        out_right_rpm = right_raw * 0.1;
        return true;
    }

} // namespace bbot_real
