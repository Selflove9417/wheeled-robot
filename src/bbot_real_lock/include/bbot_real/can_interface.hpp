#ifndef BBOT_REAL_CAN_INTERFACE_HPP
#define BBOT_REAL_CAN_INTERFACE_HPP

#include <cstdint>
#include <string>

namespace bbot_real
{

/**
 * @brief SocketCAN封装，用于Jetson Orin Nano上的实时电机控制。
 *
 * 创建原始CAN套接字，绑定到指定接口（如"can0"），
 * 提供非阻塞发送和接收。
 *
 * 参考: dwrobot asio-driver/src/io/can.cpp
 */
class CanInterface
{
public:
    CanInterface();
    ~CanInterface();

    /// 打开CAN接口（如 "can0"）
    bool open(const std::string & interface_name);
    void close();
    bool is_open() const;

    /// 发送标准帧（11位ID），返回是否成功
    bool send(uint32_t can_id, const uint8_t * data, uint8_t len);

    /// 非阻塞接收，返回数据长度（0=无数据，-1=错误）
    int recv(uint32_t & out_can_id, uint8_t * data, uint8_t max_len);

    int socket_fd() const { return sock_fd_; }

private:
    int sock_fd_ = -1;
    bool is_open_ = false;
};

}  // namespace bbot_real

#endif  // BBOT_REAL_CAN_INTERFACE_HPP
