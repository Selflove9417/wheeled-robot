#include "bbot_real/can_interface.hpp"

#include <cstring>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace bbot_real
{

CanInterface::CanInterface() = default;

CanInterface::~CanInterface() { close(); }


// 打开CAN接口
bool CanInterface::open(const std::string & interface_name)
{
    if (is_open_) close();

    // 创建原始CAN套接字
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0)
        throw std::runtime_error("创建CAN套接字失败: " + std::string(strerror(errno)));

    // 获取接口索引
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) == -1)
    {
        close();
        throw std::runtime_error("获取接口索引失败 '" + interface_name + "': " + strerror(errno));
    }

    // 绑定到CAN接口
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_fd_, reinterpret_cast<const struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        close();
        throw std::runtime_error("绑定CAN套接字失败: " + std::string(strerror(errno)));
    }

    // 设为非阻塞
    int flags = fcntl(sock_fd_, F_GETFL, 0);
    if (flags == -1 || fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        close();
        throw std::runtime_error("设置非阻塞失败: " + std::string(strerror(errno)));
    }

    is_open_ = true;
    return true;
}

void CanInterface::close()
{
    if (sock_fd_ >= 0) { ::close(sock_fd_); sock_fd_ = -1; }
    is_open_ = false;
}

bool CanInterface::is_open() const { return is_open_; }

bool CanInterface::send(uint32_t can_id, const uint8_t * data, uint8_t len)
{
    if (!is_open_ || len > 8) return false;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);

    ssize_t n = write(sock_fd_, &frame, sizeof(frame));
    if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)
            return false;  // 缓冲区满，调用方重试
        return false;
    }
    return (n == sizeof(can_frame));
}

// 接收CAN帧
int CanInterface::recv(uint32_t & out_can_id, uint8_t * data, uint8_t max_len)
{
    if (!is_open_) return -1;

    struct can_frame frame;
    ssize_t n = read(sock_fd_, &frame, sizeof(frame));

    if (n < 0)
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    if (n < static_cast<ssize_t>(sizeof(struct can_frame)))
        return 0;  // 不完整帧

    out_can_id = frame.can_id;
    uint8_t copy_len = std::min(frame.can_dlc, max_len);
    std::memcpy(data, frame.data, copy_len);
    return frame.can_dlc;
}

}  // namespace bbot_real
