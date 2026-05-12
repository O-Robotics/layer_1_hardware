#include "socket_can.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <cstring>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>
#include <net/if.h>
#include <sys/ioctl.h>

namespace
{
const rclcpp::Logger& logger()
{
    static const auto kLogger = rclcpp::get_logger("amr_sweeper_odrive.socket_can");
    return kLogger;
}
}

bool SocketCanIntf::init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor) {
    interface_ = interface;
    event_loop_ = event_loop;
    frame_processor_ = std::move(frame_processor);
    socket_id_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_id_ == -1) {
        RCLCPP_ERROR(logger(), "Failed to create SocketCAN socket on '%s': %s", interface_.c_str(), std::strerror(errno));
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_.c_str());
    if (ioctl(socket_id_, SIOCGIFINDEX, &ifr) == -1) {
        RCLCPP_ERROR(logger(), "Failed to resolve SocketCAN interface '%s': %s", interface_.c_str(), std::strerror(errno));
        close(socket_id_);
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_id_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        RCLCPP_ERROR(logger(), "Failed to bind SocketCAN interface '%s': %s", interface_.c_str(), std::strerror(errno));
        close(socket_id_);
        return false;
    }

    struct msghdr message = {
        .msg_name = nullptr,
        .msg_namelen = 0,
        .msg_iov = nullptr,
        .msg_iovlen = 0,
        .msg_control = nullptr,
        .msg_controllen = 0,
        .msg_flags = 0
    };

    int retcode = recvmsg(socket_id_, &message, 0);
    if (retcode < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close(socket_id_);
        socket_id_ = 0;
        return false;
    }

    if (!event_loop_->register_event(&socket_evt_id_, socket_id_, EPOLLIN, [this](uint32_t mask) { on_socket_event(mask); })) {
        RCLCPP_ERROR(logger(), "Failed to register SocketCAN fd for interface '%s' with event loop", interface_.c_str());
        close(socket_id_);
        socket_id_ = 0;
        return false;
    }

    return true;
}

void SocketCanIntf::deinit() {
    if (!broken_) {
        event_loop_->deregister_event(socket_evt_id_);
    }
    close(socket_id_);
    broken_ = true;
}

bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    ssize_t nbytes = write(socket_id_, &frame, sizeof(frame));
    if (nbytes == -1) {
        RCLCPP_ERROR(logger(), "Failed to send CAN frame on '%s': %s", interface_.c_str(), std::strerror(errno));
        return false;
    }

    return true;
}

void SocketCanIntf::on_socket_event(uint32_t mask) {
    if (mask & EPOLLIN) {
        while (read_nonblocking() && !broken_);
    }
    if (mask & EPOLLERR) {
        RCLCPP_ERROR(logger(), "SocketCAN interface '%s' disappeared", interface_.c_str());
        deinit();
        return;
    }
    if (mask & ~(EPOLLIN | EPOLLERR)) {
        RCLCPP_ERROR(logger(), "Unexpected epoll mask 0x%X on SocketCAN interface '%s'", mask, interface_.c_str());
        deinit();
        return;
    }
    return;
}

bool SocketCanIntf::read_nonblocking() {
    struct can_frame frame;
    struct cmsghdr ctrlmsg;

    struct iovec vec = {.iov_base = &frame, .iov_len = sizeof(frame)};
    struct msghdr message = {
        .msg_name = nullptr,
        .msg_namelen = 0,
        .msg_iov = &vec, 
        .msg_iovlen = 1,
        .msg_control = &ctrlmsg,
        .msg_controllen = sizeof(ctrlmsg),
        .msg_flags = 0
        };

    ssize_t n_received = recvmsg(socket_id_, &message, MSG_DONTWAIT);
    if (n_received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // std::cerr << "no message received" << std::endl;
            return false;
        } else {
            RCLCPP_ERROR(logger(), "SocketCAN read failed on '%s': %s", interface_.c_str(), std::strerror(errno));
            return false;
        }
    }

    if (n_received < static_cast<ssize_t>(sizeof(struct can_frame))) {
        RCLCPP_ERROR(logger(), "Invalid CAN frame length on '%s': %zd", interface_.c_str(), n_received);
        return true;
    }

    process_can_frame(frame);
    return true;
}
