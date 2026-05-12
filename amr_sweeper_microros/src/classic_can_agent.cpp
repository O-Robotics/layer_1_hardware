#include <uxr/agent/transport/custom/CustomAgent.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{

constexpr uint8_t START_FLAG = 0x80;
constexpr uint8_t END_FLAG = 0x40;
constexpr uint8_t SEQ_MASK = 0x3F;
constexpr uint32_t STANDARD_CAN_MASK = 0x7FF;

struct Config
{
    std::string can_interface{"can0"};
    uint32_t request_id_min{0x500};
    uint32_t request_id_max{0x57F};
    uint32_t reply_id_offset{0x80};
    bool same_id_reply{false};
    uint32_t verbosity{4};
};

struct Packet
{
    uint32_t source_can_id{0};
    std::vector<uint8_t> payload;
};

struct ReassemblyState
{
    bool active{false};
    uint16_t expected_length{0};
    uint8_t next_sequence{0};
    std::vector<uint8_t> payload;
};

struct AgentContext
{
    Config config;
    int socket_fd{-1};
    pollfd socket_pollfd{};
    std::unordered_map<uint32_t, ReassemblyState> reassembly_by_id;
    std::deque<Packet> completed_packets;
};

std::atomic_bool g_should_run{true};
std::shared_ptr<rclcpp::Node> g_log_node;

rclcpp::Logger get_logger()
{
    if (g_log_node)
    {
        return g_log_node->get_logger();
    }
    return rclcpp::get_logger("amr_sweeper_microros_classic_can_agent");
}

uint32_t parse_u32(const std::string& text)
{
    std::size_t parsed_chars = 0;
    const unsigned long value = std::stoul(text, &parsed_chars, 0);
    if (parsed_chars != text.size())
    {
        throw std::invalid_argument("invalid numeric value: " + text);
    }
    return static_cast<uint32_t>(value);
}

bool parse_bool(const std::string& text)
{
    if (text == "true" || text == "1")
    {
        return true;
    }
    if (text == "false" || text == "0")
    {
        return false;
    }
    throw std::invalid_argument("invalid boolean value: " + text);
}

Config parse_args(const std::vector<std::string>& args)
{
    Config config;
    for (std::size_t i = 1; i < args.size(); ++i)
    {
        const std::string& arg = args[i];
        auto require_value = [&](const std::string& name) -> std::string
        {
            if (i + 1 >= args.size())
            {
                throw std::invalid_argument("missing value for " + name);
            }
            return args[++i];
        };

        if (arg == "--can-interface")
        {
            config.can_interface = require_value(arg);
        }
        else if (arg == "--request-id-min")
        {
            config.request_id_min = parse_u32(require_value(arg));
        }
        else if (arg == "--request-id-max")
        {
            config.request_id_max = parse_u32(require_value(arg));
        }
        else if (arg == "--reply-id-offset")
        {
            config.reply_id_offset = parse_u32(require_value(arg));
        }
        else if (arg == "--same-id-reply")
        {
            config.same_id_reply = parse_bool(require_value(arg));
        }
        else if (arg == "--verbosity")
        {
            config.verbosity = parse_u32(require_value(arg));
        }
        else
        {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    if (config.request_id_min > config.request_id_max)
    {
        throw std::invalid_argument("request_id_min must be <= request_id_max");
    }
    if (config.request_id_max > STANDARD_CAN_MASK)
    {
        throw std::invalid_argument("request CAN range must fit in 11-bit classic CAN");
    }
    if (config.reply_id_offset > STANDARD_CAN_MASK)
    {
        throw std::invalid_argument("reply_id_offset must fit in 11-bit classic CAN");
    }

    return config;
}

void signal_handler(int)
{
    g_should_run.store(false);
}

bool open_socket(AgentContext& context)
{
    context.socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (context.socket_fd < 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "SocketCAN socket creation failed on '%s': %s",
            context.config.can_interface.c_str(),
            std::strerror(errno));
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, context.config.can_interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(context.socket_fd, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "SocketCAN ioctl(SIOCGIFINDEX) failed on '%s': %s",
            context.config.can_interface.c_str(),
            std::strerror(errno));
        ::close(context.socket_fd);
        context.socket_fd = -1;
        return false;
    }

    sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(context.socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "SocketCAN bind failed on '%s': %s",
            context.config.can_interface.c_str(),
            std::strerror(errno));
        ::close(context.socket_fd);
        context.socket_fd = -1;
        return false;
    }

    context.socket_pollfd.fd = context.socket_fd;
    context.socket_pollfd.events = POLLIN;
    return true;
}

bool close_socket(AgentContext& context)
{
    if (context.socket_fd >= 0)
    {
        if (::close(context.socket_fd) != 0)
        {
            return false;
        }
        context.socket_fd = -1;
    }
    return true;
}

std::optional<Packet> consume_frame(AgentContext& context, const can_frame& frame)
{
    if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0)
    {
        return std::nullopt;
    }

    const uint32_t source_can_id = frame.can_id & STANDARD_CAN_MASK;
    if (source_can_id < context.config.request_id_min || source_can_id > context.config.request_id_max)
    {
        return std::nullopt;
    }

    if (frame.can_dlc == 0)
    {
        return std::nullopt;
    }

    const uint8_t control = frame.data[0];
    const bool is_start = (control & START_FLAG) != 0;
    const bool is_end = (control & END_FLAG) != 0;
    const uint8_t sequence = control & SEQ_MASK;

    auto& state = context.reassembly_by_id[source_can_id];

    if (is_start)
    {
        if (frame.can_dlc < 3)
        {
            state = ReassemblyState{};
            return std::nullopt;
        }

        state = ReassemblyState{};
        state.active = true;
        state.expected_length = static_cast<uint16_t>((frame.data[1] << 8) | frame.data[2]);
        state.next_sequence = static_cast<uint8_t>((sequence + 1) & SEQ_MASK);
        state.payload.insert(state.payload.end(), &frame.data[3], &frame.data[frame.can_dlc]);
    }
    else
    {
        if (!state.active || sequence != state.next_sequence)
        {
            state = ReassemblyState{};
            return std::nullopt;
        }

        state.next_sequence = static_cast<uint8_t>((sequence + 1) & SEQ_MASK);
        state.payload.insert(state.payload.end(), &frame.data[1], &frame.data[frame.can_dlc]);
    }

    if (state.payload.size() > state.expected_length)
    {
        state = ReassemblyState{};
        return std::nullopt;
    }

    if (is_end)
    {
        if (state.payload.size() != state.expected_length)
        {
            state = ReassemblyState{};
            return std::nullopt;
        }

        Packet packet;
        packet.source_can_id = source_can_id;
        packet.payload = std::move(state.payload);
        state = ReassemblyState{};
        return packet;
    }

    return std::nullopt;
}

ssize_t recv_packet(
        AgentContext& context,
        eprosima::uxr::CustomEndPoint* source_endpoint,
        uint8_t* buffer,
        size_t buffer_length,
        int timeout_ms,
        eprosima::uxr::TransportRc& transport_rc)
{
    if (!context.completed_packets.empty())
    {
        Packet packet = std::move(context.completed_packets.front());
        context.completed_packets.pop_front();
        if (packet.payload.size() > buffer_length)
        {
            transport_rc = eprosima::uxr::TransportRc::server_error;
            return -1;
        }
        std::memcpy(buffer, packet.payload.data(), packet.payload.size());
        source_endpoint->set_member_value<uint32_t>("can_id", packet.source_can_id);
        transport_rc = eprosima::uxr::TransportRc::ok;
        return static_cast<ssize_t>(packet.payload.size());
    }

    const int poll_result = ::poll(&context.socket_pollfd, 1, timeout_ms);
    if (poll_result == 0)
    {
        transport_rc = eprosima::uxr::TransportRc::timeout_error;
        return 0;
    }
    if (poll_result < 0)
    {
        transport_rc = eprosima::uxr::TransportRc::server_error;
        return -1;
    }

    can_frame frame {};
    const ssize_t bytes_read = ::read(context.socket_fd, &frame, sizeof(frame));
    if (bytes_read < static_cast<ssize_t>(sizeof(can_frame)))
    {
        transport_rc = eprosima::uxr::TransportRc::server_error;
        return -1;
    }

    if (auto packet = consume_frame(context, frame); packet.has_value())
    {
        if (packet->payload.size() > buffer_length)
        {
            transport_rc = eprosima::uxr::TransportRc::server_error;
            return -1;
        }
        std::memcpy(buffer, packet->payload.data(), packet->payload.size());
        source_endpoint->set_member_value<uint32_t>("can_id", packet->source_can_id);
        transport_rc = eprosima::uxr::TransportRc::ok;
        return static_cast<ssize_t>(packet->payload.size());
    }

    transport_rc = eprosima::uxr::TransportRc::timeout_error;
    return 0;
}

bool send_packet(
        AgentContext& context,
        uint32_t destination_can_id,
        uint8_t* buffer,
        size_t message_length,
        eprosima::uxr::TransportRc& transport_rc)
{
    if (message_length > 0xFFFF)
    {
        transport_rc = eprosima::uxr::TransportRc::server_error;
        return false;
    }

    std::size_t offset = 0;
    uint8_t sequence = 0;

    while (offset < message_length || (message_length == 0 && offset == 0))
    {
        can_frame frame {};
        frame.can_id = destination_can_id & STANDARD_CAN_MASK;

        if (offset == 0)
        {
            const std::size_t chunk = std::min<std::size_t>(5, message_length);
            frame.data[0] = START_FLAG | sequence;
            if (chunk == message_length)
            {
                frame.data[0] |= END_FLAG;
            }
            frame.data[1] = static_cast<uint8_t>((message_length >> 8) & 0xFF);
            frame.data[2] = static_cast<uint8_t>(message_length & 0xFF);
            if (chunk > 0)
            {
                std::memcpy(&frame.data[3], buffer, chunk);
            }
            frame.can_dlc = static_cast<__u8>(3 + chunk);
            offset += chunk;
        }
        else
        {
            const std::size_t remaining = message_length - offset;
            const std::size_t chunk = std::min<std::size_t>(7, remaining);
            frame.data[0] = sequence;
            if (chunk == remaining)
            {
                frame.data[0] |= END_FLAG;
            }
            std::memcpy(&frame.data[1], buffer + offset, chunk);
            frame.can_dlc = static_cast<__u8>(1 + chunk);
            offset += chunk;
        }

        if (::write(context.socket_fd, &frame, sizeof(frame)) < 0)
        {
            transport_rc = eprosima::uxr::TransportRc::server_error;
            return false;
        }

        sequence = static_cast<uint8_t>((sequence + 1) & SEQ_MASK);
        if (message_length == 0)
        {
            break;
        }
    }

    transport_rc = eprosima::uxr::TransportRc::ok;
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
        g_log_node = std::make_shared<rclcpp::Node>("amr_sweeper_microros_classic_can_agent");

        AgentContext context;
        context.config = parse_args(args);

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        eprosima::uxr::Middleware::Kind mw_kind(eprosima::uxr::Middleware::Kind::FASTDDS);

        eprosima::uxr::CustomAgent::InitFunction init_function = [&]() -> bool
        {
            return open_socket(context);
        };

        eprosima::uxr::CustomAgent::FiniFunction fini_function = [&]() -> bool
        {
            return close_socket(context);
        };

        eprosima::uxr::CustomAgent::RecvMsgFunction recv_msg_function =
                [&](eprosima::uxr::CustomEndPoint* source_endpoint,
                    uint8_t* buffer,
                    size_t buffer_length,
                    int timeout,
                    eprosima::uxr::TransportRc& transport_rc) -> ssize_t
        {
            return recv_packet(context, source_endpoint, buffer, buffer_length, timeout, transport_rc);
        };

        eprosima::uxr::CustomAgent::SendMsgFunction send_msg_function =
                [&](const eprosima::uxr::CustomEndPoint* destination_endpoint,
                    uint8_t* buffer,
                    size_t message_length,
                    eprosima::uxr::TransportRc& transport_rc) -> ssize_t
        {
            const uint32_t request_can_id = destination_endpoint->get_member<uint32_t>("can_id");
            const uint32_t reply_can_id = context.config.same_id_reply
                    ? request_can_id
                    : (request_can_id + context.config.reply_id_offset);
            if (reply_can_id > STANDARD_CAN_MASK)
            {
                transport_rc = eprosima::uxr::TransportRc::server_error;
                return -1;
            }
            return send_packet(context, reply_can_id, buffer, message_length, transport_rc)
                    ? static_cast<ssize_t>(message_length)
                    : -1;
        };

        eprosima::uxr::CustomEndPoint custom_endpoint;
        custom_endpoint.add_member<uint32_t>("can_id");

        eprosima::uxr::CustomAgent custom_agent(
            "CLASSIC_CAN_CUSTOM",
            &custom_endpoint,
            mw_kind,
            false,
            init_function,
            fini_function,
            send_msg_function,
            recv_msg_function);

        custom_agent.set_verbose_level(static_cast<uint8_t>(context.config.verbosity));
        custom_agent.start();

        while (g_should_run.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        custom_agent.stop();
        g_log_node.reset();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& error)
    {
        if (rclcpp::ok())
        {
            RCLCPP_FATAL(get_logger(), "Unhandled exception: %s", error.what());
            g_log_node.reset();
            rclcpp::shutdown();
        }
        else
        {
            RCUTILS_LOG_FATAL_NAMED(
                "amr_sweeper_microros_classic_can_agent",
                "Unhandled exception before ROS startup: %s",
                error.what());
        }
        return 1;
    }
}
