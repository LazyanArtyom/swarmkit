#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <variant>

namespace swarmkit::core {

struct CmdArm {};
struct CmdDisarm {};
struct CmdLand {};

struct CmdTakeoff {
    double alt_m{};
};

struct CmdSetRole {
    std::string role;
};

struct DataRef {
    std::string transfer_id;
    std::uint64_t size_bytes{};
    std::string mime_type;
};

struct CmdSendData {
    DataRef ref;
    std::string dst_agent_id;
};

using Command = std::variant<CmdArm, CmdDisarm, CmdTakeoff, CmdLand, CmdSetRole, CmdSendData>;

struct CommandContext {
    std::string drone_id;
    int priority{0};
    std::chrono::system_clock::time_point deadline{};
    std::string correlation_id;
};

} // namespace swarmkit::core
