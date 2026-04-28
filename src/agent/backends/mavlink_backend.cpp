// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/mavlink_backend.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "swarmkit/core/logger.h"
#include "swarmkit/core/overloaded.h"

extern "C" {
#include "ardupilotmega/mavlink.h"
}

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

constexpr int kMicrosecondsPerSecond = 1000000;
constexpr int kMillisecondsPerSecond = 1000;
constexpr int kUdpReceiveTimeoutMs = 200;
constexpr std::int64_t kUdpReceiveTimeoutUsec = static_cast<std::int64_t>(kUdpReceiveTimeoutMs) *
                                                (kMicrosecondsPerSecond / kMillisecondsPerSecond);
constexpr int kMaxUdpPort = 65535;
constexpr std::size_t kUdpReceiveBufferBytes = 2048;
constexpr double kDegE7 = 10000000.0;
constexpr float kMillimetresPerMetre = 1000.0F;
constexpr float kMavlinkDefaultSpeed = -1.0F;
constexpr float kMavlinkDefaultThrottle = -1.0F;
constexpr float kMavlinkForceArmDisarmMagic = 21196.0F;
constexpr float kMavlinkPause = 0.0F;
constexpr float kMavlinkResume = 1.0F;
constexpr std::uint16_t kMissionUploadMaxItems = 1000;
constexpr int kVelocityCommandPeriodMs = 200;
constexpr float kUnknownBattery = -1.0F;

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] std::string ToLower(std::string value) {
    std::ranges::transform(value, value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

[[nodiscard]] std::optional<int> ArduCopterModeFromName(std::string mode) {
    static const std::unordered_map<std::string, int> kModeMap{
        {"stabilize", 0}, {"acro", 1},          {"alt-hold", 2},   {"althold", 2},
        {"auto", 3},      {"guided", 4},        {"loiter", 5},     {"rtl", 6},
        {"return", 6},    {"circle", 7},        {"land", 9},       {"drift", 11},
        {"sport", 13},    {"flip", 14},         {"auto-tune", 15}, {"autotune", 15},
        {"poshold", 16},  {"pos-hold", 16},     {"brake", 17},     {"throw", 18},
        {"avoid", 19},    {"guided-nogps", 20}, {"smart-rtl", 21}, {"smartrtl", 21},
    };
    const auto iter = kModeMap.find(ToLower(std::move(mode)));
    if (iter == kModeMap.end()) {
        return std::nullopt;
    }
    return iter->second;
}

[[nodiscard]] std::optional<std::pair<std::string, std::uint16_t>> SplitHostPort(
    const std::string& value) {
    const std::size_t colon = value.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= value.size()) {
        return std::nullopt;
    }

    const std::string host = value.substr(0, colon);
    const std::string port_text = value.substr(colon + 1);
    try {
        const int port = std::stoi(port_text);
        if (port <= 0 || port > kMaxUdpPort) {
            return std::nullopt;
        }
        return std::make_pair(host, static_cast<std::uint16_t>(port));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

void CloseSocket(SocketHandle socket_handle) {
    if (socket_handle == kInvalidSocket) {
        return;
    }
    close(socket_handle);
}

[[nodiscard]] core::Result ConfigureReceiveTimeout(SocketHandle socket_handle) {
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = static_cast<suseconds_t>(kUdpReceiveTimeoutUsec);
    if (setsockopt(socket_handle, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
        return core::Result::Failed("failed to configure UDP receive timeout");
    }
    return core::Result::Success();
}

[[nodiscard]] std::string ModeString(const mavlink_heartbeat_t& heartbeat) {
    std::string mode = "MAVLINK";
    if ((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0U) {
        mode += ":ARMED";
    } else {
        mode += ":DISARMED";
    }
    mode += ":custom=" + std::to_string(heartbeat.custom_mode);
    return mode;
}

struct TelemetryCache {
    double lat_deg{};
    double lon_deg{};
    float rel_alt_m{};
    float battery_percent{kUnknownBattery};
    std::string mode{"MAVLINK"};
};

struct CommandAck {
    std::uint16_t command{};
    std::uint8_t result{};
    std::int32_t result_param2{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
};

struct MissionRequest {
    std::uint16_t seq{};
    std::uint8_t mission_type{MAV_MISSION_TYPE_MISSION};
};

struct MissionAck {
    std::uint8_t type{};
    std::uint8_t mission_type{MAV_MISSION_TYPE_MISSION};
};

[[nodiscard]] std::string MavResultName(std::uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED:
            return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED:
            return "TEMPORARILY_REJECTED";
        case MAV_RESULT_DENIED:
            return "DENIED";
        case MAV_RESULT_UNSUPPORTED:
            return "UNSUPPORTED";
        case MAV_RESULT_FAILED:
            return "FAILED";
        case MAV_RESULT_IN_PROGRESS:
            return "IN_PROGRESS";
        case MAV_RESULT_CANCELLED:
            return "CANCELLED";
        case MAV_RESULT_COMMAND_LONG_ONLY:
            return "COMMAND_LONG_ONLY";
        case MAV_RESULT_COMMAND_INT_ONLY:
            return "COMMAND_INT_ONLY";
        case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME:
            return "UNSUPPORTED_MAV_FRAME";
        case MAV_RESULT_NOT_IN_CONTROL:
            return "NOT_IN_CONTROL";
        default:
            return "UNKNOWN(" + std::to_string(result) + ")";
    }
}

[[nodiscard]] std::string MissionResultName(std::uint8_t result) {
    switch (result) {
        case MAV_MISSION_ACCEPTED:
            return "ACCEPTED";
        case MAV_MISSION_ERROR:
            return "ERROR";
        case MAV_MISSION_UNSUPPORTED_FRAME:
            return "UNSUPPORTED_FRAME";
        case MAV_MISSION_UNSUPPORTED:
            return "UNSUPPORTED";
        case MAV_MISSION_NO_SPACE:
            return "NO_SPACE";
        case MAV_MISSION_INVALID:
            return "INVALID";
        case MAV_MISSION_INVALID_PARAM1:
            return "INVALID_PARAM1";
        case MAV_MISSION_INVALID_PARAM2:
            return "INVALID_PARAM2";
        case MAV_MISSION_INVALID_PARAM3:
            return "INVALID_PARAM3";
        case MAV_MISSION_INVALID_PARAM4:
            return "INVALID_PARAM4";
        case MAV_MISSION_INVALID_PARAM5_X:
            return "INVALID_PARAM5_X";
        case MAV_MISSION_INVALID_PARAM6_Y:
            return "INVALID_PARAM6_Y";
        case MAV_MISSION_INVALID_PARAM7:
            return "INVALID_PARAM7";
        case MAV_MISSION_INVALID_SEQUENCE:
            return "INVALID_SEQUENCE";
        case MAV_MISSION_DENIED:
            return "DENIED";
        default:
            return "UNKNOWN(" + std::to_string(result) + ")";
    }
}

class MavlinkBackend final : public IDroneBackend {
   public:
    explicit MavlinkBackend(MavlinkBackendConfig config) : config_(std::move(config)) {}

    ~MavlinkBackend() override {
        running_.store(false, std::memory_order_relaxed);
        CloseSocket(socket_);
        if (receiver_.joinable()) {
            receiver_.join();
        }
    }

    core::Result Execute(const CommandEnvelope& envelope) override {
        if (const core::Result result = EnsureReceiverStarted(); !result.IsOk()) {
            return result;
        }
        if (!WaitForEndpoint()) {
            return core::Result::Rejected("no MAVLink peer discovered yet for target system " +
                                          std::to_string(config_.target_system) + " on " +
                                          config_.bind_addr);
        }

        std::lock_guard<std::mutex> command_lock(command_mutex_);
        core::Result result = core::Result::Rejected("command not handled");
        std::visit(core::Overloaded{
                       [&](const FlightCmd& flight) {
                           result = ExecuteFlightCommand(flight, envelope.context);
                       },
                       [&](const NavCmd& nav) { result = ExecuteNavCommand(nav); },
                       [&](const MissionCmd& mission) { result = ExecuteMissionCommand(mission); },
                       [&](const SwarmCmd&) {
                           result = core::Result::Rejected(
                               "MAVLink backend does not support swarm commands");
                       },
                       [&](const PayloadCmd& payload) { result = ExecutePayloadCommand(payload); },
                   },
                   envelope.command);
        return result;
    }

    core::Result StartTelemetry(const std::string& drone_id, int rate_hertz,
                                TelemetryCallback callback) override {
        if (callback == nullptr) {
            return core::Result::Rejected("telemetry callback must not be empty");
        }
        if (drone_id != config_.drone_id && drone_id != "default") {
            return core::Result::Rejected("MAVLink backend is configured for drone '" +
                                          config_.drone_id + "', not '" + drone_id + "'");
        }

        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (telemetry_active_) {
                return core::Result::Rejected("MAVLink telemetry already running");
            }
            telemetry_active_ = true;
            active_drone_id_ = (drone_id == "default") ? config_.drone_id : drone_id;
            telemetry_callback_ = std::move(callback);
            config_.telemetry_rate_hz = std::max(1, rate_hertz);
        }

        core::Result result = EnsureReceiverStarted();
        if (!result.IsOk()) {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            telemetry_active_ = false;
            telemetry_callback_ = nullptr;
            return result;
        }
        return core::Result::Success();
    }

    core::Result StopTelemetry(const std::string& drone_id) override {
        if (drone_id != config_.drone_id && drone_id != "default") {
            return core::Result::Success();
        }

        std::lock_guard<std::mutex> lock(callback_mutex_);
        telemetry_active_ = false;
        telemetry_callback_ = nullptr;
        active_drone_id_.clear();
        return core::Result::Success();
    }

   private:
    [[nodiscard]] core::Result EnsureReceiverStarted() {
        std::lock_guard<std::mutex> lock(start_mutex_);
        if (receiver_started_) {
            return core::Result::Success();
        }

        const auto host_port = SplitHostPort(config_.bind_addr);
        if (!host_port.has_value()) {
            return core::Result::Rejected("mavlink bind_addr must be in host:port format");
        }

        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == kInvalidSocket) {
            return core::Result::Failed("failed to create MAVLink UDP socket");
        }

        if (const core::Result timeout_result = ConfigureReceiveTimeout(socket_);
            !timeout_result.IsOk()) {
            CloseSocket(socket_);
            socket_ = kInvalidSocket;
            return timeout_result;
        }

        sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(host_port->second);
        if (inet_pton(AF_INET, host_port->first.c_str(), &bind_addr.sin_addr) != 1) {
            CloseSocket(socket_);
            socket_ = kInvalidSocket;
            return core::Result::Rejected("mavlink bind host must be an IPv4 address");
        }

        if (bind(socket_, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
            CloseSocket(socket_);
            socket_ = kInvalidSocket;
            return core::Result::Failed("failed to bind MAVLink UDP socket to " +
                                        config_.bind_addr);
        }

        running_.store(true, std::memory_order_relaxed);
        receiver_ = std::thread([this]() { ReceiverLoop(); });
        receiver_started_ = true;

        core::Logger::InfoFmt("MavlinkBackend: listening on {} target_sys={} target_comp={}",
                              config_.bind_addr, static_cast<int>(config_.target_system),
                              static_cast<int>(config_.target_component));
        return core::Result::Success();
    }

    void ReceiverLoop() {
        std::array<std::uint8_t, kUdpReceiveBufferBytes> buffer{};
        mavlink_status_t status{};
        mavlink_message_t message{};

        while (running_.load(std::memory_order_relaxed)) {
            sockaddr_storage remote_addr{};
            socklen_t remote_len = sizeof(remote_addr);
            const auto byte_count =
                recvfrom(socket_, buffer.data(), buffer.size(), 0,
                         reinterpret_cast<sockaddr*>(&remote_addr), &remote_len);
            if (byte_count <= 0) {
                continue;
            }

            const auto valid_byte_count = static_cast<std::size_t>(byte_count);
            for (std::size_t i = 0; i < valid_byte_count; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 0) {
                    continue;
                }
                HandleMessage(message, remote_addr, remote_len);
            }
        }
    }

    void RememberEndpoint(const sockaddr_storage& remote_addr, socklen_t remote_len) {
        {
            std::lock_guard<std::mutex> lock(endpoint_mutex_);
            last_remote_addr_ = remote_addr;
            last_remote_len_ = remote_len;
            endpoint_known_ = true;
        }
        endpoint_cv_.notify_all();
    }

    [[nodiscard]] bool WaitForEndpoint() {
        std::unique_lock<std::mutex> lock(endpoint_mutex_);
        return endpoint_cv_.wait_for(lock,
                                     std::chrono::milliseconds{config_.peer_discovery_timeout_ms},
                                     [&] { return endpoint_known_; });
    }

    void HandleMessage(const mavlink_message_t& message, const sockaddr_storage& remote_addr,
                       socklen_t remote_len) {
        if (message.sysid != config_.target_system) {
            return;
        }

        RememberEndpoint(remote_addr, remote_len);

        if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&message, &ack);
            RecordCommandAck(CommandAck{
                .command = ack.command,
                .result = ack.result,
                .result_param2 = ack.result_param2,
                .target_system = ack.target_system,
                .target_component = ack.target_component,
            });
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
            mavlink_mission_request_int_t request{};
            mavlink_msg_mission_request_int_decode(&message, &request);
            RecordMissionRequest(MissionRequest{
                .seq = request.seq,
                .mission_type = request.mission_type,
            });
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
            mavlink_mission_request_t request{};
            mavlink_msg_mission_request_decode(&message, &request);
            RecordMissionRequest(MissionRequest{
                .seq = request.seq,
                .mission_type = request.mission_type,
            });
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
            mavlink_mission_ack_t ack{};
            mavlink_msg_mission_ack_decode(&message, &ack);
            RecordMissionAck(MissionAck{
                .type = ack.type,
                .mission_type = ack.mission_type,
            });
            return;
        }

        if (message.compid != config_.target_component) {
            return;
        }

        bool should_publish = false;
        bool should_request_intervals = false;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t heartbeat{};
                    mavlink_msg_heartbeat_decode(&message, &heartbeat);
                    telemetry_cache_.mode = ModeString(heartbeat);
                    should_publish = true;
                    if (!message_intervals_requested_) {
                        message_intervals_requested_ = true;
                        should_request_intervals = true;
                    }
                    break;
                }
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t position{};
                    mavlink_msg_global_position_int_decode(&message, &position);
                    telemetry_cache_.lat_deg = static_cast<double>(position.lat) / kDegE7;
                    telemetry_cache_.lon_deg = static_cast<double>(position.lon) / kDegE7;
                    telemetry_cache_.rel_alt_m =
                        static_cast<float>(position.relative_alt) / kMillimetresPerMetre;
                    should_publish = true;
                    break;
                }
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t sys_status{};
                    mavlink_msg_sys_status_decode(&message, &sys_status);
                    if (sys_status.battery_remaining >= 0) {
                        telemetry_cache_.battery_percent =
                            static_cast<float>(sys_status.battery_remaining);
                    }
                    should_publish = true;
                    break;
                }
                case MAVLINK_MSG_ID_BATTERY_STATUS: {
                    mavlink_battery_status_t battery{};
                    mavlink_msg_battery_status_decode(&message, &battery);
                    if (battery.battery_remaining >= 0) {
                        telemetry_cache_.battery_percent =
                            static_cast<float>(battery.battery_remaining);
                    }
                    should_publish = true;
                    break;
                }
                default:
                    break;
            }
        }

        if (should_publish) {
            PublishTelemetry();
        }
        if (should_request_intervals) {
            RequestTelemetryIntervals();
        }
    }

    void PublishTelemetry() {
        TelemetryCallback callback;
        std::string drone_id;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (!telemetry_active_ || !telemetry_callback_) {
                return;
            }
            callback = telemetry_callback_;
            drone_id = active_drone_id_.empty() ? config_.drone_id : active_drone_id_;
        }

        TelemetryCache cache;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            cache = telemetry_cache_;
        }

        core::TelemetryFrame frame;
        frame.drone_id = drone_id;
        frame.unix_time_ms = NowUnixMs();
        frame.lat_deg = cache.lat_deg;
        frame.lon_deg = cache.lon_deg;
        frame.rel_alt_m = cache.rel_alt_m;
        frame.battery_percent = cache.battery_percent;
        frame.mode = cache.mode;
        callback(frame);
    }

    void RequestTelemetryIntervals() {
        const int rate_hz = std::max(1, config_.telemetry_rate_hz);
        const float interval_us =
            static_cast<float>(kMicrosecondsPerSecond) / static_cast<float>(rate_hz);

        const auto request_interval = [&](std::uint32_t message_id) {
            const core::Result result =
                SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, static_cast<float>(message_id),
                                interval_us, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, false);
            if (!result.IsOk()) {
                core::Logger::WarnFmt(
                    "MavlinkBackend: failed to request message interval msgid={} detail={}",
                    message_id, result.message);
            }
        };

        request_interval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
        request_interval(MAVLINK_MSG_ID_SYS_STATUS);
        request_interval(MAVLINK_MSG_ID_BATTERY_STATUS);
    }

    [[nodiscard]] core::Result ExecuteFlightCommand(const FlightCmd& flight,
                                                    const CommandContext& context) {
        core::Result result = core::Result::Rejected("flight command not handled");
        std::visit(
            core::Overloaded{
                [&](const CmdArm&) {
                    if (config_.set_guided_before_arm) {
                        const core::Result mode_result = SetConfiguredGuidedMode();
                        if (!mode_result.IsOk()) {
                            result = core::Result::Failed(
                                "failed to set configured mode before arm: " + mode_result.message);
                            return;
                        }
                    }
                    result = SendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1.0F);
                },
                [&](const CmdDisarm&) {
                    result = SendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0.0F);
                },
                [&](const CmdTakeoff& takeoff) {
                    if (config_.set_guided_before_takeoff) {
                        const core::Result mode_result = SetConfiguredGuidedMode();
                        if (!mode_result.IsOk()) {
                            result = core::Result::Failed(
                                "failed to set configured takeoff mode before "
                                "takeoff: " +
                                mode_result.message);
                            return;
                        }
                    }
                    result = SendCommandLong(MAV_CMD_NAV_TAKEOFF, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                             0.0F, static_cast<float>(takeoff.alt_m));
                },
                [&](const CmdLand&) { result = SendCommandLong(MAV_CMD_NAV_LAND); },
                [&](const CmdSetMode& mode) { result = SetMode(mode); },
                [&](const CmdForceDisarm&) {
                    if (context.priority != CommandPriority::kEmergency) {
                        result = core::Result::Rejected("force-disarm requires emergency priority");
                        return;
                    }
                    result = SendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0.0F,
                                             kMavlinkForceArmDisarmMagic);
                },
                [&](const CmdFlightTerminate&) {
                    if (context.priority != CommandPriority::kEmergency) {
                        result =
                            core::Result::Rejected("flight-terminate requires emergency priority");
                        return;
                    }
                    if (!config_.allow_flight_termination) {
                        result = core::Result::Rejected(
                            "flight termination is disabled in MAVLink backend config");
                        return;
                    }
                    result = SendCommandLong(MAV_CMD_DO_FLIGHTTERMINATION, 1.0F);
                },
            },
            flight);
        return result;
    }

    [[nodiscard]] core::Result ExecuteNavCommand(const NavCmd& nav) {
        core::Result result = core::Result::Rejected("nav command not handled");
        std::visit(
            core::Overloaded{
                [&](const CmdSetWaypoint& waypoint) {
                    result = SendSetPositionTargetGlobalInt(waypoint);
                },
                [&](const CmdReturnHome&) {
                    result = SendCommandLong(MAV_CMD_NAV_RETURN_TO_LAUNCH);
                },
                [&](const CmdHoldPosition&) { result = SendCommandLong(MAV_CMD_NAV_LOITER_UNLIM); },
                [&](const CmdSetSpeed& speed) { result = SendSetSpeed(speed.ground_mps); },
                [&](const CmdGoto& go_to) { result = SendReposition(go_to); },
                [&](const CmdPause&) {
                    result = SendCommandLong(MAV_CMD_DO_PAUSE_CONTINUE, kMavlinkPause);
                },
                [&](const CmdResume&) {
                    result = SendCommandLong(MAV_CMD_DO_PAUSE_CONTINUE, kMavlinkResume);
                },
                [&](const CmdSetYaw& yaw) {
                    result =
                        SendCommandLong(MAV_CMD_CONDITION_YAW, yaw.yaw_deg, yaw.rate_deg_s,
                                        yaw.relative ? 1.0F : 0.0F, yaw.relative ? 1.0F : 0.0F);
                },
                [&](const CmdVelocity& velocity) { result = SendVelocity(velocity); },
                [&](const CmdSetHome& home) { result = SendSetHome(home); },
            },
            nav);
        return result;
    }

    [[nodiscard]] core::Result SetConfiguredGuidedMode() {
        return SetCustomMode(config_.guided_mode);
    }

    [[nodiscard]] core::Result SetCustomMode(int custom_mode) {
        return SendCommandLong(MAV_CMD_DO_SET_MODE,
                               static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                               static_cast<float>(custom_mode));
    }

    [[nodiscard]] core::Result SetMode(const CmdSetMode& mode) {
        int custom_mode = mode.custom_mode;
        if (custom_mode < 0) {
            const auto mapped_mode = ArduCopterModeFromName(mode.mode);
            if (!mapped_mode.has_value()) {
                return core::Result::Rejected("unknown mode '" + mode.mode +
                                              "'; use a known ArduCopter mode or --custom-mode");
            }
            custom_mode = *mapped_mode;
        }
        return SetCustomMode(custom_mode);
    }

    [[nodiscard]] core::Result SendSetSpeed(float ground_mps) {
        const float speed = ground_mps > 0.0F ? ground_mps : kMavlinkDefaultSpeed;
        return SendCommandLong(MAV_CMD_DO_CHANGE_SPEED, static_cast<float>(SPEED_TYPE_GROUNDSPEED),
                               speed, kMavlinkDefaultThrottle);
    }

    [[nodiscard]] core::Result SendReposition(const CmdGoto& go_to) {
        if (go_to.speed_mps > 0.0F) {
            if (const core::Result speed_result = SendSetSpeed(go_to.speed_mps);
                !speed_result.IsOk()) {
                return speed_result;
            }
        }

        const float yaw = go_to.use_yaw ? go_to.yaw_deg : std::numeric_limits<float>::quiet_NaN();
        return SendCommandLong(MAV_CMD_DO_REPOSITION,
                               go_to.speed_mps > 0.0F ? go_to.speed_mps : kMavlinkDefaultSpeed,
                               static_cast<float>(MAV_DO_REPOSITION_FLAGS_CHANGE_MODE), 0.0F, yaw,
                               static_cast<float>(go_to.lat_deg), static_cast<float>(go_to.lon_deg),
                               static_cast<float>(go_to.alt_m));
    }

    [[nodiscard]] core::Result SendVelocity(const CmdVelocity& velocity) {
        if (velocity.duration_ms <= 0) {
            return SendVelocityOnce(velocity);
        }

        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds{velocity.duration_ms};
        while (std::chrono::steady_clock::now() < deadline) {
            if (const core::Result result = SendVelocityOnce(velocity); !result.IsOk()) {
                return result;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{kVelocityCommandPeriodMs});
        }

        CmdVelocity stop{};
        stop.body_frame = velocity.body_frame;
        return SendVelocityOnce(stop);
    }

    [[nodiscard]] core::Result SendVelocityOnce(const CmdVelocity& velocity) {
        mavlink_message_t message{};
        const auto type_mask = static_cast<std::uint16_t>(
            POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
            POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
            POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);

        mavlink_msg_set_position_target_local_ned_pack(
            config_.source_system, config_.source_component, &message, 0, config_.target_system,
            config_.target_component,
            velocity.body_frame ? MAV_FRAME_BODY_NED : MAV_FRAME_LOCAL_NED, type_mask, 0.0F, 0.0F,
            0.0F, velocity.vx_mps, velocity.vy_mps, velocity.vz_mps, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        return SendMavlinkMessage(message);
    }

    [[nodiscard]] core::Result SendSetHome(const CmdSetHome& home) {
        return SendCommandLong(MAV_CMD_DO_SET_HOME, home.use_current ? 1.0F : 0.0F, 0.0F, 0.0F,
                               0.0F, static_cast<float>(home.lat_deg),
                               static_cast<float>(home.lon_deg), static_cast<float>(home.alt_m));
    }

    [[nodiscard]] core::Result SendCommandLong(std::uint16_t command, float param1 = 0.0F,
                                               float param2 = 0.0F, float param3 = 0.0F,
                                               float param4 = 0.0F, float param5 = 0.0F,
                                               float param6 = 0.0F, float param7 = 0.0F,
                                               bool wait_for_ack = true) {
        std::uint64_t ack_start_sequence{};
        if (wait_for_ack) {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            ack_start_sequence = ack_sequence_;
        }

        mavlink_message_t message{};
        mavlink_msg_command_long_pack(config_.source_system, config_.source_component, &message,
                                      config_.target_system, config_.target_component, command, 0,
                                      param1, param2, param3, param4, param5, param6, param7);
        core::Result send_result = SendMavlinkMessage(message);
        if (!send_result.IsOk() || !wait_for_ack) {
            return send_result;
        }
        return WaitForCommandAck(command, ack_start_sequence);
    }

    [[nodiscard]] core::Result SendSetPositionTargetGlobalInt(const CmdSetWaypoint& waypoint) {
        if (waypoint.speed_mps > 0.0F) {
            if (const core::Result speed_result = SendSetSpeed(waypoint.speed_mps);
                !speed_result.IsOk()) {
                return speed_result;
            }
        }

        mavlink_message_t message{};
        const auto type_mask = static_cast<std::uint16_t>(
            POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
            POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
            POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);

        mavlink_msg_set_position_target_global_int_pack(
            config_.source_system, config_.source_component, &message, 0, config_.target_system,
            config_.target_component, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
            static_cast<std::int32_t>(std::llround(waypoint.lat_deg * kDegE7)),
            static_cast<std::int32_t>(std::llround(waypoint.lon_deg * kDegE7)),
            static_cast<float>(waypoint.alt_m), 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        return SendMavlinkMessage(message);
    }

    [[nodiscard]] core::Result ExecuteMissionCommand(const MissionCmd& mission) {
        core::Result result = core::Result::Rejected("mission command not handled");
        std::visit(core::Overloaded{
                       [&](const CmdUploadMission& upload) { result = UploadMission(upload); },
                       [&](const CmdClearMission&) {
                           mavlink_message_t message{};
                           mavlink_msg_mission_clear_all_pack(
                               config_.source_system, config_.source_component, &message,
                               config_.target_system, config_.target_component,
                               MAV_MISSION_TYPE_MISSION);
                           const std::uint64_t start_sequence = CaptureMissionAckSequence();
                           if (const core::Result send_result = SendMavlinkMessage(message);
                               !send_result.IsOk()) {
                               result = send_result;
                               return;
                           }
                           result = WaitForMissionAck(start_sequence);
                       },
                       [&](const CmdStartMission& start) {
                           result = SendCommandLong(MAV_CMD_MISSION_START,
                                                    static_cast<float>(start.first_item),
                                                    static_cast<float>(start.last_item));
                       },
                       [&](const CmdPauseMission&) {
                           result = SendCommandLong(MAV_CMD_DO_PAUSE_CONTINUE, kMavlinkPause);
                       },
                       [&](const CmdResumeMission&) {
                           result = SendCommandLong(MAV_CMD_DO_PAUSE_CONTINUE, kMavlinkResume);
                       },
                       [&](const CmdSetCurrentMissionItem& current) {
                           mavlink_message_t message{};
                           mavlink_msg_mission_set_current_pack(
                               config_.source_system, config_.source_component, &message,
                               config_.target_system, config_.target_component,
                               static_cast<std::uint16_t>(std::max(0, current.seq)));
                           result = SendMavlinkMessage(message);
                       },
                   },
                   mission);
        return result;
    }

    [[nodiscard]] core::Result UploadMission(const CmdUploadMission& upload) {
        if (upload.items.empty()) {
            return core::Result::Rejected("mission upload requires at least one item");
        }
        if (upload.items.size() > kMissionUploadMaxItems) {
            return core::Result::Rejected("mission upload item count exceeds limit");
        }

        const std::uint64_t request_start_sequence = CaptureMissionRequestSequence();
        const std::uint64_t ack_start_sequence = CaptureMissionAckSequence();

        mavlink_message_t count_message{};
        mavlink_msg_mission_count_pack(
            config_.source_system, config_.source_component, &count_message, config_.target_system,
            config_.target_component, static_cast<std::uint16_t>(upload.items.size()),
            MAV_MISSION_TYPE_MISSION, 0);
        if (const core::Result send_result = SendMavlinkMessage(count_message);
            !send_result.IsOk()) {
            return send_result;
        }

        std::uint64_t request_sequence = request_start_sequence;
        for (std::size_t sent_items = 0; sent_items < upload.items.size(); ++sent_items) {
            const auto request = WaitForMissionRequest(request_sequence);
            if (!request.has_value()) {
                return core::Result::Failed("timed out waiting for MISSION_REQUEST");
            }
            request_sequence = request->second;
            const std::uint16_t seq = request->first.seq;
            if (seq >= upload.items.size()) {
                return core::Result::Failed("autopilot requested invalid mission seq=" +
                                            std::to_string(seq));
            }
            if (const core::Result item_result = SendMissionItem(upload.items[seq], seq);
                !item_result.IsOk()) {
                return item_result;
            }
        }

        return WaitForMissionAck(ack_start_sequence);
    }

    [[nodiscard]] core::Result SendMissionItem(const MissionItem& item, std::uint16_t seq) {
        mavlink_message_t message{};
        mavlink_msg_mission_item_int_pack(
            config_.source_system, config_.source_component, &message, config_.target_system,
            config_.target_component, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, item.command,
            item.current ? 1 : 0, item.autocontinue ? 1 : 0, item.param1, item.param2, item.param3,
            item.param4, static_cast<std::int32_t>(std::llround(item.lat_deg * kDegE7)),
            static_cast<std::int32_t>(std::llround(item.lon_deg * kDegE7)),
            static_cast<float>(item.alt_m), MAV_MISSION_TYPE_MISSION);
        return SendMavlinkMessage(message);
    }

    [[nodiscard]] core::Result ExecutePayloadCommand(const PayloadCmd& payload) {
        core::Result result = core::Result::Rejected("payload command not handled");
        std::visit(
            core::Overloaded{
                [&](const CmdPhoto& photo) {
                    result = SendCommandLong(MAV_CMD_IMAGE_START_CAPTURE,
                                             static_cast<float>(photo.camera_id), 0.0F, 1.0F);
                },
                [&](const CmdPhotoIntervalStart& photo) {
                    result = SendCommandLong(MAV_CMD_IMAGE_START_CAPTURE,
                                             static_cast<float>(photo.camera_id), photo.interval_s,
                                             static_cast<float>(photo.count));
                },
                [&](const CmdPhotoIntervalStop& photo) {
                    result = SendCommandLong(MAV_CMD_IMAGE_STOP_CAPTURE,
                                             static_cast<float>(photo.camera_id));
                },
                [&](const CmdVideoStart& video) {
                    result = SendCommandLong(MAV_CMD_VIDEO_START_CAPTURE,
                                             static_cast<float>(video.stream_id), 0.0F,
                                             static_cast<float>(video.camera_id));
                },
                [&](const CmdVideoStop& video) {
                    result = SendCommandLong(MAV_CMD_VIDEO_STOP_CAPTURE,
                                             static_cast<float>(video.stream_id),
                                             static_cast<float>(video.camera_id));
                },
                [&](const CmdGimbalPoint& gimbal) {
                    result = SendCommandLong(MAV_CMD_DO_MOUNT_CONTROL, gimbal.pitch_deg,
                                             gimbal.roll_deg, gimbal.yaw_deg, 0.0F, 0.0F, 0.0F,
                                             static_cast<float>(MAV_MOUNT_MODE_MAVLINK_TARGETING));
                },
                [&](const CmdRoiLocation& roi) {
                    result = SendCommandLong(
                        MAV_CMD_DO_SET_ROI_LOCATION, static_cast<float>(roi.gimbal_id), 0.0F, 0.0F,
                        0.0F, static_cast<float>(roi.lat_deg), static_cast<float>(roi.lon_deg),
                        static_cast<float>(roi.alt_m));
                },
                [&](const CmdRoiClear& roi) {
                    result =
                        SendCommandLong(MAV_CMD_DO_SET_ROI_NONE, static_cast<float>(roi.gimbal_id));
                },
                [&](const CmdServo& servo) {
                    result = SendCommandLong(MAV_CMD_DO_SET_SERVO, static_cast<float>(servo.servo),
                                             static_cast<float>(servo.pwm));
                },
                [&](const CmdRelay& relay) {
                    result = SendCommandLong(MAV_CMD_DO_SET_RELAY, static_cast<float>(relay.relay),
                                             relay.enabled ? 1.0F : 0.0F);
                },
                [&](const CmdGripper& gripper) {
                    result =
                        SendCommandLong(MAV_CMD_DO_GRIPPER, static_cast<float>(gripper.gripper),
                                        gripper.release ? static_cast<float>(GRIPPER_ACTION_RELEASE)
                                                        : static_cast<float>(GRIPPER_ACTION_GRAB));
                },
            },
            payload);
        return result;
    }

    [[nodiscard]] core::Result SendMavlinkMessage(const mavlink_message_t& message) {
        std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> send_buffer{};
        const std::uint16_t length = mavlink_msg_to_send_buffer(send_buffer.data(), &message);

        sockaddr_storage remote_addr{};
        socklen_t remote_len{};
        {
            std::lock_guard<std::mutex> lock(endpoint_mutex_);
            if (!endpoint_known_) {
                return core::Result::Rejected("no MAVLink peer endpoint known yet");
            }
            remote_addr = last_remote_addr_;
            remote_len = last_remote_len_;
        }

        const auto sent = sendto(socket_, reinterpret_cast<const char*>(send_buffer.data()), length,
                                 0, reinterpret_cast<const sockaddr*>(&remote_addr), remote_len);
        if (sent != static_cast<ssize_t>(length)) {
            return core::Result::Failed(
                "failed to send MAVLink UDP packet: sent=" + std::to_string(sent) +
                " expected=" + std::to_string(length));
        }
        return core::Result::Success();
    }

    void RecordCommandAck(const CommandAck& ack) {
        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            last_ack_ = ack;
            ++ack_sequence_;
        }
        ack_cv_.notify_all();
        core::Logger::DebugFmt(
            "MavlinkBackend: COMMAND_ACK command={} result={} param2={} target={}/{}", ack.command,
            MavResultName(ack.result), ack.result_param2, static_cast<int>(ack.target_system),
            static_cast<int>(ack.target_component));
    }

    void RecordMissionRequest(const MissionRequest& request) {
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            last_mission_request_ = request;
            ++mission_request_sequence_;
        }
        mission_cv_.notify_all();
        core::Logger::DebugFmt("MavlinkBackend: MISSION_REQUEST seq={} type={}", request.seq,
                               static_cast<int>(request.mission_type));
    }

    void RecordMissionAck(const MissionAck& ack) {
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            last_mission_ack_ = ack;
            ++mission_ack_sequence_;
        }
        mission_cv_.notify_all();
        core::Logger::DebugFmt("MavlinkBackend: MISSION_ACK result={} type={}",
                               MissionResultName(ack.type), static_cast<int>(ack.mission_type));
    }

    [[nodiscard]] std::uint64_t CaptureMissionRequestSequence() {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        return mission_request_sequence_;
    }

    [[nodiscard]] std::uint64_t CaptureMissionAckSequence() {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        return mission_ack_sequence_;
    }

    [[nodiscard]] std::optional<std::pair<MissionRequest, std::uint64_t>> WaitForMissionRequest(
        std::uint64_t start_sequence) {
        std::unique_lock<std::mutex> lock(mission_mutex_);
        const bool got_request = mission_cv_.wait_for(
            lock, std::chrono::milliseconds{config_.command_ack_timeout_ms}, [&] {
                return mission_request_sequence_ != start_sequence &&
                       last_mission_request_.has_value();
            });
        if (!got_request) {
            return std::nullopt;
        }
        return std::make_pair(last_mission_request_.value_or(MissionRequest{}),
                              mission_request_sequence_);
    }

    [[nodiscard]] core::Result WaitForMissionAck(std::uint64_t start_sequence) {
        std::unique_lock<std::mutex> lock(mission_mutex_);
        const bool got_ack = mission_cv_.wait_for(
            lock, std::chrono::milliseconds{config_.command_ack_timeout_ms}, [&] {
                return mission_ack_sequence_ != start_sequence && last_mission_ack_.has_value();
            });
        if (!got_ack) {
            return core::Result::Failed("timed out waiting for MISSION_ACK");
        }

        const MissionAck ack = last_mission_ack_.value_or(MissionAck{});
        const std::string detail = "MISSION_ACK result=" + MissionResultName(ack.type) +
                                   " type=" + std::to_string(ack.mission_type);
        if (ack.type == MAV_MISSION_ACCEPTED) {
            return core::Result::Success(detail);
        }
        return core::Result::Failed(detail);
    }

    [[nodiscard]] core::Result WaitForCommandAck(std::uint16_t command,
                                                 std::uint64_t start_sequence) {
        std::unique_lock<std::mutex> lock(ack_mutex_);
        const bool got_ack =
            ack_cv_.wait_for(lock, std::chrono::milliseconds{config_.command_ack_timeout_ms}, [&] {
                if (ack_sequence_ == start_sequence || !last_ack_.has_value()) {
                    return false;
                }
                const CommandAck ack = last_ack_.value_or(CommandAck{});
                return ack.command == command && AckTargetsThisBackend(ack) &&
                       ack.result != MAV_RESULT_IN_PROGRESS;
            });

        if (!got_ack) {
            return core::Result::Failed("timed out waiting for COMMAND_ACK command=" +
                                        std::to_string(command));
        }

        const CommandAck ack = last_ack_.value_or(CommandAck{});
        const std::string detail = "COMMAND_ACK command=" + std::to_string(ack.command) +
                                   " result=" + MavResultName(ack.result) +
                                   " param2=" + std::to_string(ack.result_param2) +
                                   " target=" + std::to_string(ack.target_system) + "/" +
                                   std::to_string(ack.target_component);
        switch (ack.result) {
            case MAV_RESULT_ACCEPTED:
                return core::Result::Success(detail);
            case MAV_RESULT_TEMPORARILY_REJECTED:
            case MAV_RESULT_DENIED:
            case MAV_RESULT_UNSUPPORTED:
            case MAV_RESULT_CANCELLED:
            case MAV_RESULT_COMMAND_LONG_ONLY:
            case MAV_RESULT_COMMAND_INT_ONLY:
            case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME:
            case MAV_RESULT_NOT_IN_CONTROL:
                return core::Result::Rejected(detail);
            case MAV_RESULT_FAILED:
            default:
                return core::Result::Failed(detail);
        }
    }

    [[nodiscard]] bool AckTargetsThisBackend(const CommandAck& ack) const {
        if (ack.target_system == 0 && ack.target_component == 0) {
            return true;
        }
        return ack.target_system == config_.source_system &&
               ack.target_component == config_.source_component;
    }

    MavlinkBackendConfig config_;

    std::mutex start_mutex_;
    bool receiver_started_{false};
    std::atomic<bool> running_{false};
    SocketHandle socket_{kInvalidSocket};
    std::thread receiver_;

    std::mutex endpoint_mutex_;
    std::condition_variable endpoint_cv_;
    sockaddr_storage last_remote_addr_{};
    socklen_t last_remote_len_{};
    bool endpoint_known_{false};

    std::mutex callback_mutex_;
    bool telemetry_active_{false};
    std::string active_drone_id_;
    TelemetryCallback telemetry_callback_;

    std::mutex command_mutex_;

    std::mutex telemetry_mutex_;
    TelemetryCache telemetry_cache_;
    bool message_intervals_requested_{false};

    std::mutex ack_mutex_;
    std::condition_variable ack_cv_;
    std::optional<CommandAck> last_ack_;
    std::uint64_t ack_sequence_{0};

    std::mutex mission_mutex_;
    std::condition_variable mission_cv_;
    std::optional<MissionRequest> last_mission_request_;
    std::optional<MissionAck> last_mission_ack_;
    std::uint64_t mission_request_sequence_{0};
    std::uint64_t mission_ack_sequence_{0};
};

}  // namespace

core::Result MavlinkBackendConfig::Validate() const {
    if (drone_id.empty()) {
        return core::Result::Rejected("mavlink.drone_id must not be empty");
    }
    if (!SplitHostPort(bind_addr).has_value()) {
        return core::Result::Rejected("mavlink.bind_addr must be in host:port format");
    }
    if (target_system == 0) {
        return core::Result::Rejected("mavlink.target_system must be > 0");
    }
    if (target_component == 0) {
        return core::Result::Rejected("mavlink.target_component must be > 0");
    }
    if (source_system == 0) {
        return core::Result::Rejected("mavlink.source_system must be > 0");
    }
    if (source_component == 0) {
        return core::Result::Rejected("mavlink.source_component must be > 0");
    }
    if (telemetry_rate_hz <= 0) {
        return core::Result::Rejected("mavlink.telemetry_rate_hz must be > 0");
    }
    if (peer_discovery_timeout_ms <= 0) {
        return core::Result::Rejected("mavlink.peer_discovery_timeout_ms must be > 0");
    }
    if (command_ack_timeout_ms <= 0) {
        return core::Result::Rejected("mavlink.command_ack_timeout_ms must be > 0");
    }
    if (guided_mode < 0) {
        return core::Result::Rejected("mavlink.guided_mode must be >= 0");
    }
    return core::Result::Success();
}

DroneBackendPtr MakeMavlinkBackend(MavlinkBackendConfig config) {
    if (const core::Result result = config.Validate(); !result.IsOk()) {
        core::Logger::WarnFmt("MakeMavlinkBackend: invalid config: {}", result.message);
    }
    return std::make_unique<MavlinkBackend>(std::move(config));
}

}  // namespace swarmkit::agent
