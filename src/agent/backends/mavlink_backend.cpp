// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "swarmkit/agent/mavlink_backend.h"

#include "mavlink_command_executor.h"
#include "mavlink_mission_protocol.h"
#include "mavlink_state_cache.h"
#include "mavlink_telemetry_decoder.h"
#include "mavlink_udp_transport.h"

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

namespace swarmkit::agent {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)
namespace mav = swarmkit::agent::mavlink;

namespace {

constexpr float kMavlinkDefaultSpeed = -1.0F;
constexpr float kMavlinkDefaultThrottle = -1.0F;
constexpr float kMavlinkForceArmDisarmMagic = 21196.0F;
constexpr float kMavlinkPause = 0.0F;
constexpr float kMavlinkResume = 1.0F;
constexpr int kVelocityCommandPeriodMs = 200;

class MavlinkBackend final : public IDroneBackend {
   public:
    explicit MavlinkBackend(MavlinkBackendConfig config) : config_(std::move(config)) {}

    ~MavlinkBackend() override {
        running_.store(false, std::memory_order_relaxed);
        transport_.Close();
        if (receiver_.joinable()) {
            receiver_.join();
        }
    }

    core::Result Start() override {
        return EnsureReceiverStarted();
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

    BackendHealth GetHealth() const override {
        return state_cache_.Health();
    }

    [[nodiscard]] BackendCapabilities GetCapabilities() const override {
        return mav::MavlinkCommandExecutor::Capabilities(config_);
    }

   private:
    [[nodiscard]] core::Result EnsureReceiverStarted() {
        std::lock_guard<std::mutex> lock(start_mutex_);
        if (receiver_started_) {
            return core::Result::Success();
        }

        if (const core::Result bind_result = transport_.Bind(config_.bind_addr);
            !bind_result.IsOk()) {
            return bind_result;
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
        mavlink_status_t status{};
        mavlink_message_t message{};

        while (running_.load(std::memory_order_relaxed)) {
            const auto datagram = transport_.Receive();
            if (!datagram.has_value()) {
                continue;
            }

            for (std::size_t i = 0; i < datagram->size; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, datagram->bytes[i], &message, &status) ==
                    0) {
                    continue;
                }
                HandleMessage(message, datagram->remote_addr, datagram->remote_len);
            }
        }
    }

    [[nodiscard]] bool WaitForEndpoint() {
        return transport_.WaitForEndpoint(
            std::chrono::milliseconds{config_.peer_discovery_timeout_ms});
    }

    void HandleMessage(const mavlink_message_t& message, const sockaddr_storage& remote_addr,
                       socklen_t remote_len) {
        if (message.sysid != config_.target_system) {
            return;
        }

        transport_.RememberEndpoint(remote_addr, remote_len);

        if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&message, &ack);
            RecordCommandAck(mav::CommandAck{
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
            mission_protocol_.RecordMissionRequest(mav::MissionRequest{
                .seq = request.seq,
                .mission_type = request.mission_type,
            });
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
            mavlink_mission_request_t request{};
            mavlink_msg_mission_request_decode(&message, &request);
            mission_protocol_.RecordMissionRequest(mav::MissionRequest{
                .seq = request.seq,
                .mission_type = request.mission_type,
            });
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
            mavlink_mission_ack_t ack{};
            mavlink_msg_mission_ack_decode(&message, &ack);
            mission_protocol_.RecordMissionAck(mav::MissionAck{
                .type = ack.type,
                .mission_type = ack.mission_type,
            });
            return;
        }

        if (message.compid != config_.target_component) {
            return;
        }

        mav::MavlinkTelemetryDecodeResult decode_result;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            decode_result = telemetry_decoder_.Decode(message, &telemetry_cache_, &state_cache_);
        }

        if (decode_result.should_publish) {
            PublishTelemetry();
        }
        if (decode_result.should_request_intervals) {
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

        mav::TelemetryCache cache;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            cache = telemetry_cache_;
        }

        core::TelemetryFrame frame;
        frame.drone_id = drone_id;
        frame.unix_time_ms = mav::NowUnixMs();
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
            static_cast<float>(mav::kMicrosecondsPerSecond) / static_cast<float>(rate_hz);

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
        if (config_.autopilot_profile == MavlinkAutopilotProfile::kPx4) {
            return SetPx4Mode(mode);
        }

        int custom_mode = mode.custom_mode;
        if (const core::Result result =
                mav::MavlinkCommandExecutor::ResolveCustomMode(config_, mode, &custom_mode);
            !result.IsOk()) {
            return result;
        }
        return SetCustomMode(custom_mode);
    }

    [[nodiscard]] core::Result SetPx4Mode(const CmdSetMode& mode) {
        if (mode.custom_mode >= 0) {
            return SendCommandLong(MAV_CMD_DO_SET_MODE,
                                   static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   static_cast<float>(mode.custom_mode));
        }
        const auto mapped_mode = mav::Px4ModeFromName(mode.mode);
        if (!mapped_mode.has_value()) {
            return core::Result::Rejected("unknown PX4 mode '" + mode.mode +
                                          "'; use --custom-mode or a known PX4 mode");
        }
        return SendCommandLong(
            MAV_CMD_DO_SET_MODE, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            static_cast<float>(mapped_mode->main_mode), static_cast<float>(mapped_mode->sub_mode));
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
        const mav::MavlinkCommandAckResult reposition_result = SendCommandLongDetailed(
            MAV_CMD_DO_REPOSITION, go_to.speed_mps > 0.0F ? go_to.speed_mps : kMavlinkDefaultSpeed,
            static_cast<float>(MAV_DO_REPOSITION_FLAGS_CHANGE_MODE), 0.0F, yaw,
            static_cast<float>(go_to.lat_deg), static_cast<float>(go_to.lon_deg),
            static_cast<float>(go_to.alt_m));
        if (!reposition_result.IsUnsupported()) {
            return reposition_result.ToCoreResult();
        }

        core::Logger::WarnFmt(
            "MavlinkBackend: MAV_CMD_DO_REPOSITION unsupported for drone={}, falling back to "
            "SET_POSITION_TARGET_GLOBAL_INT",
            config_.drone_id);
        return SendSetPositionTargetGlobalInt(go_to);
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
        return SendCommandLongDetailed(command, param1, param2, param3, param4, param5, param6,
                                       param7, wait_for_ack)
            .ToCoreResult();
    }

    [[nodiscard]] mav::MavlinkCommandAckResult SendCommandLongDetailed(
        std::uint16_t command, float param1 = 0.0F, float param2 = 0.0F, float param3 = 0.0F,
        float param4 = 0.0F, float param5 = 0.0F, float param6 = 0.0F, float param7 = 0.0F,
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
            return {.send_result = std::move(send_result)};
        }
        return WaitForCommandAck(command, ack_start_sequence);
    }

    [[nodiscard]] core::Result SendSetPositionTargetGlobalInt(const CmdSetWaypoint& waypoint) {
        return SendSetPositionTargetGlobalInt(waypoint.lat_deg, waypoint.lon_deg, waypoint.alt_m,
                                              waypoint.speed_mps, std::nullopt);
    }

    [[nodiscard]] core::Result SendSetPositionTargetGlobalInt(const CmdGoto& go_to) {
        return SendSetPositionTargetGlobalInt(
            go_to.lat_deg, go_to.lon_deg, go_to.alt_m, go_to.speed_mps,
            go_to.use_yaw ? std::optional<float>{go_to.yaw_deg} : std::nullopt);
    }

    [[nodiscard]] core::Result SendSetPositionTargetGlobalInt(double lat_deg, double lon_deg,
                                                              double alt_m, float speed_mps,
                                                              std::optional<float> yaw_deg) {
        if (speed_mps > 0.0F) {
            if (const core::Result speed_result = SendSetSpeed(speed_mps); !speed_result.IsOk()) {
                return speed_result;
            }
        }

        mavlink_message_t message{};
        auto type_mask = static_cast<std::uint16_t>(
            POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
            POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
            POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);
        if (yaw_deg.has_value()) {
            type_mask =
                static_cast<std::uint16_t>(type_mask & ~POSITION_TARGET_TYPEMASK_YAW_IGNORE);
        }

        mavlink_msg_set_position_target_global_int_pack(
            config_.source_system, config_.source_component, &message, 0, config_.target_system,
            config_.target_component, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
            static_cast<std::int32_t>(std::llround(lat_deg * mav::kDegE7)),
            static_cast<std::int32_t>(std::llround(lon_deg * mav::kDegE7)),
            static_cast<float>(alt_m),
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, yaw_deg.value_or(0.0F), 0.0F);
        return SendMavlinkMessage(message);
    }

    [[nodiscard]] core::Result ExecuteMissionCommand(const MissionCmd& mission) {
        core::Result result = core::Result::Rejected("mission command not handled");
        std::visit(core::Overloaded{
                       [&](const CmdUploadMission& upload) {
                           result = mission_protocol_.UploadMission(
                               upload, config_,
                               [this](const mavlink_message_t& message) {
                                   return SendMavlinkMessage(message);
                               });
                       },
                       [&](const CmdClearMission&) {
                           result = mission_protocol_.ClearMission(
                               config_, [this](const mavlink_message_t& message) {
                                   return SendMavlinkMessage(message);
                               });
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
                           result = mav::MavlinkMissionProtocol::SetCurrentMissionItem(
                               current, config_, [this](const mavlink_message_t& message) {
                                   return SendMavlinkMessage(message);
                               });
                       },
                   },
                   mission);
        return result;
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

        return transport_.Send(send_buffer.data(), length);
    }

    void RecordCommandAck(const mav::CommandAck& ack) {
        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            last_ack_ = ack;
            ++ack_sequence_;
        }
        ack_cv_.notify_all();
        core::Logger::DebugFmt(
            "MavlinkBackend: COMMAND_ACK command={} result={} param2={} target={}/{}", ack.command,
            mav::MavResultName(ack.result), ack.result_param2,
            static_cast<int>(ack.target_system),
            static_cast<int>(ack.target_component));
    }

    [[nodiscard]] mav::MavlinkCommandAckResult WaitForCommandAck(
        std::uint16_t command, std::uint64_t start_sequence) {
        std::unique_lock<std::mutex> lock(ack_mutex_);
        const bool got_ack =
            ack_cv_.wait_for(lock, std::chrono::milliseconds{config_.command_ack_timeout_ms}, [&] {
                if (ack_sequence_ == start_sequence || !last_ack_.has_value()) {
                    return false;
                }
                const mav::CommandAck ack = last_ack_.value_or(mav::CommandAck{});
                return ack.command == command && AckTargetsThisBackend(ack) &&
                       ack.result != MAV_RESULT_IN_PROGRESS;
            });

        if (!got_ack) {
            return {.send_result = core::Result::Failed(
                        "timed out waiting for COMMAND_ACK command=" + std::to_string(command))};
        }

        const mav::CommandAck ack = last_ack_.value_or(mav::CommandAck{});
        return {.ack = ack, .has_ack = true};
    }

    [[nodiscard]] bool AckTargetsThisBackend(const mav::CommandAck& ack) const {
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
    std::thread receiver_;
    mav::MavlinkUdpTransport transport_;

    std::mutex callback_mutex_;
    bool telemetry_active_{false};
    std::string active_drone_id_;
    TelemetryCallback telemetry_callback_;

    std::mutex command_mutex_;

    std::mutex telemetry_mutex_;
    mav::TelemetryCache telemetry_cache_;
    mav::MavlinkTelemetryDecoder telemetry_decoder_;
    mav::MavlinkStateCache state_cache_;

    std::mutex ack_mutex_;
    std::condition_variable ack_cv_;
    std::optional<mav::CommandAck> last_ack_;
    std::uint64_t ack_sequence_{0};

    mav::MavlinkMissionProtocol mission_protocol_;
};

}  // namespace

std::string_view ToString(MavlinkAutopilotProfile profile) noexcept {
    switch (profile) {
        case MavlinkAutopilotProfile::kArdupilotCopter:
            return "ardupilot-copter";
        case MavlinkAutopilotProfile::kArdupilotPlane:
            return "ardupilot-plane";
        case MavlinkAutopilotProfile::kPx4:
            return "px4";
    }
    return "unknown";
}

std::expected<MavlinkAutopilotProfile, core::Result> ParseMavlinkAutopilotProfile(
    std::string_view value) {
    const std::string normalized = mav::ToLower(std::string(value));
    if (normalized == "ardupilot-copter" || normalized == "arducopter" || normalized == "copter") {
        return MavlinkAutopilotProfile::kArdupilotCopter;
    }
    if (normalized == "ardupilot-plane" || normalized == "arduplane" || normalized == "plane") {
        return MavlinkAutopilotProfile::kArdupilotPlane;
    }
    if (normalized == "px4") {
        return MavlinkAutopilotProfile::kPx4;
    }
    return std::unexpected(
        core::Result::Rejected("unsupported mavlink.autopilot_profile '" + std::string(value) +
                               "'; expected ardupilot-copter|ardupilot-plane|px4"));
}

core::Result MavlinkBackendConfig::Validate() const {
    if (drone_id.empty()) {
        return core::Result::Rejected("mavlink.drone_id must not be empty");
    }
    if (!mav::SplitHostPort(bind_addr).has_value()) {
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
