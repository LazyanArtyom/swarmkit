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
#include <cstring>
#include <deque>
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

#include "mavlink_command_executor.h"
#include "mavlink_state_cache.h"
#include "mavlink_telemetry_decoder.h"
#include "mavlink_udp_transport.h"
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
constexpr float kMavlinkPause = 0.0F;
constexpr float kMavlinkResume = 1.0F;
constexpr int kVelocityCommandPeriodMs = 200;
constexpr std::size_t kMaxCommandAckHistory = 32;
constexpr std::size_t kMaxArmStateHistory = 16;
constexpr auto kArmStateConfirmationGrace = std::chrono::milliseconds{1000};

struct SequencedArmState {
    std::uint64_t sequence{};
    bool armed{false};
};

struct ArmStateObservation {
    bool observed_desired_after_start{false};
    bool current_desired{false};
};

[[nodiscard]] std::optional<float> FloatParam(
    const std::unordered_map<std::string, std::string>& params, const std::string& name) {
    const auto iter = params.find(name);
    if (iter == params.end()) {
        return std::nullopt;
    }
    try {
        return std::stof(iter->second);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

[[nodiscard]] std::optional<std::uint16_t> CommandParam(
    const std::unordered_map<std::string, std::string>& params) {
    const auto iter = params.find("command");
    if (iter == params.end()) {
        return std::nullopt;
    }
    try {
        const int command = std::stoi(iter->second);
        if (command > 0 && command <= std::numeric_limits<std::uint16_t>::max()) {
            return static_cast<std::uint16_t>(command);
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return std::nullopt;
}

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
                       [&](const PayloadCmd& payload) { result = ExecutePayloadCommand(payload); },
                       [&](const BackendCmd& backend) { result = ExecuteBackendCommand(backend); },
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
        BackendHealth health = state_cache_.Health();
        if (health.custom_mode >= 0) {
            mavlink_heartbeat_t heartbeat{};
            heartbeat.custom_mode = static_cast<std::uint32_t>(health.custom_mode);
            health.mode = mav::ModeString(heartbeat, config_.autopilot_profile);
        }
        return health;
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

        if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
            mavlink_statustext_t status_text{};
            mavlink_msg_statustext_decode(&message, &status_text);
            const auto text_length = strnlen(status_text.text, sizeof(status_text.text));
            RecordStatusText(mav::StatusText{
                .severity = status_text.severity,
                .text = std::string(status_text.text, text_length),
            });
            return;
        }

        if (message.compid != config_.target_component) {
            return;
        }

        mav::MavlinkTelemetryDecodeResult decode_result;
        {
            std::lock_guard<std::mutex> lock(telemetry_mutex_);
            decode_result = telemetry_decoder_.Decode(message, &telemetry_cache_, &state_cache_,
                                                      config_.autopilot_profile);
        }
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            RecordArmState(state_cache_.Snapshot().armed);
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
        frame.source_unix_time_ms = cache.source_unix_time_ms;
        frame.source_time_boot_ms = cache.source_time_boot_ms;
        frame.lat_deg = cache.lat_deg;
        frame.lon_deg = cache.lon_deg;
        frame.rel_alt_m = cache.rel_alt_m;
        frame.abs_alt_m = cache.abs_alt_m;
        frame.vx_mps = cache.vx_mps;
        frame.vy_mps = cache.vy_mps;
        frame.vz_mps = cache.vz_mps;
        frame.roll_deg = cache.roll_deg;
        frame.pitch_deg = cache.pitch_deg;
        frame.yaw_deg = cache.yaw_deg;
        frame.battery_percent = cache.battery_percent;
        frame.armed = cache.armed;
        frame.landed = cache.landed;
        frame.failsafe = cache.failsafe;
        frame.ekf_ok = cache.ekf_ok;
        frame.gps_fix_type = cache.gps_fix_type;
        frame.satellites_visible = cache.satellites_visible;
        frame.gps_hdop = cache.gps_hdop;
        frame.link_quality_percent = cache.link_quality_percent;
        frame.position_frame = cache.position_frame;
        frame.velocity_frame = cache.velocity_frame;
        frame.validity = cache.validity;
        frame.accuracy = cache.accuracy;
        frame.home_origin = cache.home_origin;
        frame.gps_quality = cache.gps_quality;
        frame.estimator_state = cache.estimator_state;
        frame.estimator_flags = cache.estimator_flags;
        frame.estimator_position_ok = cache.estimator_position_ok;
        frame.estimator_velocity_ok = cache.estimator_velocity_ok;
        frame.estimator_attitude_ok = cache.estimator_attitude_ok;
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
        request_interval(MAVLINK_MSG_ID_GPS_RAW_INT);
        request_interval(MAVLINK_MSG_ID_ATTITUDE);
        request_interval(MAVLINK_MSG_ID_HOME_POSITION);
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
                    result = SendArmDisarmCommand(/*arm=*/true, /*force=*/false);
                },
                [&](const CmdForceArm&) {
                    if (context.priority != CommandPriority::kEmergency) {
                        result = core::Result::Rejected("force-arm requires emergency priority");
                        return;
                    }
                    result = SendArmDisarmCommand(/*arm=*/true, /*force=*/true);
                },
                [&](const CmdDisarm&) {
                    result = SendArmDisarmCommand(/*arm=*/false, /*force=*/false);
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
                    result = SendArmDisarmCommand(/*arm=*/false, /*force=*/true);
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
        if (const core::Result result =
                mav::MavlinkCommandExecutor::ResolveCustomMode(config_, mode, &custom_mode);
            !result.IsOk()) {
            return result;
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
        return SendUnverifiedMavlinkMessage(message);
    }

    [[nodiscard]] core::Result SendSetHome(const CmdSetHome& home) {
        return SendCommandLong(MAV_CMD_DO_SET_HOME, home.use_current ? 1.0F : 0.0F, 0.0F, 0.0F,
                               0.0F, static_cast<float>(home.lat_deg),
                               static_cast<float>(home.lon_deg), static_cast<float>(home.alt_m));
    }

    [[nodiscard]] core::Result SendArmDisarmCommand(bool arm, bool force) {
        const std::uint64_t arm_state_start_sequence = ArmStateSequence();
        const mav::MavlinkCommandLongSpec spec =
            mav::MavlinkCommandExecutor::ArmDisarmCommand(arm, force);
        mav::MavlinkCommandAckResult ack_result = SendCommandLongDetailed(spec);
        if (!ack_result.ack_timed_out) {
            return ack_result.ToCoreResult();
        }

        const ArmStateObservation observation =
            WaitForArmStateObservation(arm, arm_state_start_sequence, kArmStateConfirmationGrace);
        const std::string desired_state = arm ? "armed" : "disarmed";
        if (observation.current_desired) {
            return core::Result::Success(
                "COMMAND_ACK timed out for arm/disarm command, but "
                "vehicle is " +
                desired_state + " by heartbeat");
        }
        if (observation.observed_desired_after_start) {
            return core::Result::Failed(
                "COMMAND_ACK timed out for arm/disarm command; vehicle "
                "became " +
                desired_state + " but is no longer " + desired_state);
        }
        return ack_result.ToCoreResult();
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

    [[nodiscard]] core::Result SendCommandLong(const mav::MavlinkCommandLongSpec& spec,
                                               bool wait_for_ack = true) {
        return SendCommandLong(spec.command, spec.params[0], spec.params[1], spec.params[2],
                               spec.params[3], spec.params[4], spec.params[5], spec.params[6],
                               wait_for_ack);
    }

    [[nodiscard]] mav::MavlinkCommandAckResult SendCommandLongDetailed(
        const mav::MavlinkCommandLongSpec& spec, bool wait_for_ack = true) {
        return SendCommandLongDetailed(spec.command, spec.params[0], spec.params[1], spec.params[2],
                                       spec.params[3], spec.params[4], spec.params[5],
                                       spec.params[6], wait_for_ack);
    }

    [[nodiscard]] mav::MavlinkCommandAckResult SendCommandLongDetailed(
        std::uint16_t command, float param1 = 0.0F, float param2 = 0.0F, float param3 = 0.0F,
        float param4 = 0.0F, float param5 = 0.0F, float param6 = 0.0F, float param7 = 0.0F,
        bool wait_for_ack = true) {
        std::uint64_t ack_start_sequence{};
        std::uint64_t status_start_sequence{};
        if (wait_for_ack) {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            ack_start_sequence = ack_sequence_;
        }
        if (wait_for_ack) {
            std::lock_guard<std::mutex> lock(status_text_mutex_);
            status_start_sequence = status_text_sequence_;
        }

        mavlink_message_t message{};
        core::Logger::DebugFmt(
            "MavlinkBackend: sending COMMAND_LONG command={} params=[{:.3f},{:.3f},{:.3f},{:.3f},"
            "{:.3f},{:.3f},{:.3f}] target={}/{}",
            command, param1, param2, param3, param4, param5, param6, param7, config_.target_system,
            config_.target_component);
        mavlink_msg_command_long_pack(config_.source_system, config_.source_component, &message,
                                      config_.target_system, config_.target_component, command, 0,
                                      param1, param2, param3, param4, param5, param6, param7);
        core::Result send_result = SendMavlinkMessage(message);
        if (!send_result.IsOk() || !wait_for_ack) {
            mav::MavlinkCommandAckResult result;
            result.send_result = std::move(send_result);
            return result;
        }
        return WaitForCommandAck(command, ack_start_sequence, status_start_sequence);
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
            static_cast<float>(alt_m), 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, yaw_deg.value_or(0.0F),
            0.0F);
        return SendUnverifiedMavlinkMessage(message);
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

    [[nodiscard]] core::Result ExecuteBackendCommand(const BackendCmd& backend) {
        core::Result result = core::Result::Rejected("backend command not handled");
        std::visit(core::Overloaded{[&](const CmdBackendCommand& command) {
                       if (command.backend_namespace != "mavlink") {
                           result = core::Result::Rejected(
                               "MAVLink backend command namespace must be "
                               "'mavlink'");
                           return;
                       }
                       if (command.name != "command-long") {
                           result = core::Result::Rejected("unsupported MAVLink backend command '" +
                                                           command.name + "'");
                           return;
                       }
                       const auto mav_command = CommandParam(command.params);
                       if (!mav_command.has_value()) {
                           result = core::Result::Rejected(
                               "mavlink command-long requires numeric param 'command'");
                           return;
                       }
                       result = SendCommandLong(
                           *mav_command, FloatParam(command.params, "param1").value_or(0.0F),
                           FloatParam(command.params, "param2").value_or(0.0F),
                           FloatParam(command.params, "param3").value_or(0.0F),
                           FloatParam(command.params, "param4").value_or(0.0F),
                           FloatParam(command.params, "param5").value_or(0.0F),
                           FloatParam(command.params, "param6").value_or(0.0F),
                           FloatParam(command.params, "param7").value_or(0.0F));
                   }},
                   backend);
        return result;
    }

    [[nodiscard]] core::Result SendMavlinkMessage(const mavlink_message_t& message) {
        std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> send_buffer{};
        const std::uint16_t length = mavlink_msg_to_send_buffer(send_buffer.data(), &message);

        return transport_.Send(send_buffer.data(), length);
    }

    [[nodiscard]] core::Result SendUnverifiedMavlinkMessage(const mavlink_message_t& message) {
        const core::Result result = SendMavlinkMessage(message);
        if (!result.IsOk()) {
            return result;
        }
        return core::Result::Success("MAVLink setpoint sent; setpoint stream is not acknowledged");
    }

    void RecordCommandAck(const mav::CommandAck& ack) {
        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            const std::uint64_t sequence = ++ack_sequence_;
            ack_history_.push_back(mav::SequencedCommandAck{
                .sequence = sequence,
                .ack = ack,
            });
            while (ack_history_.size() > kMaxCommandAckHistory) {
                ack_history_.pop_front();
            }
        }
        ack_cv_.notify_all();
        core::Logger::DebugFmt(
            "MavlinkBackend: COMMAND_ACK command={} result={} param2={} target={}/{}", ack.command,
            mav::MavResultName(ack.result), ack.result_param2, static_cast<int>(ack.target_system),
            static_cast<int>(ack.target_component));
    }

    void RecordStatusText(const mav::StatusText& status_text) {
        if (status_text.text.empty()) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(status_text_mutex_);
            last_status_text_ = status_text;
            ++status_text_sequence_;
        }
        status_text_cv_.notify_all();
        core::Logger::DebugFmt("MavlinkBackend: STATUSTEXT severity={} text={}",
                               static_cast<int>(status_text.severity), status_text.text);
    }

    [[nodiscard]] mav::MavlinkCommandAckResult WaitForCommandAck(
        std::uint16_t command, std::uint64_t start_sequence, std::uint64_t status_start_sequence) {
        std::unique_lock<std::mutex> lock(ack_mutex_);
        std::optional<mav::CommandAck> matched_ack =
            FindCommandAckAfterLocked(command, start_sequence);
        if (!matched_ack.has_value()) {
            static_cast<void>(ack_cv_.wait_for(
                lock, std::chrono::milliseconds{config_.command_ack_timeout_ms}, [&] {
                    matched_ack = FindCommandAckAfterLocked(command, start_sequence);
                    return matched_ack.has_value();
                }));
        }

        if (!matched_ack.has_value()) {
            mav::MavlinkCommandAckResult result;
            result.ack_timed_out = true;
            result.send_result = core::Result::Failed("timed out waiting for COMMAND_ACK command=" +
                                                      std::to_string(command));
            return result;
        }

        const mav::CommandAck ack = *matched_ack;
        lock.unlock();

        mav::MavlinkCommandAckResult result;
        result.ack = ack;
        result.has_ack = true;
        if (ack.result != MAV_RESULT_ACCEPTED) {
            if (auto status_text = WaitForStatusText(status_start_sequence);
                status_text.has_value()) {
                result.status_text = *status_text;
                result.has_status_text = true;
            }
        }
        return result;
    }

    [[nodiscard]] std::optional<mav::CommandAck> FindCommandAckAfterLocked(
        std::uint16_t command, std::uint64_t start_sequence) const {
        return mav::FindCommandAckAfter(ack_history_, command, start_sequence,
                                        config_.source_system, config_.source_component);
    }

    [[nodiscard]] std::optional<mav::StatusText> WaitForStatusText(std::uint64_t start_sequence) {
        std::unique_lock<std::mutex> lock(status_text_mutex_);
        const bool got_status = status_text_cv_.wait_for(lock, std::chrono::milliseconds{500}, [&] {
            return status_text_sequence_ != start_sequence && last_status_text_.has_value();
        });
        if (!got_status) {
            return std::nullopt;
        }
        return last_status_text_;
    }

    [[nodiscard]] std::uint64_t ArmStateSequence() const {
        std::lock_guard<std::mutex> lock(arm_state_mutex_);
        return arm_state_sequence_;
    }

    void RecordArmState(bool armed) {
        {
            std::lock_guard<std::mutex> lock(arm_state_mutex_);
            if (!arm_state_history_.empty() && arm_state_history_.back().armed == armed) {
                return;
            }
            const std::uint64_t sequence = ++arm_state_sequence_;
            arm_state_history_.push_back(SequencedArmState{
                .sequence = sequence,
                .armed = armed,
            });
            while (arm_state_history_.size() > kMaxArmStateHistory) {
                arm_state_history_.pop_front();
            }
        }
        arm_state_cv_.notify_all();
    }

    [[nodiscard]] ArmStateObservation WaitForArmStateObservation(
        bool desired_armed, std::uint64_t start_sequence, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(arm_state_mutex_);
        auto observation = ArmStateObservationLocked(desired_armed, start_sequence);
        if (!observation.observed_desired_after_start) {
            static_cast<void>(arm_state_cv_.wait_for(lock, timeout, [&] {
                observation = ArmStateObservationLocked(desired_armed, start_sequence);
                return observation.observed_desired_after_start;
            }));
            observation = ArmStateObservationLocked(desired_armed, start_sequence);
        }
        lock.unlock();
        observation.current_desired = state_cache_.Snapshot().armed == desired_armed;
        return observation;
    }

    [[nodiscard]] ArmStateObservation ArmStateObservationLocked(
        bool desired_armed, std::uint64_t start_sequence) const {
        ArmStateObservation observation;
        for (const SequencedArmState& state : arm_state_history_) {
            if (state.sequence <= start_sequence) {
                continue;
            }
            if (state.armed == desired_armed) {
                observation.observed_desired_after_start = true;
            }
        }
        if (!arm_state_history_.empty()) {
            observation.current_desired = arm_state_history_.back().armed == desired_armed;
        }
        return observation;
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
    std::deque<mav::SequencedCommandAck> ack_history_;
    std::uint64_t ack_sequence_{0};
    std::mutex status_text_mutex_;
    std::condition_variable status_text_cv_;
    std::optional<mav::StatusText> last_status_text_;
    std::uint64_t status_text_sequence_{0};
    mutable std::mutex arm_state_mutex_;
    std::condition_variable arm_state_cv_;
    std::deque<SequencedArmState> arm_state_history_;
    std::uint64_t arm_state_sequence_{0};
};

}  // namespace

std::string_view ToString(MavlinkAutopilotProfile profile) noexcept {
    switch (profile) {
        case MavlinkAutopilotProfile::kArdupilotCopter:
            return "ardupilot-copter";
        case MavlinkAutopilotProfile::kArdupilotPlane:
            return "ardupilot-plane";
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
    return std::unexpected(core::Result::Rejected("unsupported mavlink.autopilot_profile '" +
                                                  std::string(value) +
                                                  "'; expected ardupilot-copter|ardupilot-plane"));
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
