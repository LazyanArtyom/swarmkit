// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/core/result.h"

extern "C" {
#include "ardupilotmega/mavlink.h"
}

namespace swarmkit::agent::mavlink {

inline constexpr int kMicrosecondsPerSecond = 1000000;
inline constexpr int kMillisecondsPerSecond = 1000;
inline constexpr int kUdpReceiveTimeoutMs = 200;
inline constexpr int kMaxUdpPort = 65535;
inline constexpr std::size_t kUdpReceiveBufferBytes = 2048;
inline constexpr double kDegE7 = 10000000.0;
inline constexpr float kMillimetresPerMetre = 1000.0F;
inline constexpr float kUnknownBattery = -1.0F;
inline constexpr std::uint16_t kMissionUploadMaxItems = 1000;
inline constexpr int kHeartbeatStaleTimeoutMs = 3000;
inline constexpr int kTelemetryStaleTimeoutMs = 5000;

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

struct MavlinkCommandAckResult {
    CommandAck ack;
    bool has_ack{false};
    core::Result send_result{core::Result::Success()};

    [[nodiscard]] bool IsUnsupported() const;
    [[nodiscard]] core::Result ToCoreResult() const;
};

struct MissionRequest {
    std::uint16_t seq{};
    std::uint8_t mission_type{MAV_MISSION_TYPE_MISSION};
};

struct MissionAck {
    std::uint8_t type{};
    std::uint8_t mission_type{MAV_MISSION_TYPE_MISSION};
};

struct MavlinkVehicleState {
    std::int64_t last_heartbeat_unix_ms{};
    std::int64_t last_telemetry_unix_ms{};
    std::uint8_t system_id{};
    std::uint8_t component_id{};
    std::uint8_t mav_type{};
    std::uint8_t autopilot{};
    std::uint8_t base_mode{};
    std::uint8_t system_status{};
    bool armed{false};
    int custom_mode{-1};
    bool failsafe{false};
};

struct Px4Mode {
    int main_mode{};
    int sub_mode{};
};

[[nodiscard]] std::int64_t NowUnixMs();
[[nodiscard]] std::string ToLower(std::string value);
[[nodiscard]] std::optional<std::pair<std::string, std::uint16_t>> SplitHostPort(
    const std::string& value);
[[nodiscard]] std::string ModeString(const mavlink_heartbeat_t& heartbeat);
[[nodiscard]] std::string MavResultName(std::uint8_t result);
[[nodiscard]] std::string MissionResultName(std::uint8_t result);
[[nodiscard]] std::optional<int> ArduCopterModeFromName(std::string mode);
[[nodiscard]] std::optional<int> ArduPlaneModeFromName(std::string mode);
[[nodiscard]] std::optional<Px4Mode> Px4ModeFromName(std::string mode);
[[nodiscard]] std::vector<std::string> SupportedModes(MavlinkAutopilotProfile profile);

}  // namespace swarmkit::agent::mavlink
