// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent {

enum class MavlinkAutopilotProfile : std::uint8_t {
    kArdupilotCopter,
    kArdupilotPlane,
};

/// @brief Convert a MAVLink autopilot profile to its YAML/CLI string.
[[nodiscard]] std::string_view ToString(MavlinkAutopilotProfile profile) noexcept;

/// @brief Parse a MAVLink autopilot profile string.
/// @param value Expected values include "ardupilot-copter" and "ardupilot-plane".
/// @return Parsed profile, or Rejected when the value is unknown.
[[nodiscard]] std::expected<MavlinkAutopilotProfile, core::Result> ParseMavlinkAutopilotProfile(
    std::string_view value);

/// Runtime configuration for the direct MAVLink UDP backend.
struct MavlinkBackendConfig {
    std::string drone_id{"drone-1"};         ///< SwarmKit drone id exposed to clients.
    std::string bind_addr{"0.0.0.0:14601"};  ///< UDP listen address in host:port form.
    MavlinkAutopilotProfile autopilot_profile{MavlinkAutopilotProfile::kArdupilotCopter};
    std::uint8_t target_system{1};         ///< MAVLink target system id.
    std::uint8_t target_component{1};      ///< MAVLink target component id.
    std::uint8_t source_system{245};       ///< MAVLink source system id used by SwarmKit.
    std::uint8_t source_component{191};    ///< MAVLink source component id used by SwarmKit.
    int telemetry_rate_hz{5};              ///< Requested telemetry stream rate in hertz.
    int peer_discovery_timeout_ms{2000};   ///< Time to wait for the first MAVLink heartbeat.
    int command_ack_timeout_ms{2000};      ///< Time to wait for COMMAND_ACK messages.
    bool set_guided_before_arm{false};     ///< Switch to guided_mode before arm commands.
    bool set_guided_before_takeoff{true};  ///< Switch to guided_mode before takeoff commands.
    int guided_mode{4};                    ///< Backend-specific guided mode id.
    bool allow_flight_termination{false};  ///< Permit flight-termination command mapping.

    /// @brief Validate ids, address, rates, and timeout fields.
    /// @return Success when the backend can be constructed.
    [[nodiscard]] core::Result Validate() const;
};

/// Create a backend that talks MAVLink UDP directly to SITL/autopilot traffic.
/// @param config MAVLink UDP, target, source, timeout, and safety settings.
/// @return Owning backend pointer.
[[nodiscard]] DroneBackendPtr MakeMavlinkBackend(MavlinkBackendConfig config);

}  // namespace swarmkit::agent
