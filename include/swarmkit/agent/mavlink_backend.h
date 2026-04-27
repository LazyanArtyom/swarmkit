// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <string>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent {

/// Runtime configuration for the direct MAVLink UDP backend.
struct MavlinkBackendConfig {
    std::string drone_id{"drone-1"};
    std::string bind_addr{"0.0.0.0:14601"};
    std::uint8_t target_system{1};
    std::uint8_t target_component{1};
    std::uint8_t source_system{245};
    std::uint8_t source_component{191};
    int telemetry_rate_hz{5};
    int peer_discovery_timeout_ms{2000};
    int command_ack_timeout_ms{2000};
    bool set_guided_before_arm{true};
    bool set_guided_before_takeoff{true};
    int guided_mode{4};

    [[nodiscard]] core::Result Validate() const;
};

/// Create a backend that talks MAVLink UDP directly to SITL/autopilot traffic.
[[nodiscard]] DroneBackendPtr MakeMavlinkBackend(MavlinkBackendConfig config);

}  // namespace swarmkit::agent
