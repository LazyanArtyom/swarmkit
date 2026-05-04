// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <string>
#include <string_view>
#include <vector>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"
namespace swarmkit::agent {

inline constexpr int kDefaultAuthorityTtlMs = 5000;
inline constexpr int kDefaultTelemetryRateHz = 5;
inline constexpr int kMinimumTelemetryRateHz = 1;
inline constexpr float kDefaultCruiseSpeedMps = 4.0F;
inline constexpr float kDefaultClimbSpeedMps = 1.5F;
inline constexpr float kDefaultDescentSpeedMps = 1.0F;
inline constexpr int kDefaultGoalMarginMs = 15000;
inline constexpr int kDefaultMaxGoalTimeoutMs = 300000;

struct VehicleProfile {
    std::string profile_id{"generic-quad"};
    float cruise_speed_mps{kDefaultCruiseSpeedMps};
    float climb_speed_mps{kDefaultClimbSpeedMps};
    float descent_speed_mps{kDefaultDescentSpeedMps};
    int goal_margin_ms{kDefaultGoalMarginMs};
    int max_goal_timeout_ms{kDefaultMaxGoalTimeoutMs};

    [[nodiscard]] core::Result Validate() const;
};

struct AgentSecurityConfig {
    std::string root_ca_cert_path;
    std::string cert_chain_path;
    std::string private_key_path;
    std::vector<std::string> allowed_client_ids;

    [[nodiscard]] core::Result Validate() const;
};

/// ---------------------------------------------------------------------------
/// AgentConfig -- startup parameters for the gRPC agent server.
/// ---------------------------------------------------------------------------
struct AgentConfig {
    std::string agent_id{"agent-1"};         ///< Unique identifier for this agent.
    std::string bind_addr{"0.0.0.0:50061"};  ///< gRPC listen address.
    int default_authority_ttl_ms{kDefaultAuthorityTtlMs};
    int default_telemetry_rate_hz{kDefaultTelemetryRateHz};
    int min_telemetry_rate_hz{kMinimumTelemetryRateHz};
    VehicleProfile vehicle_profile{};
    AgentSecurityConfig security{};

    [[nodiscard]] core::Result Validate() const;
    void ApplyEnvironment(std::string_view prefix = "SWARMKIT_AGENT_");
};

/// @brief Load agent configuration from a YAML file.
///
/// Supports either a root-level agent map or an `agent:` section.
[[nodiscard]] std::expected<AgentConfig, core::Result> LoadAgentConfigFromFile(
    const std::string& path);

/// @brief Start the agent gRPC server.
///
/// Blocks until the server shuts down.  Returns 0 on clean exit, non-zero on
/// error (e.g. port already in use, null backend).
[[nodiscard]] int RunAgentServer(const AgentConfig& config, DroneBackendPtr backend);

}  // namespace swarmkit::agent
