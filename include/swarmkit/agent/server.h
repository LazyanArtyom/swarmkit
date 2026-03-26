// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent {

inline constexpr int kDefaultAuthorityTtlMs = 5000;
inline constexpr int kDefaultTelemetryRateHz = 5;
inline constexpr int kMinimumTelemetryRateHz = 1;

/// ---------------------------------------------------------------------------
/// AgentConfig -- startup parameters for the gRPC agent server.
/// ---------------------------------------------------------------------------
struct AgentConfig {
    std::string agent_id{"agent-1"};         ///< Unique identifier for this agent.
    std::string bind_addr{"0.0.0.0:50061"};  ///< gRPC listen address.
    int default_authority_ttl_ms{kDefaultAuthorityTtlMs};
    int default_telemetry_rate_hz{kDefaultTelemetryRateHz};
    int min_telemetry_rate_hz{kMinimumTelemetryRateHz};

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
