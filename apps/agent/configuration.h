// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <string>

#include "swarmkit/agent/mavlink_backend.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::apps::agent::internal {

struct BackendSelection {
    std::string backend;
    swarmkit::agent::MavlinkBackendConfig mavlink;
};

[[nodiscard]] std::expected<core::LoggerConfig, int> BuildAgentLoggerConfig(int argc, char** argv);
[[nodiscard]] std::expected<swarmkit::agent::AgentConfig, int> BuildAgentConfig(int argc,
                                                                                char** argv);
[[nodiscard]] std::expected<BackendSelection, int> BuildBackendSelection(int argc, char** argv);

}  // namespace swarmkit::apps::agent::internal
