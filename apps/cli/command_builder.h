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

#include "swarmkit/commands.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::vector<std::string> FindCommandActions(int argc, char** argv);
[[nodiscard]] std::expected<commands::Command, std::string> BuildCommandFromActions(
    const std::vector<std::string>& actions, int argc, char** argv);
[[nodiscard]] std::expected<commands::Command, std::string> BuildCommandFromArgs(int argc,
                                                                                 char** argv);
[[nodiscard]] commands::CommandEnvelope MakeCommandEnvelope(
    std::string_view drone_id, commands::Command command,
    commands::CommandPriority priority = commands::CommandPriority::kSupervisor);

}  // namespace swarmkit::apps::cli::internal
