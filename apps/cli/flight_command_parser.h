// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <string>
#include <vector>

#include "swarmkit/commands.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::expected<commands::Command, std::string> BuildFlightCommand(
    const std::vector<std::string>& actions, int argc, char** argv);

}  // namespace swarmkit::apps::cli::internal
