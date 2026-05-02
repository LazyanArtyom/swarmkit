// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"

namespace swarmkit::apps::cli::internal {

struct CliInvocation {
    std::string address;
    std::string command;
    bool has_explicit_address{false};
};

[[nodiscard]] bool IsSubcommand(std::string_view value);
[[nodiscard]] bool IsOptionWithValue(std::string_view value);

[[nodiscard]] std::expected<float, std::string> ParseFloatArg(std::string_view value,
                                                              std::string_view key);
[[nodiscard]] std::expected<double, std::string> ParseDoubleArg(std::string_view value,
                                                                std::string_view key);
[[nodiscard]] std::expected<int, std::string> ParseIntArg(std::string_view value,
                                                          std::string_view key);
[[nodiscard]] std::expected<commands::CommandPriority, std::string> ParseCliPriority(int argc,
                                                                                     char** argv);
[[nodiscard]] std::expected<CliInvocation, int> ParseInvocation(int argc, char** argv);
[[nodiscard]] std::expected<core::LoggerConfig, int> BuildCliLoggerConfig(int argc, char** argv);
[[nodiscard]] std::expected<client::ClientConfig, int> BuildCliClientConfig(
    const CliInvocation& invocation, int argc, char** argv);

}  // namespace swarmkit::apps::cli::internal
