// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "swarmkit/commands.h"

namespace swarmkit::client {
class Client;
struct ClientConfig;
}  // namespace swarmkit::client

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::expected<int, std::string> ParseTelemetryRate(int argc, char** argv);
int RunTelemetry(client::Client& client, std::string_view drone_id, int rate_hz, int argc, char** argv);

}  // namespace swarmkit::apps::cli::internal
