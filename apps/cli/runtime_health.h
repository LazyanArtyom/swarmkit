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

int RunPing(client::Client& client);
int RunPreflight(client::Client& client, std::string_view drone_id, int argc, char** argv);
int RunHealth(client::Client& client);
int RunStats(client::Client& client);
int RunCapabilities(client::Client& client);

}  // namespace swarmkit::apps::cli::internal
