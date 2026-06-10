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

int RunSwarm(const client::ClientConfig& client_cfg, int argc, char** argv);

}  // namespace swarmkit::apps::cli::internal
