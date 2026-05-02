// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include "options.h"
#include "swarmkit/client/client.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] int DispatchCommand(const CliInvocation& invocation,
                                  const client::ClientConfig& client_cfg, client::Client& client,
                                  int argc, char** argv);

}  // namespace swarmkit::apps::cli::internal
