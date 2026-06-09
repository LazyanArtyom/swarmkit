// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

namespace swarmkit::client {
class Client;
}

namespace swarmkit::apps::cli::internal {

int RunMessage(client::Client& client, int argc, char** argv);
int RunArtifact(client::Client& client, int argc, char** argv);

}  // namespace swarmkit::apps::cli::internal
