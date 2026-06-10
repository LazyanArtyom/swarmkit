// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime.h"

#include <cstdlib>
#include <iostream>

#include "common/arg_utils.h"
#include "constants.h"
#include "data_runtime.h"
#include "runtime_authority.h"
#include "runtime_command.h"
#include "runtime_goal.h"
#include "runtime_health.h"
#include "runtime_reports.h"
#include "runtime_swarm.h"
#include "runtime_telemetry.h"
#include "swarmkit/client/client.h"
#include "usage.h"

namespace swarmkit::apps::cli::internal {

using swarmkit::client::Client;
using swarmkit::client::ClientConfig;

[[nodiscard]] int DispatchCommand(const CliInvocation& invocation, const ClientConfig& client_cfg,
                                  Client& client, int argc, char** argv) {
    if (invocation.command == "ping") {
        return RunPing(client);
    }
    if (invocation.command == "health") {
        return RunHealth(client);
    }
    if (invocation.command == "preflight") {
        return RunPreflight(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                            argc, argv);
    }
    if (invocation.command == "stats") {
        return RunStats(client);
    }
    if (invocation.command == "capabilities") {
        return RunCapabilities(client);
    }
    if (invocation.command == "telemetry") {
        const auto rate_hz = ParseTelemetryRate(argc, argv);
        if (!rate_hz.has_value()) {
            std::cerr << rate_hz.error() << "\n";
            return EXIT_FAILURE;
        }
        return RunTelemetry(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                            *rate_hz, argc, argv);
    }
    if (invocation.command == "command") {
        return RunCommand(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                          client_cfg.client_id, client_cfg.priority, argc, argv);
    }
    if (invocation.command == "sequence") {
        return RunSequence(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                           client_cfg.client_id, client_cfg.priority, argc, argv);
    }
    if (invocation.command == "goal") {
        return RunGoal(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId), argc,
                       argv);
    }
    if (invocation.command == "reports") {
        return RunReports(client, common::GetOptionValue(argc, argv, "--drone", "all"), argc, argv);
    }
    if (invocation.command == "message") {
        return RunMessage(client, argc, argv);
    }
    if (invocation.command == "artifact") {
        return RunArtifact(client, argc, argv);
    }
    if (invocation.command == "lock") {
        return RunLock(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId), argc,
                       argv);
    }
    if (invocation.command == "unlock") {
        return RunUnlock(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId));
    }
    if (invocation.command == "watch-authority") {
        return RunWatchAuthority(client,
                                 common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                                 client_cfg.priority);
    }
    if (invocation.command == "swarm") {
        return RunSwarm(client_cfg, argc, argv);
    }

    std::cerr << "Unknown command: " << invocation.command << "\n\n";
    PrintUsage();
    return EXIT_FAILURE;
}

}  // namespace swarmkit::apps::cli::internal
