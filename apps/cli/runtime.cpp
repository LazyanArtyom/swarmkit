// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <expected>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "command_builder.h"
#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"
#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/telemetry.h"
#include "usage.h"

namespace swarmkit::apps::cli::internal {
namespace {

using swarmkit::client::Client;
using swarmkit::client::ClientConfig;
using swarmkit::commands::CmdLand;
using swarmkit::commands::CmdReturnHome;
using swarmkit::commands::Command;
using swarmkit::commands::CommandPriority;
using swarmkit::commands::FlightCmd;
using swarmkit::commands::NavCmd;

[[nodiscard]] volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

extern "C" void OnSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
}

[[nodiscard]] bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

int RunCommand(Client& client, std::string_view drone_id, CommandPriority priority, int argc,
               char** argv) {
    const auto kCommand = BuildCommandFromArgs(argc, argv);
    if (!kCommand.has_value()) {
        std::cerr << kCommand.error() << "\n";
        if (kCommand.error().starts_with("Unknown action:")) {
            std::cerr << "\n";
            PrintUsage();
        }
        return EXIT_FAILURE;
    }

    const auto kResult = client.SendCommand(MakeCommandEnvelope(drone_id, *kCommand, priority));
    if (!kResult.ok) {
        std::cerr << "Command FAILED: " << kResult.message;
        if (!kResult.correlation_id.empty()) {
            std::cerr << " [corr=" << kResult.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Command OK" << (kResult.message.empty() ? "" : ": " + kResult.message) << "\n";
    return EXIT_SUCCESS;
}

int RunPing(Client& client) {
    const auto kResult = client.Ping();

    if (!kResult.ok) {
        std::cerr << "Ping FAILED: " << kResult.error_message;
        if (!kResult.correlation_id.empty()) {
            std::cerr << " [corr=" << kResult.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id  : " << kResult.agent_id << "\n"
              << "  version   : " << kResult.version << "\n"
              << "  time_ms   : " << kResult.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}

int RunHealth(Client& client) {
    const auto kStatus = client.GetHealth();
    if (!kStatus.ok) {
        std::cerr << "Health FAILED: " << kStatus.message;
        if (!kStatus.correlation_id.empty()) {
            std::cerr << " [corr=" << kStatus.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Health OK\n"
              << "  ready     : " << (kStatus.ready ? "true" : "false") << "\n"
              << "  agent_id  : " << kStatus.agent_id << "\n"
              << "  version   : " << kStatus.version << "\n"
              << "  time_ms   : " << kStatus.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}

int RunStats(Client& client) {
    const auto kStats = client.GetRuntimeStats();
    if (!kStats.ok) {
        std::cerr << "Stats FAILED: " << kStats.error.user_message;
        if (!kStats.correlation_id.empty()) {
            std::cerr << " [corr=" << kStats.correlation_id << "]";
        }
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Runtime Stats\n"
              << "  agent_id                    : " << kStats.agent_id << "\n"
              << "  ready                       : " << (kStats.ready ? "true" : "false") << "\n"
              << "  ping_requests_total         : " << kStats.ping_requests_total << "\n"
              << "  health_requests_total       : " << kStats.health_requests_total << "\n"
              << "  stats_requests_total        : " << kStats.runtime_stats_requests_total << "\n"
              << "  command_requests_total      : " << kStats.command_requests_total << "\n"
              << "  command_rejected_total      : " << kStats.command_rejected_total << "\n"
              << "  command_failed_total        : " << kStats.command_failed_total << "\n"
              << "  lock_requests_total         : " << kStats.lock_requests_total << "\n"
              << "  watch_requests_total        : " << kStats.watch_requests_total << "\n"
              << "  current_authority_watchers  : " << kStats.current_authority_watchers << "\n"
              << "  total_telemetry_subs        : " << kStats.total_telemetry_subscriptions << "\n"
              << "  current_telemetry_streams   : " << kStats.current_telemetry_streams << "\n"
              << "  telemetry_frames_sent_total : " << kStats.telemetry_frames_sent_total << "\n"
              << "  backend_failures_total      : " << kStats.backend_failures_total << "\n";
    return EXIT_SUCCESS;
}

[[nodiscard]] std::expected<int, std::string> ParseTelemetryRate(int argc, char** argv) {
    try {
        return std::stoi(common::GetOptionValue(argc, argv, "--rate", kDefaultTelemetryRate));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid --rate value '" +
                               common::GetOptionValue(argc, argv, "--rate", kDefaultTelemetryRate) +
                               "': " + exc.what());
    }
}

int RunTelemetry(Client& client, std::string_view drone_id, int rate_hz) {
    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::cout << "Subscribing to telemetry: drone=" << drone_id << " rate=" << rate_hz << " Hz\n"
              << "Press Ctrl+C to stop.\n\n";

    swarmkit::client::TelemetrySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.rate_hertz = rate_hz;

    client.SubscribeTelemetry(
        subscription,
        [](const swarmkit::core::TelemetryFrame& frame) {
            std::cout << std::fixed << std::setprecision(kTelemetryCoordPrecision) << "["
                      << frame.unix_time_ms << "]"
                      << " drone=" << frame.drone_id << " lat=" << frame.lat_deg
                      << " lon=" << frame.lon_deg << std::setprecision(kTelemetryValuePrecision)
                      << " alt=" << frame.rel_alt_m << "m"
                      << " bat=" << frame.battery_percent << "%"
                      << " mode=" << frame.mode << "\n";
        },
        [](const std::string& error_msg) {
            std::cerr << "Telemetry stream error: " << error_msg << "\n";
        });

    while (!IsStopRequested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryPollIntervalMs));
    }

    client.StopTelemetry();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

[[nodiscard]] std::expected<std::int64_t, std::string> ParseTtlMs(int argc, char** argv) {
    const auto ttl =
        ParseIntArg(common::GetOptionValue(argc, argv, "--ttl-ms", kDefaultZero), "--ttl-ms");
    if (!ttl.has_value()) {
        return std::unexpected(ttl.error());
    }
    return static_cast<std::int64_t>(*ttl);
}

int RunLock(Client& client, std::string_view drone_id, int argc, char** argv) {
    const auto ttl_ms = ParseTtlMs(argc, argv);
    if (!ttl_ms.has_value()) {
        std::cerr << ttl_ms.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto result = client.LockAuthority(std::string(drone_id), *ttl_ms);
    if (!result.ok) {
        std::cerr << "Lock FAILED: " << result.message << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "Lock OK" << (result.message.empty() ? "" : ": " + result.message) << "\n";
    return EXIT_SUCCESS;
}

int RunUnlock(Client& client, std::string_view drone_id) {
    client.ReleaseAuthority(std::string(drone_id));
    std::cout << "Unlock OK\n";
    return EXIT_SUCCESS;
}

int RunWatchAuthority(Client& client, std::string_view drone_id, CommandPriority priority) {
    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::client::AuthoritySubscription subscription;
    subscription.drone_id = std::string(drone_id);
    subscription.priority = priority;

    client.WatchAuthority(
        subscription,
        [](const swarmkit::client::AuthorityEventInfo& event) {
            std::cout << "authority event drone=" << event.drone_id
                      << " holder=" << event.holder_client_id
                      << " priority=" << static_cast<int>(event.holder_priority)
                      << " kind=" << static_cast<int>(event.kind) << "\n";
        },
        [](const std::string& error_msg) {
            std::cerr << "Authority stream error: " << error_msg << "\n";
        });

    while (!IsStopRequested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryPollIntervalMs));
    }
    client.StopAuthorityWatch();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

[[nodiscard]] std::expected<std::unique_ptr<swarmkit::client::SwarmClient>, int> BuildSwarmClient(
    const ClientConfig& client_cfg, int argc, char** argv) {
    const std::string config_path = common::GetOptionValue(argc, argv, "--swarm-config");
    if (config_path.empty()) {
        std::cerr << "swarm requires --swarm-config PATH\n";
        return std::unexpected(EXIT_FAILURE);
    }
    auto loaded = swarmkit::client::LoadSwarmConfigFromFile(config_path);
    if (!loaded.has_value()) {
        std::cerr << "Failed to load swarm config '" << config_path
                  << "': " << loaded.error().message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    loaded->default_client_config = client_cfg;

    auto swarm = std::make_unique<swarmkit::client::SwarmClient>(client_cfg);
    const std::string address_mode =
        common::GetOptionValue(argc, argv, "--address-mode", kDefaultAddressMode);
    const auto preference = address_mode == "local"
                                ? swarmkit::client::SwarmAddressPreference::kPreferLocal
                                : swarmkit::client::SwarmAddressPreference::kPrimary;
    if (const auto result = swarm->ApplyConfig(*loaded, preference); !result.IsOk()) {
        std::cerr << "Invalid swarm config: " << result.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    return swarm;
}

void PrintSwarmResults(
    const std::unordered_map<std::string, swarmkit::client::CommandResult>& results) {
    for (const auto& [drone_id, result] : results) {
        std::cout << drone_id << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        std::cout << "\n";
    }
}

[[nodiscard]] std::vector<std::string> FindSwarmActions(int argc, char** argv) {
    int swarm_index = -1;
    for (int index = 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        if (IsOptionWithValue(arg)) {
            ++index;
            continue;
        }
        if (arg == "swarm") {
            swarm_index = index;
            break;
        }
    }
    if (swarm_index < 0) {
        return {};
    }
    std::vector<std::string> actions;
    for (int index = swarm_index + 1; index < argc; ++index) {
        const std::string_view arg = argv[index];
        if (IsOptionWithValue(arg)) {
            ++index;
            continue;
        }
        if (!arg.starts_with("-")) {
            actions.emplace_back(arg);
        }
    }
    return actions;
}

int RunSwarm(const ClientConfig& client_cfg, int argc, char** argv) {
    auto swarm = BuildSwarmClient(client_cfg, argc, argv);
    if (!swarm.has_value()) {
        return swarm.error();
    }
    const auto actions = FindSwarmActions(argc, argv);
    if (actions.empty()) {
        std::cerr << "swarm requires lock-all, unlock-all, broadcast, land-all, or rtl-all\n";
        return EXIT_FAILURE;
    }

    Command command;
    if (actions[0] == "lock-all") {
        const auto ttl_ms = ParseTtlMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        PrintSwarmResults((*swarm)->LockAll(*ttl_ms));
        return EXIT_SUCCESS;
    }
    if (actions[0] == "unlock-all") {
        (*swarm)->UnlockAll();
        std::cout << "Unlock all OK\n";
        return EXIT_SUCCESS;
    }
    if (actions[0] == "land-all") {
        command = FlightCmd{CmdLand{}};
    } else if (actions[0] == "rtl-all") {
        command = NavCmd{CmdReturnHome{}};
    } else if (actions[0] == "broadcast") {
        std::vector<std::string> command_actions;
        if (actions.size() > 1) {
            command_actions.assign(actions.begin() + 1, actions.end());
        }
        const auto built = BuildCommandFromActions(command_actions, argc, argv);
        if (!built.has_value()) {
            std::cerr << built.error() << "\n";
            return EXIT_FAILURE;
        }
        command = *built;
    } else {
        std::cerr << "Unknown swarm action: " << actions[0] << "\n";
        return EXIT_FAILURE;
    }

    swarmkit::commands::CommandContext context;
    context.client_id = std::string(kCliClientId);
    context.priority = client_cfg.priority;
    PrintSwarmResults((*swarm)->BroadcastCommand(command, context));
    return EXIT_SUCCESS;
}

}  // namespace

[[nodiscard]] int DispatchCommand(const CliInvocation& invocation, const ClientConfig& client_cfg,
                                  Client& client, int argc, char** argv) {
    if (invocation.command == "ping") {
        return RunPing(client);
    }
    if (invocation.command == "health") {
        return RunHealth(client);
    }
    if (invocation.command == "stats") {
        return RunStats(client);
    }
    if (invocation.command == "telemetry") {
        const auto kRateHz = ParseTelemetryRate(argc, argv);
        if (!kRateHz.has_value()) {
            std::cerr << kRateHz.error() << "\n";
            return EXIT_FAILURE;
        }
        return RunTelemetry(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                            *kRateHz);
    }
    if (invocation.command == "command") {
        return RunCommand(client, common::GetOptionValue(argc, argv, "--drone", kDefaultDroneId),
                          client_cfg.priority, argc, argv);
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
