// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

/// @file test_client.cpp
/// @brief Connects three independent clients (one per simulated drone) at the
///        lowest command priority (kOperator = 0) and runs a continuous command
///        cycle: ARM -> TAKEOFF(20m) -> WAYPOINT -> LAND -> (repeat).
///
/// If a command is rejected (because the CLI at kSupervisor=10 or the test
/// server at kOverride=20 currently holds authority), the client logs the
/// rejection and retries after 3 seconds without advancing the cycle.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "swarmkit/client/client.h"
#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"

namespace sc = swarmkit::client;
namespace cmd = swarmkit::commands;

namespace {

constexpr int kDeadlineMs = 3000;
constexpr auto kRetryDelay = std::chrono::seconds{3};
constexpr auto kCommandDelay = std::chrono::seconds{2};
constexpr auto kPollInterval = std::chrono::milliseconds{100};
constexpr auto kStartupDelay = std::chrono::seconds{2};
constexpr std::string_view kDefaultCaCertPath = "testdata/certs/ca.pem";
constexpr std::string_view kDefaultClientCertPath = "testdata/certs/test-client.pem";
constexpr std::string_view kDefaultClientKeyPath = "testdata/certs/test-client.key";
constexpr std::string_view kDefaultServerName = "localhost";

std::atomic<bool> g_running{true};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
}

[[nodiscard]] std::string GetArg(int argc, char** argv, std::string_view key,
                                 std::string_view default_value) {
    for (int idx = 1; idx + 1 < argc; ++idx) {
        if (std::string_view{argv[idx]} == key) {
            return {argv[idx + 1]};
        }
    }
    return std::string{default_value};
}

[[nodiscard]] std::optional<swarmkit::core::LogLevel> TryParseLogLevel(std::string_view value) {
    if (value == "trace") {
        return swarmkit::core::LogLevel::kTrace;
    }
    if (value == "debug") {
        return swarmkit::core::LogLevel::kDebug;
    }
    if (value == "info") {
        return swarmkit::core::LogLevel::kInfo;
    }
    if (value == "warn") {
        return swarmkit::core::LogLevel::kWarn;
    }
    if (value == "error") {
        return swarmkit::core::LogLevel::kError;
    }
    if (value == "critical") {
        return swarmkit::core::LogLevel::kCritical;
    }
    if (value == "off") {
        return swarmkit::core::LogLevel::kOff;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<swarmkit::core::LogSinkType> TryParseLogSink(std::string_view value) {
    if (value == "stdout" || value == "console") {
        return swarmkit::core::LogSinkType::kStdout;
    }
    if (value == "file") {
        return swarmkit::core::LogSinkType::kRotatingFile;
    }
    return std::nullopt;
}

struct DroneSpec {
    std::string drone_id;
    std::string agent_addr;
};

[[nodiscard]] sc::ClientSecurityConfig BuildSecurityConfig(int argc, char** argv) {
    sc::ClientSecurityConfig security;
    security.root_ca_cert_path = GetArg(argc, argv, "--ca-cert", kDefaultCaCertPath);
    security.cert_chain_path = GetArg(argc, argv, "--client-cert", kDefaultClientCertPath);
    security.private_key_path = GetArg(argc, argv, "--client-key", kDefaultClientKeyPath);
    security.server_authority_override = GetArg(argc, argv, "--server-name", kDefaultServerName);
    return security;
}

const std::vector<DroneSpec> kDrones = {
    {.drone_id = "drone-1", .agent_addr = "127.0.0.1:50061"},
    {.drone_id = "drone-2", .agent_addr = "127.0.0.1:50062"},
    {.drone_id = "drone-3", .agent_addr = "127.0.0.1:50063"},
};

[[nodiscard]] sc::SwarmAddressPreference ParseAddressPreference(std::string_view value) {
    if (value == "local") {
        return sc::SwarmAddressPreference::kPreferLocal;
    }
    return sc::SwarmAddressPreference::kPrimary;
}

[[nodiscard]] std::vector<DroneSpec> LoadDroneSpecs(int argc, char** argv) {
    const std::string kSwarmConfigPath = GetArg(argc, argv, "--swarm-config", "");
    if (kSwarmConfigPath.empty()) {
        return kDrones;
    }

    const auto kSwarmConfig = sc::LoadSwarmConfigFromFile(kSwarmConfigPath);
    if (!kSwarmConfig.has_value()) {
        swarmkit::core::Logger::ErrorFmt("Failed to load swarm config '{}': {}", kSwarmConfigPath,
                                         kSwarmConfig.error().message);
        return {};
    }

    const auto kAddressPreference =
        ParseAddressPreference(GetArg(argc, argv, "--address-mode", "primary"));

    std::vector<DroneSpec> drone_specs;
    drone_specs.reserve(kSwarmConfig->drones.size());
    for (const auto& drone : kSwarmConfig->drones) {
        const bool kUseLocal = kAddressPreference == sc::SwarmAddressPreference::kPreferLocal &&
                               !drone.local_address.empty();
        drone_specs.push_back({.drone_id = drone.drone_id,
                               .agent_addr = kUseLocal ? drone.local_address : drone.address});
    }
    return drone_specs;
}

/// @brief Sleep for the given duration while checking g_running every kPollInterval.
/// @return false if g_running was cleared before the duration elapsed.
[[nodiscard]] bool SleepWhileRunning(std::chrono::milliseconds duration) {
    const auto kDeadline = std::chrono::steady_clock::now() + duration;
    while (g_running.load(std::memory_order_relaxed)) {
        const auto kNow = std::chrono::steady_clock::now();
        if (kNow >= kDeadline) {
            return true;
        }
        std::this_thread::sleep_for(
            std::min(kPollInterval,
                     std::chrono::duration_cast<std::chrono::milliseconds>(kDeadline - kNow)));
    }
    return false;
}

/// @brief Run an infinite command cycle for a single drone.
void RunCommandLoop(const DroneSpec& spec, const sc::ClientSecurityConfig& security) {
    if (!SleepWhileRunning(kStartupDelay)) {
        return;
    }

    sc::ClientConfig cfg;
    cfg.address = spec.agent_addr;
    cfg.client_id = "test-client";
    cfg.deadline_ms = kDeadlineMs;
    cfg.security = security;

    sc::Client client(cfg);

    struct Step {
        std::string label;
        cmd::Command command;
    };

    const std::vector<Step> kSteps = {
        {.label = "ARM", .command = cmd::FlightCmd{cmd::CmdArm{}}},
        {.label = "TAKEOFF(20m)", .command = cmd::FlightCmd{cmd::CmdTakeoff{20.0}}},
        {.label = "WAYPOINT",
         .command = cmd::NavCmd{cmd::CmdSetWaypoint{
             .lat_deg = 40.1811,
             .lon_deg = 44.5136,
             .alt_m = 30.0,
             .speed_mps = 5.0F,
         }}},
        {.label = "LAND", .command = cmd::FlightCmd{cmd::CmdLand{}}},
    };

    std::size_t step_idx = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        const Step& step = kSteps[step_idx % kSteps.size()];

        cmd::CommandEnvelope envelope;
        envelope.context.drone_id = spec.drone_id;
        envelope.context.client_id = "test-client";
        envelope.context.priority = cmd::CommandPriority::kOperator;
        envelope.command = step.command;

        const sc::CommandResult kResult = client.SendCommand(envelope);

        if (kResult.ok) {
            swarmkit::core::Logger::InfoFmt("[{} @ {}] test-client -> {} -- ACCEPTED",
                                            spec.drone_id, spec.agent_addr, step.label);
            ++step_idx;

            if (!SleepWhileRunning(
                    std::chrono::duration_cast<std::chrono::milliseconds>(kCommandDelay))) {
                break;
            }
        } else {
            swarmkit::core::Logger::InfoFmt(
                "[{} @ {}] test-client -> {} -- REJECTED: {} (retrying in 3s)", spec.drone_id,
                spec.agent_addr, step.label, kResult.message);

            if (!SleepWhileRunning(
                    std::chrono::duration_cast<std::chrono::milliseconds>(kRetryDelay))) {
                break;
            }
        }
    }

    swarmkit::core::Logger::InfoFmt("[{}] Client stopped.", spec.drone_id);
}

}  // namespace

int main(int argc, char** argv) {
    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kInfo;

    const std::string kLogSink = GetArg(argc, argv, "--log-sink", "");
    if (!kLogSink.empty()) {
        const auto kParsedSink = TryParseLogSink(kLogSink);
        if (!kParsedSink) {
            std::cerr << "Invalid --log-sink value: " << kLogSink << "\n";
            return EXIT_FAILURE;
        }
        log_cfg.sink_type = *kParsedSink;
    }

    const std::string kLogFile = GetArg(argc, argv, "--log-file", "");
    if (!kLogFile.empty()) {
        log_cfg.log_file_path = kLogFile;
        if (kLogSink.empty()) {
            log_cfg.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
        }
    }

    const auto kParsedLevel = TryParseLogLevel(GetArg(argc, argv, "--log-level", "info"));
    if (!kParsedLevel) {
        std::cerr << "Invalid --log-level value\n";
        return EXIT_FAILURE;
    }
    log_cfg.level = *kParsedLevel;

    if (log_cfg.sink_type == swarmkit::core::LogSinkType::kRotatingFile &&
        log_cfg.log_file_path.empty()) {
        std::cerr << "Invalid logger configuration: log file path is required for file sink\n";
        return EXIT_FAILURE;
    }

    swarmkit::core::Logger::Init(log_cfg);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::core::Logger::Info(
        "SwarmKit Test Client -- kOperator priority (0)\n"
        "Will be preempted by CLI (kSupervisor=10) and test-server (kOverride=20).\n"
        "Press Ctrl+C to stop.");

    const std::vector<DroneSpec> kDroneSpecs = LoadDroneSpecs(argc, argv);
    const sc::ClientSecurityConfig kSecurityConfig = BuildSecurityConfig(argc, argv);
    if (kDroneSpecs.empty()) {
        return EXIT_FAILURE;
    }

    std::vector<std::thread> threads;
    threads.reserve(kDroneSpecs.size());

    for (const auto& drone : kDroneSpecs) {
        threads.emplace_back(RunCommandLoop, drone, kSecurityConfig);
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    return EXIT_SUCCESS;
}
