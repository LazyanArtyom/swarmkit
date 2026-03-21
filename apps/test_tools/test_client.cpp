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
#include <string>
#include <thread>
#include <vector>

#include "swarmkit/client/client.h"
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

std::atomic<bool> g_running{true};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
}

struct DroneSpec {
    std::string drone_id;
    std::string agent_addr;
};

const std::vector<DroneSpec> kDrones = {
    {.drone_id = "drone-1", .agent_addr = "127.0.0.1:50061"},
    {.drone_id = "drone-2", .agent_addr = "127.0.0.1:50062"},
    {.drone_id = "drone-3", .agent_addr = "127.0.0.1:50063"},
};

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
void RunCommandLoop(const DroneSpec& spec) {
    if (!SleepWhileRunning(kStartupDelay)) {
        return;
    }

    sc::ClientConfig cfg;
    cfg.address = spec.agent_addr;
    cfg.client_id = "test-client";
    cfg.deadline_ms = kDeadlineMs;

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

int main() {
    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kInfo;
    swarmkit::core::Logger::Init(log_cfg);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    swarmkit::core::Logger::Info(
        "SwarmKit Test Client -- kOperator priority (0)\n"
        "Will be preempted by CLI (kSupervisor=10) and test-server (kOverride=20).\n"
        "Press Ctrl+C to stop.");

    std::vector<std::thread> threads;
    threads.reserve(kDrones.size());

    for (const auto& drone : kDrones) {
        threads.emplace_back(RunCommandLoop, drone);
    }

    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    return EXIT_SUCCESS;
}
