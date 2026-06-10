// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_command.h"

#include "runtime_common.h"
#include "usage.h"

namespace swarmkit::apps::cli::internal {

int RunSequence(Client& client, std::string_view default_drone_id, std::string_view client_id,
                CommandPriority priority, int argc, char** argv) {
    const auto path = ParseSequenceFilePath(argc, argv);
    if (!path.has_value()) {
        std::cerr << path.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto steps = LoadSequenceSteps(*path);
    if (!steps.has_value()) {
        std::cerr << steps.error() << "\n";
        return EXIT_FAILURE;
    }
    const bool verify_commands = common::HasFlag(argc, argv, "--verify");
    const bool continue_on_error = common::HasFlag(argc, argv, "--continue-on-error");
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }

    const bool needs_telemetry = std::ranges::any_of(*steps, [](const SequenceStep& step) {
        return !step.wait_conditions.empty() || IsDisarmAction(step);
    });
    SequenceTelemetryMonitor monitor;
    if (needs_telemetry) {
        if (!monitor.StartSingle(client, std::string(default_drone_id),
                                 kDefaultSequenceTelemetryRateHz)) {
            return EXIT_FAILURE;
        }
    }

    int failed_steps = 0;
    for (std::size_t index = 0; index < steps->size(); ++index) {
        const SequenceStep& step = steps->at(index);
        DelaySequenceStep(step);
        const std::vector<std::string> target_drones = StepTargetDrones(step, default_drone_id, {});
        if (!step.wait_conditions.empty() &&
            !WaitForConditions(monitor, target_drones, step.wait_conditions, index)) {
            ++failed_steps;
            if (!step.continue_on_error && !continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (step.args.empty()) {
            continue;
        }

        const std::string drone_id =
            step.drone_id.empty() ? std::string(default_drone_id) : step.drone_id;
        const auto command = BuildCommandFromTokens(step.args);
        if (!command.has_value()) {
            std::cerr << "step " << index << " build failed: " << command.error() << "\n";
            ++failed_steps;
            if (!step.continue_on_error && !continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        bool step_ok = false;
        const int attempts = step.retries + 1;
        for (int attempt = 1; attempt <= attempts; ++attempt) {
            if (IsDisarmAction(step) && !IsEmergencyAction(step)) {
                WaitCondition landed_condition;
                landed_condition.wait_landed = true;
                landed_condition.timeout_ms = 30000;
                if (!WaitForConditions(monitor, target_drones, {landed_condition}, index)) {
                    step_ok = false;
                    break;
                }
            }
            const auto envelope = MakeCommandEnvelope(drone_id, client_id, *command, priority);
            const bool verify_step = verify_commands || step.verify;
            const auto result = verify_step ? client.SendCommandAndWait(
                                                  envelope, StepWaitOptions(*wait_options, step))
                                            : client.SendCommand(envelope);
            std::string label = "step " + std::to_string(index) + " drone=" + drone_id;
            if (attempts > 1) {
                label += " attempt=" + std::to_string(attempt) + "/" + std::to_string(attempts);
            }
            step_ok = PrintCommandResult(label, result);
            if (step_ok || attempt == attempts) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{step.retry_delay_ms});
        }

        if (!step_ok) {
            ++failed_steps;
            if (!step.continue_on_error && !continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
    }

    if (failed_steps > 0 && !continue_on_error) {
        std::cerr << "sequence completed with failed_steps=" << failed_steps << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "sequence OK steps=" << steps->size() << "\n";
    return EXIT_SUCCESS;
}

int RunCommand(Client& client, std::string_view drone_id, std::string_view client_id,
               CommandPriority priority, int argc, char** argv) {
    const auto kCommand = BuildCommandFromArgs(argc, argv);
    if (!kCommand.has_value()) {
        std::cerr << kCommand.error() << "\n";
        if (kCommand.error().starts_with("Unknown action:")) {
            std::cerr << "\n";
            PrintUsage();
        }
        return EXIT_FAILURE;
    }

    const auto envelope = MakeCommandEnvelope(drone_id, client_id, *kCommand, priority);
    const bool verify = common::HasFlag(argc, argv, "--verify");
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto kResult =
        verify ? client.SendCommandAndWait(envelope, *wait_options) : client.SendCommand(envelope);
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

}  // namespace swarmkit::apps::cli::internal
