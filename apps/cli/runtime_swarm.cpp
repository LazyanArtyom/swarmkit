// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_swarm.h"

#include "runtime_common.h"
#include "runtime_telemetry.h"


namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::expected<SwarmRuntime, int> BuildSwarmRuntime(const ClientConfig& client_cfg,
                                                                 int argc, char** argv) {
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

    SwarmRuntime runtime;
    runtime.drone_ids.reserve(loaded->drones.size());
    for (const auto& drone : loaded->drones) {
        runtime.drone_ids.push_back(drone.drone_id);
    }
    runtime.client = std::make_unique<swarmkit::client::SwarmClient>(client_cfg);
    runtime.client_id = client_cfg.client_id;
    const std::string address_mode =
        common::GetOptionValue(argc, argv, "--address-mode", kDefaultAddressMode);
    const auto preference = address_mode == "local"
                                ? swarmkit::client::SwarmAddressPreference::kPreferLocal
                                : swarmkit::client::SwarmAddressPreference::kPrimary;
    if (const auto result = runtime.client->ApplyConfig(*loaded, preference); !result.IsOk()) {
        std::cerr << "Invalid swarm config: " << result.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    return runtime;
}

[[nodiscard]] SwarmResultSummary SummarizeSwarmResults(
    const std::unordered_map<std::string, swarmkit::client::CommandResult>& results) {
    SwarmResultSummary summary;
    summary.sent = static_cast<int>(results.size());
    for (const auto& [drone_id, result] : results) {
        static_cast<void>(drone_id);
        if (IsAlreadySatisfied(result)) {
            ++summary.already_satisfied;
            ++summary.succeeded;
        } else if (result.ok) {
            ++summary.accepted;
            ++summary.succeeded;
        } else {
            ++summary.failed;
        }
    }
    return summary;
}

[[nodiscard]] bool SwarmResultAccepted(const SwarmResultSummary& summary,
                                       const SwarmResultPolicy& policy) {
    if (summary.sent == 0) {
        return false;
    }
    if (policy.require_all) {
        return summary.failed == 0 && summary.succeeded == summary.sent;
    }
    if (policy.continue_on_error) {
        return summary.succeeded > 0;
    }
    return summary.failed == 0 && summary.succeeded == summary.sent;
}

[[nodiscard]] bool PrintSwarmResults(
    const std::unordered_map<std::string, swarmkit::client::CommandResult>& results,
    const SwarmResultPolicy& policy) {
    for (const auto& [drone_id, result] : results) {
        std::cout << drone_id << ": ";
        if (IsAlreadySatisfied(result)) {
            std::cout << "ALREADY_SATISFIED";
        } else {
            std::cout << (result.ok ? "OK" : "FAILED");
        }
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
    }
    const SwarmResultSummary summary = SummarizeSwarmResults(results);
    std::cout << "summary: sent=" << summary.sent << " succeeded=" << summary.succeeded
              << " accepted=" << summary.accepted
              << " already_satisfied=" << summary.already_satisfied << " failed=" << summary.failed
              << "\n";
    return SwarmResultAccepted(summary, policy);
}

[[nodiscard]] bool PrintSwarmReleaseResults(
    const std::unordered_map<std::string, swarmkit::client::ReleaseAuthorityResult>& results,
    const SwarmResultPolicy& policy) {
    SwarmResultSummary summary;
    summary.sent = static_cast<int>(results.size());
    for (const auto& [drone_id, result] : results) {
        std::cout << drone_id << ": " << (result.ok ? "OK" : "FAILED");
        if (!result.message.empty()) {
            std::cout << " " << result.message;
        }
        if (!result.correlation_id.empty()) {
            std::cout << " [corr=" << result.correlation_id << "]";
        }
        std::cout << "\n";
        if (result.ok) {
            ++summary.accepted;
            ++summary.succeeded;
        } else {
            ++summary.failed;
        }
    }
    std::cout << "summary: sent=" << summary.sent << " succeeded=" << summary.succeeded
              << " released=" << summary.accepted << " failed=" << summary.failed << "\n";
    return SwarmResultAccepted(summary, policy);
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

[[nodiscard]] bool ContainsDrone(const std::vector<std::string>& drone_ids,
                                 std::string_view drone_id) {
    return std::ranges::find(drone_ids, drone_id) != drone_ids.end();
}

int RunSwarmHealth(const SwarmRuntime& runtime) {
    bool all_ok = true;
    const std::int64_t now_ms = NowUnixMs();
    for (const auto& drone_id : runtime.drone_ids) {
        const auto status = runtime.client->GetHealth(drone_id);
        const bool heartbeat_stale =
            status.ok && IsStaleTimestamp(status.last_heartbeat_unix_ms, now_ms);
        const bool telemetry_stale =
            status.ok && IsStaleTimestamp(status.last_telemetry_unix_ms, now_ms);
        const std::string reason = SwarmHealthReason(status, heartbeat_stale, telemetry_stale);
        const bool healthy = status.ok && status.ready && !status.failsafe && status.gps_ok &&
                             status.ekf_ok && !heartbeat_stale && !telemetry_stale;
        all_ok = all_ok && healthy;
        std::cout << drone_id << ": " << (status.ok ? "OK" : "FAILED");
        if (status.ok) {
            std::cout << " ready=" << BoolText(status.ready) << " backend=" << status.backend_name
                      << " protocol=" << status.protocol << " armed=" << BoolText(status.armed)
                      << " landed=" << BoolText(status.landed)
                      << " failsafe=" << BoolText(status.failsafe)
                      << " gps=" << BoolText(status.gps_ok) << " ekf=" << BoolText(status.ekf_ok)
                      << " link_quality=" << OptionalFloatText(status.link_quality_percent)
                      << " heartbeat_age_ms="
                      << TimestampAgeMsText(status.last_heartbeat_unix_ms, now_ms)
                      << " heartbeat_stale=" << BoolText(heartbeat_stale)
                      << " telemetry_age_ms="
                      << TimestampAgeMsText(status.last_telemetry_unix_ms, now_ms)
                      << " telemetry_stale=" << BoolText(telemetry_stale)
                      << " agent_id=" << status.agent_id << " version=" << status.version;
        }
        if (!reason.empty()) {
            std::cout << " reason=" << reason;
        }
        std::cout << "\n";
    }
    return all_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunSwarmStats(const SwarmRuntime& runtime) {
    bool all_ok = true;
    for (const auto& drone_id : runtime.drone_ids) {
        const auto stats = runtime.client->GetRuntimeStats(drone_id);
        all_ok = all_ok && stats.ok;
        std::cout << drone_id << ": " << (stats.ok ? "OK" : "FAILED");
        if (stats.ok) {
            std::cout << " ready=" << (stats.ready ? "true" : "false")
                      << " commands=" << stats.command_requests_total
                      << " rejected=" << stats.command_rejected_total
                      << " failed=" << stats.command_failed_total
                      << " telemetry_streams=" << stats.current_telemetry_streams
                      << " frames=" << stats.telemetry_frames_sent_total
                      << " backend_failures=" << stats.backend_failures_total
                      << " data_messages=" << stats.data_messages_published_total
                      << " data_rejected=" << stats.data_messages_rejected_total
                      << " artifacts_up=" << stats.artifact_uploads_total
                      << " artifacts_down=" << stats.artifact_downloads_total
                      << " artifact_rx_bytes=" << stats.artifact_bytes_received_total
                      << " artifact_tx_bytes=" << stats.artifact_bytes_sent_total
                      << " artifact_failures=" << stats.artifact_failures_total;
        } else if (!stats.error.user_message.empty()) {
            std::cout << " " << stats.error.user_message;
        }
        std::cout << "\n";
    }
    return all_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int RunSwarmTelemetry(SwarmRuntime& runtime, int argc, char** argv) {
    const auto rate_hz = ParseTelemetryRate(argc, argv);
    if (!rate_hz.has_value()) {
        std::cerr << rate_hz.error() << "\n";
        return EXIT_FAILURE;
    }
    auto sink = TelemetrySink::FromArgs(argc, argv, true);
    if (!sink.has_value()) {
        std::cerr << sink.error() << "\n";
        return EXIT_FAILURE;
    }
    auto telemetry_sink = std::move(*sink);
    const auto duration_ms = ParseDurationMs(argc, argv);
    if (!duration_ms.has_value()) {
        std::cerr << duration_ms.error() << "\n";
        return EXIT_FAILURE;
    }

    const std::string drone_id = common::GetOptionValue(argc, argv, "--drone");
    if (!drone_id.empty() && !ContainsDrone(runtime.drone_ids, drone_id)) {
        std::cerr << "drone '" << drone_id << "' is not present in swarm config\n";
        return EXIT_FAILURE;
    }

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    if (drone_id.empty()) {
        std::cout << "Subscribing to swarm telemetry: drones=" << runtime.drone_ids.size()
                  << " rate=" << *rate_hz << " Hz\n";
        auto telemetry_streams = runtime.client->StartAllTelemetry(
            *rate_hz,
            [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
                telemetry_sink->Write(frame);
            },
            [](const std::string& error_msg) {
                std::cerr << "Telemetry stream error: " << error_msg << "\n";
            });
        bool started = true;
        for (const auto& [stream_drone_id, result] : telemetry_streams) {
            if (!result.has_value()) {
                std::cerr << "Failed to start telemetry stream for " << stream_drone_id << ": "
                          << result.error().user_message << "\n";
                started = false;
            }
        }
        if (!started) {
            for (auto& entry : telemetry_streams) {
                auto& result = entry.second;
                if (result.has_value()) {
                    result->Stop();
                }
            }
            return EXIT_FAILURE;
        }
        std::cout << "Press Ctrl+C to stop.\n\n";
        if (telemetry_sink->WritesFiles()) {
            std::cout << "Telemetry CSV logging enabled.\n";
        }
        WaitForStop(*duration_ms);
        for (auto& entry : telemetry_streams) {
            auto& result = entry.second;
            if (result.has_value()) {
                result->Stop();
            }
        }
    } else {
        std::cout << "Subscribing to swarm telemetry: drone=" << drone_id << " rate=" << *rate_hz
                  << " Hz\n";
        auto telemetry_stream = runtime.client->StartTelemetry(
            {.drone_id = drone_id, .rate_hertz = *rate_hz},
            [&telemetry_sink](const swarmkit::core::TelemetryFrame& frame) {
                telemetry_sink->Write(frame);
            },
            [](const std::string& error_msg) {
                std::cerr << "Telemetry stream error: " << error_msg << "\n";
            });
        if (!telemetry_stream.has_value()) {
            std::cerr << "Failed to start telemetry stream: "
                      << telemetry_stream.error().user_message << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "Press Ctrl+C to stop.\n\n";
        if (telemetry_sink->WritesFiles()) {
            std::cout << "Telemetry CSV logging enabled.\n";
        }
        WaitForStop(*duration_ms);
        telemetry_stream->Stop();
    }
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

int RunSwarmSequence(SwarmRuntime& runtime, CommandPriority priority, int argc, char** argv) {
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
    SwarmResultPolicy result_policy = ParseSwarmResultPolicy(argc, argv);
    if (!result_policy.require_all && !result_policy.continue_on_error) {
        result_policy.require_all = true;
    }
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }

    const std::string default_drone_id = common::GetOptionValue(argc, argv, "--drone");
    if (!default_drone_id.empty() && !ContainsDrone(runtime.drone_ids, default_drone_id)) {
        std::cerr << "drone '" << default_drone_id << "' is not present in swarm config\n";
        return EXIT_FAILURE;
    }

    const bool needs_telemetry = std::ranges::any_of(*steps, [](const SequenceStep& step) {
        return !step.wait_conditions.empty() || IsDisarmAction(step);
    });
    SequenceTelemetryMonitor monitor;
    if (needs_telemetry) {
        if (!monitor.StartSwarm(runtime, kDefaultSequenceTelemetryRateHz)) {
            return EXIT_FAILURE;
        }
    }

    int failed_steps = 0;
    for (std::size_t index = 0; index < steps->size(); ++index) {
        const SequenceStep& step = steps->at(index);
        DelaySequenceStep(step);
        const std::vector<std::string> target_drones =
            StepTargetDrones(step, default_drone_id, runtime.drone_ids);
        if (!step.wait_conditions.empty() && target_drones.size() == 1 &&
            target_drones.front().empty()) {
            std::cerr << "step " << index
                      << " wait requires drone: DRONE_ID, broadcast: true, or global --drone\n";
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (!step.wait_conditions.empty() &&
            !WaitForConditions(monitor, target_drones, step.wait_conditions, index)) {
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (step.args.empty()) {
            continue;
        }

        const auto command = BuildCommandFromTokens(step.args);
        if (!command.has_value()) {
            std::cerr << "step " << index << " build failed: " << command.error() << "\n";
            ++failed_steps;
            if (!step.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        const std::string drone_id = step.drone_id.empty() ? default_drone_id : step.drone_id;
        if (!step.broadcast && drone_id.empty()) {
            std::cerr << "step " << index
                      << " requires drone: DRONE_ID, broadcast: true, or global --drone\n";
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
        if (!step.broadcast && !ContainsDrone(runtime.drone_ids, drone_id)) {
            std::cerr << "step " << index << " drone '" << drone_id
                      << "' is not present in swarm config\n";
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }

        const int attempts = step.retries + 1;
        bool step_ok = false;
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
            if (step.broadcast) {
                if (attempts > 1) {
                    std::cout << "step " << index << " broadcast attempt=" << attempt << "/"
                              << attempts << "\n";
                }
                swarmkit::commands::CommandContext context;
                context.client_id = runtime.client_id;
                context.priority = priority;
                const bool verify_step = verify_commands || step.verify;
                step_ok = PrintSwarmResults(
                    verify_step ? runtime.client->BroadcastCommandAndWait(
                                      *command, context, StepWaitOptions(*wait_options, step))
                                : runtime.client->BroadcastCommand(*command, context),
                    result_policy);
            } else {
                const auto envelope =
                    MakeCommandEnvelope(drone_id, runtime.client_id, *command, priority);
                const bool verify_step = verify_commands || step.verify;
                const auto result = verify_step
                                        ? runtime.client->SendCommandAndWait(
                                              envelope, StepWaitOptions(*wait_options, step))
                                        : runtime.client->SendCommand(envelope);
                std::string label = "step " + std::to_string(index) + " drone=" + drone_id;
                if (attempts > 1) {
                    label += " attempt=" + std::to_string(attempt) + "/" + std::to_string(attempts);
                }
                step_ok = PrintCommandResult(label, result);
            }

            if (step_ok || attempt == attempts) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{step.retry_delay_ms});
        }

        if (!step_ok) {
            ++failed_steps;
            if (!step.continue_on_error && !result_policy.continue_on_error) {
                return EXIT_FAILURE;
            }
            continue;
        }
    }

    if (failed_steps > 0 && !result_policy.continue_on_error) {
        std::cerr << "swarm sequence completed with failed_steps=" << failed_steps << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "swarm sequence OK steps=" << steps->size() << "\n";
    return EXIT_SUCCESS;
}

int RunSwarm(const ClientConfig& client_cfg, int argc, char** argv) {
    auto runtime = BuildSwarmRuntime(client_cfg, argc, argv);
    if (!runtime.has_value()) {
        return runtime.error();
    }
    const auto actions = FindSwarmActions(argc, argv);
    if (actions.empty()) {
        std::cerr << "swarm requires health, stats, telemetry, sequence, command, lock-all, "
                     "unlock-all, broadcast, land-all, or rtl-all\n";
        return EXIT_FAILURE;
    }

    Command command;
    SwarmResultPolicy result_policy = ParseSwarmResultPolicy(argc, argv);
    if (!result_policy.require_all && !result_policy.continue_on_error) {
        result_policy.require_all = true;
    }
    if (actions[0] == "health") {
        return RunSwarmHealth(*runtime);
    }
    if (actions[0] == "stats") {
        return RunSwarmStats(*runtime);
    }
    if (actions[0] == "telemetry") {
        return RunSwarmTelemetry(*runtime, argc, argv);
    }
    if (actions[0] == "sequence") {
        return RunSwarmSequence(*runtime, client_cfg.priority, argc, argv);
    }
    if (actions[0] == "lock-all") {
        const auto ttl_ms = ParseTtlMs(argc, argv);
        if (!ttl_ms.has_value()) {
            std::cerr << ttl_ms.error() << "\n";
            return EXIT_FAILURE;
        }
        return PrintSwarmResults(runtime->client->LockAll(*ttl_ms), result_policy) ? EXIT_SUCCESS
                                                                                   : EXIT_FAILURE;
    }
    if (actions[0] == "unlock-all") {
        return PrintSwarmReleaseResults(runtime->client->UnlockAll(), result_policy) ? EXIT_SUCCESS
                                                                                     : EXIT_FAILURE;
    }
    if (actions[0] == "command") {
        const std::string drone_id = common::GetOptionValue(argc, argv, "--drone");
        if (drone_id.empty()) {
            std::cerr << "swarm command requires --drone DRONE_ID\n";
            return EXIT_FAILURE;
        }
        if (!ContainsDrone(runtime->drone_ids, drone_id)) {
            std::cerr << "drone '" << drone_id << "' is not present in swarm config\n";
            return EXIT_FAILURE;
        }
        std::vector<std::string> command_actions;
        if (actions.size() > 1) {
            command_actions.assign(actions.begin() + 1, actions.end());
        }
        const auto built = BuildCommandFromActions(command_actions, argc, argv);
        if (!built.has_value()) {
            std::cerr << built.error() << "\n";
            return EXIT_FAILURE;
        }
        swarmkit::commands::CommandEnvelope envelope =
            MakeCommandEnvelope(drone_id, client_cfg.client_id, *built, client_cfg.priority);
        const bool verify = common::HasFlag(argc, argv, "--verify");
        const auto wait_options = ParseCommandWaitOptions(argc, argv);
        if (!wait_options.has_value()) {
            std::cerr << wait_options.error() << "\n";
            return EXIT_FAILURE;
        }
        const auto result = verify ? runtime->client->SendCommandAndWait(envelope, *wait_options)
                                   : runtime->client->SendCommand(envelope);
        return PrintSwarmResults({{drone_id, result}}, result_policy) ? EXIT_SUCCESS : EXIT_FAILURE;
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
    context.client_id = client_cfg.client_id;
    context.priority = client_cfg.priority;
    const bool verify = common::HasFlag(argc, argv, "--verify");
    const auto wait_options = ParseCommandWaitOptions(argc, argv);
    if (!wait_options.has_value()) {
        std::cerr << wait_options.error() << "\n";
        return EXIT_FAILURE;
    }
    const auto results =
        verify ? runtime->client->BroadcastCommandAndWait(command, context, *wait_options)
               : runtime->client->BroadcastCommand(command, context);
    return PrintSwarmResults(results, result_policy) ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace swarmkit::apps::cli::internal
