// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "app.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "common/arg_utils.h"
#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::apps::cli {
namespace {

using swarmkit::client::Client;
using swarmkit::client::ClientConfig;
using swarmkit::commands::CmdArm;
using swarmkit::commands::CmdDisarm;
using swarmkit::commands::CmdHoldPosition;
using swarmkit::commands::CmdLand;
using swarmkit::commands::CmdReturnHome;
using swarmkit::commands::CmdSetWaypoint;
using swarmkit::commands::CmdTakeoff;
using swarmkit::commands::Command;
using swarmkit::commands::CommandEnvelope;
using swarmkit::commands::CommandPriority;
using swarmkit::commands::FlightCmd;
using swarmkit::commands::NavCmd;

[[nodiscard]] volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

constexpr std::string_view kDefaultAddr = "127.0.0.1:50061";
constexpr std::string_view kDefaultCommand = "ping";
constexpr std::string_view kDefaultDroneId = "default";
constexpr std::string_view kDefaultTakeoffAlt = "10";
constexpr std::string_view kDefaultWaypointCoord = "0";
constexpr std::string_view kDefaultWaypointSpeed = "0";
constexpr std::string_view kDefaultTelemetryRate = "1";
constexpr std::string_view kDefaultCliLogLevel = "warn";
constexpr std::string_view kCliClientId = "swarmkit-cli";
constexpr int kTelemetryPollIntervalMs = 100;
constexpr int kTelemetryCoordPrecision = 5;
constexpr int kTelemetryValuePrecision = 1;

extern "C" void OnSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
}

[[nodiscard]] bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

struct CliInvocation {
    std::string address;
    std::string command;
    bool has_explicit_address{false};
};

[[nodiscard]] bool IsSubcommand(std::string_view value) {
    return value == "ping" || value == "health" || value == "stats" || value == "telemetry" ||
           value == "command";
}

[[nodiscard]] bool IsOptionWithValue(std::string_view value) {
    return value == "--config" || value == "--drone" || value == "--rate" || value == "--alt" ||
           value == "--lat" || value == "--lon" || value == "--speed" || value == "--log-sink" ||
           value == "--log-file" || value == "--log-level" || value == "--ca-cert" ||
           value == "--client-cert" || value == "--client-key" || value == "--server-name";
}

void PrintUsage() {
    std::cout << "Usage: swarmkit-cli [ADDRESS] COMMAND [OPTIONS]\n"
                 "\n"
                 "  ADDRESS   Agent address, default: 127.0.0.1:50061\n"
                 "\n"
                 "Commands:\n"
                 "  ping                   Send a ping and print the agent response.\n"
                 "  health                 Read agent health/readiness.\n"
                 "  stats                  Read agent runtime counters.\n"
                 "  telemetry              Subscribe to telemetry and print frames.\n"
                 "                         Press Ctrl+C to stop.\n"
                 "  command                Send a flight/nav command (Supervisor priority).\n"
                 "\n"
                 "Global options:\n"
                 "  --config PATH          Load client config from YAML file\n"
                 "  --ca-cert PATH         mTLS CA certificate path\n"
                 "  --client-cert PATH     mTLS client certificate path\n"
                 "  --client-key PATH      mTLS client private key path\n"
                 "  --server-name NAME     TLS server name / authority override\n"
                 "  --log-sink TYPE        stdout|file|both for SDK/runtime logs\n"
                 "  --log-file PATH        Rotating log file path when file logging is used\n"
                 "  --log-level LEVEL      trace|debug|info|warn|error|critical|off\n"
                 "\n"
                 "Telemetry options:\n"
                 "  --drone  DRONE_ID      Drone to subscribe to (default: default)\n"
                 "  --rate   HZ            Frames per second     (default: 1)\n"
                 "\n"
                 "Command options:\n"
                 "  --drone  DRONE_ID      Target drone (default: default)\n"
                 "  arm                    Arm motors\n"
                 "  disarm                 Disarm motors\n"
                 "  land                   Land in place\n"
                 "  hold                   Hold current position\n"
                 "  return-home            Return to home\n"
                 "  takeoff [--alt M]      Take off to altitude in metres (default: 10)\n"
                 "  waypoint --lat L --lon L --alt M [--speed S]\n"
                 "                         Fly to GPS waypoint\n"
                 "\n"
                 "Examples:\n"
                 "  swarmkit-cli ping\n"
                 "  swarmkit-cli 192.168.1.10:50061 ping\n"
                 "  swarmkit-cli telemetry --drone uav-1 --rate 2\n"
                 "  swarmkit-cli command --drone uav-1 arm\n"
                 "  swarmkit-cli command --drone uav-1 takeoff --alt 30\n"
                 "  swarmkit-cli command --drone uav-1 waypoint --lat 40.18 --lon 44.51 --alt 50\n";
}

[[nodiscard]] std::expected<float, std::string> ParseFloatArg(std::string_view value,
                                                              std::string_view key) {
    try {
        return std::stof(std::string(value));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid " + std::string(key) + " value '" + std::string(value) +
                               "': " + exc.what());
    }
}

[[nodiscard]] std::expected<double, std::string> ParseDoubleArg(std::string_view value,
                                                                std::string_view key) {
    try {
        return std::stod(std::string(value));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid " + std::string(key) + " value '" + std::string(value) +
                               "': " + exc.what());
    }
}

[[nodiscard]] std::optional<std::string> FindCommandAction(int argc, char** argv) {
    int command_index = -1;
    for (int index = 1; index < argc; ++index) {
        const std::string_view kCurrentArg = argv[index];
        if (IsOptionWithValue(kCurrentArg)) {
            ++index;
            continue;
        }
        if (kCurrentArg == "command") {
            command_index = index;
            break;
        }
    }
    if (command_index < 0) {
        return std::nullopt;
    }

    for (int index = command_index + 1; index < argc; ++index) {
        const std::string_view kCurrentArg = argv[index];
        if (IsOptionWithValue(kCurrentArg)) {
            ++index;
            continue;
        }
        if (kCurrentArg.starts_with("-") || IsSubcommand(kCurrentArg)) {
            continue;
        }
        return std::string(kCurrentArg);
    }
    return std::nullopt;
}

[[nodiscard]] std::expected<Command, std::string> BuildTakeoffCommand(int argc, char** argv) {
    const auto kAltitude =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--alt", kDefaultTakeoffAlt), "--alt");
    if (!kAltitude.has_value()) {
        return std::unexpected(kAltitude.error());
    }

    CmdTakeoff takeoff_cmd;
    takeoff_cmd.alt_m = *kAltitude;
    return FlightCmd{takeoff_cmd};
}

[[nodiscard]] std::expected<Command, std::string> BuildWaypointCommand(int argc, char** argv) {
    const auto kLatitude =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat", kDefaultWaypointCoord), "--lat");
    if (!kLatitude.has_value()) {
        return std::unexpected(kLatitude.error());
    }

    const auto kLongitude =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon", kDefaultWaypointCoord), "--lon");
    if (!kLongitude.has_value()) {
        return std::unexpected(kLongitude.error());
    }

    const auto kAltitude =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--alt", kDefaultWaypointCoord), "--alt");
    if (!kAltitude.has_value()) {
        return std::unexpected(kAltitude.error());
    }

    const auto kSpeed = ParseFloatArg(
        common::GetOptionValue(argc, argv, "--speed", kDefaultWaypointSpeed), "--speed");
    if (!kSpeed.has_value()) {
        return std::unexpected(kSpeed.error());
    }

    CmdSetWaypoint waypoint_cmd;
    waypoint_cmd.lat_deg = *kLatitude;
    waypoint_cmd.lon_deg = *kLongitude;
    waypoint_cmd.alt_m = *kAltitude;
    waypoint_cmd.speed_mps = *kSpeed;
    return NavCmd{waypoint_cmd};
}

[[nodiscard]] std::expected<Command, std::string> BuildCommandFromArgs(int argc, char** argv) {
    const auto kAction = FindCommandAction(argc, argv);
    if (!kAction.has_value()) {
        return std::unexpected("No action specified. See --help for options.");
    }

    if (*kAction == "arm") {
        return FlightCmd{CmdArm{}};
    }
    if (*kAction == "disarm") {
        return FlightCmd{CmdDisarm{}};
    }
    if (*kAction == "land") {
        return FlightCmd{CmdLand{}};
    }
    if (*kAction == "hold") {
        return NavCmd{CmdHoldPosition{}};
    }
    if (*kAction == "return-home") {
        return NavCmd{CmdReturnHome{}};
    }
    if (*kAction == "takeoff") {
        return BuildTakeoffCommand(argc, argv);
    }
    if (*kAction == "waypoint") {
        return BuildWaypointCommand(argc, argv);
    }
    return std::unexpected("Unknown action: " + *kAction);
}

[[nodiscard]] CommandEnvelope MakeCommandEnvelope(std::string_view drone_id, Command command) {
    CommandEnvelope envelope;
    envelope.context.drone_id = std::string(drone_id);
    envelope.context.client_id = std::string(kCliClientId);
    envelope.context.priority = CommandPriority::kSupervisor;
    envelope.command = std::move(command);
    return envelope;
}

[[nodiscard]] std::expected<CliInvocation, int> ParseInvocation(int argc, char** argv) {
    std::vector<std::string> positional_args;
    positional_args.reserve(static_cast<std::size_t>(argc));
    for (int idx = 1; idx < argc; ++idx) {
        const std::string_view kArg = argv[idx];
        if (IsOptionWithValue(kArg)) {
            ++idx;
            continue;
        }
        if (kArg.starts_with('-')) {
            continue;
        }
        positional_args.emplace_back(kArg);
    }

    CliInvocation invocation;
    if (!positional_args.empty() && !IsSubcommand(positional_args.front())) {
        invocation.has_explicit_address = true;
        invocation.address = positional_args.front();
        invocation.command =
            positional_args.size() >= 2 ? positional_args[1] : std::string(kDefaultCommand);
        return invocation;
    }

    invocation.command =
        positional_args.empty() ? std::string(kDefaultCommand) : positional_args.front();
    return invocation;
}

[[nodiscard]] std::expected<swarmkit::core::LoggerConfig, int> BuildCliLoggerConfig(int argc,
                                                                                    char** argv) {
    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kWarn;

    const std::string kLogSink = common::GetOptionValue(argc, argv, "--log-sink");
    if (!kLogSink.empty()) {
        const auto kParsedSink = swarmkit::core::ParseLogSinkType(kLogSink);
        if (!kParsedSink.has_value()) {
            std::cerr << "Invalid --log-sink value: " << kParsedSink.error().message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        log_cfg.sink_type = *kParsedSink;
    }

    const std::string kLogFile = common::GetOptionValue(argc, argv, "--log-file");
    if (!kLogFile.empty()) {
        log_cfg.log_file_path = kLogFile;
        if (kLogSink.empty()) {
            log_cfg.sink_type = swarmkit::core::LogSinkType::kRotatingFile;
        }
    }

    const auto kParsedLevel = swarmkit::core::ParseLogLevel(
        common::GetOptionValue(argc, argv, "--log-level", kDefaultCliLogLevel));
    if (!kParsedLevel.has_value()) {
        std::cerr << "Invalid --log-level value: " << kParsedLevel.error().message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    log_cfg.level = *kParsedLevel;

    if (const swarmkit::core::Result kValidation = swarmkit::core::ValidateLoggerConfig(log_cfg);
        !kValidation.IsOk()) {
        std::cerr << "Invalid logger configuration: " << kValidation.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }

    return log_cfg;
}

[[nodiscard]] std::expected<ClientConfig, int> BuildCliClientConfig(const CliInvocation& invocation,
                                                                    int argc, char** argv) {
    ClientConfig client_cfg;
    const std::string kConfigPath = common::GetOptionValue(argc, argv, "--config");
    if (!kConfigPath.empty()) {
        const auto kLoaded = swarmkit::client::LoadClientConfigFromFile(kConfigPath);
        if (!kLoaded.has_value()) {
            std::cerr << "Failed to load client config '" << kConfigPath
                      << "': " << kLoaded.error().message << "\n";
            return std::unexpected(EXIT_FAILURE);
        }
        client_cfg = *kLoaded;
    }

    client_cfg.ApplyEnvironment();
    client_cfg.client_id = std::string(kCliClientId);
    if (const std::string kRootCaCert = common::GetOptionValue(argc, argv, "--ca-cert");
        !kRootCaCert.empty()) {
        client_cfg.security.root_ca_cert_path = kRootCaCert;
    }
    if (const std::string kClientCert = common::GetOptionValue(argc, argv, "--client-cert");
        !kClientCert.empty()) {
        client_cfg.security.cert_chain_path = kClientCert;
    }
    if (const std::string kClientKey = common::GetOptionValue(argc, argv, "--client-key");
        !kClientKey.empty()) {
        client_cfg.security.private_key_path = kClientKey;
    }
    if (const std::string kServerName = common::GetOptionValue(argc, argv, "--server-name");
        !kServerName.empty()) {
        client_cfg.security.server_authority_override = kServerName;
    }
    if (invocation.has_explicit_address) {
        client_cfg.address = invocation.address;
    } else if (client_cfg.address.empty()) {
        client_cfg.address = std::string(kDefaultAddr);
    }

    if (const swarmkit::core::Result kValidation = client_cfg.Validate(); !kValidation.IsOk()) {
        std::cerr << "Invalid client configuration: " << kValidation.message << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    return client_cfg;
}

int RunCommand(Client& client, std::string_view drone_id, int argc, char** argv) {
    const auto kCommand = BuildCommandFromArgs(argc, argv);
    if (!kCommand.has_value()) {
        std::cerr << kCommand.error() << "\n";
        if (kCommand.error().starts_with("Unknown action:")) {
            std::cerr << "\n";
            PrintUsage();
        }
        return EXIT_FAILURE;
    }

    const auto kResult = client.SendCommand(MakeCommandEnvelope(drone_id, *kCommand));
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

[[nodiscard]] int DispatchCommand(const CliInvocation& invocation, Client& client, int argc,
                                  char** argv) {
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
                          argc, argv);
    }

    std::cerr << "Unknown command: " << invocation.command << "\n\n";
    PrintUsage();
    return EXIT_FAILURE;
}

}  // namespace

int RunCliApp(int argc, char** argv) {
    if (common::HasFlag(argc, argv, "--help") || common::HasFlag(argc, argv, "-h")) {
        PrintUsage();
        return EXIT_SUCCESS;
    }

    const auto kInvocation = ParseInvocation(argc, argv);
    if (!kInvocation.has_value()) {
        return kInvocation.error();
    }

    const auto kLoggerConfig = BuildCliLoggerConfig(argc, argv);
    if (!kLoggerConfig.has_value()) {
        return kLoggerConfig.error();
    }
    swarmkit::core::Logger::Init(*kLoggerConfig);

    const auto kClientConfig = BuildCliClientConfig(*kInvocation, argc, argv);
    if (!kClientConfig.has_value()) {
        return kClientConfig.error();
    }

    Client client(*kClientConfig);
    return DispatchCommand(*kInvocation, client, argc, argv);
}

}  // namespace swarmkit::apps::cli
