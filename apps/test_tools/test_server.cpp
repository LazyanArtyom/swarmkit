// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

/// @file test_server.cpp
/// @brief High-authority ground-control server (kOverride = 20) that connects a
///        SwarmClient to all three simulated drones, subscribes to telemetry
///        (writing frames to a CSV file), and exposes an interactive stdin
///        command loop for ad-hoc commands at the highest non-emergency priority.
///
/// Interactive syntax:
///   <drone-id>  arm | disarm | land | hold | return-home
///   <drone-id>  takeoff  <alt_m>
///   <drone-id>  waypoint <lat_deg> <lon_deg> <alt_m>
///   all         <command>          (broadcast to all drones)
///   quit  (or Ctrl+D)

#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace sc = swarmkit::client;
namespace cmd = swarmkit::commands;
namespace cor = swarmkit::core;
namespace fs = std::filesystem;

namespace {

constexpr int kDeadlineMs = 3000;
constexpr double kDefaultTakeoffAlt = 10.0;
constexpr double kDefaultWaypointAlt = 10.0;
constexpr int kCsvCoordPrecision = 6;
constexpr int kCsvValuePrecision = 2;
constexpr std::string_view kAllTarget = "all";
constexpr std::string_view kLockAction = "lock";
constexpr std::string_view kUnlockAction = "unlock";
constexpr std::string_view kServerClientId = "test-server";

class TestServerLogBackend final : public cor::ILogBackend {
   public:
    explicit TestServerLogBackend(std::shared_ptr<cor::ILogBackend> fallback_backend)
        : fallback_backend_(std::move(fallback_backend)) {}

    [[nodiscard]] bool IsEnabled(cor::LogLevel level) const override {
        return fallback_backend_ && fallback_backend_->IsEnabled(level);
    }

    void Log(cor::LogLevel level, std::string_view message) override {
        if (!fallback_backend_) {
            return;
        }

        fallback_backend_->Log(level, "[test-server] " + std::string(message));
    }

    void Flush() override {
        if (fallback_backend_) {
            fallback_backend_->Flush();
        }
    }

   private:
    std::shared_ptr<cor::ILogBackend> fallback_backend_;
};

volatile std::sig_atomic_t& StopRequestedFlag() {
    static volatile std::sig_atomic_t stop_requested = 0;
    return stop_requested;
}

void ResetStopRequested() {
    StopRequestedFlag() = 0;
}

[[nodiscard]] bool IsStopRequested() {
    return StopRequestedFlag() != 0;
}

extern "C" void OnSignal(int /*sig*/) {
    StopRequestedFlag() = 1;
    close(STDIN_FILENO);
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

[[nodiscard]] sc::SwarmAddressPreference ParseAddressPreference(std::string_view value) {
    if (value == "local") {
        return sc::SwarmAddressPreference::kPreferLocal;
    }
    return sc::SwarmAddressPreference::kPrimary;
}

[[nodiscard]] std::optional<cor::LogLevel> TryParseLogLevel(std::string_view value) {
    if (value == "trace") {
        return cor::LogLevel::kTrace;
    }
    if (value == "debug") {
        return cor::LogLevel::kDebug;
    }
    if (value == "info") {
        return cor::LogLevel::kInfo;
    }
    if (value == "warn") {
        return cor::LogLevel::kWarn;
    }
    if (value == "error") {
        return cor::LogLevel::kError;
    }
    if (value == "critical") {
        return cor::LogLevel::kCritical;
    }
    if (value == "off") {
        return cor::LogLevel::kOff;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<cor::LogSinkType> TryParseLogSink(std::string_view value) {
    if (value == "stdout" || value == "console") {
        return cor::LogSinkType::kStdout;
    }
    if (value == "file") {
        return cor::LogSinkType::kRotatingFile;
    }
    if (value == "both") {
        return cor::LogSinkType::kStdoutAndRotatingFile;
    }
    return std::nullopt;
}

const std::vector<std::pair<std::string, std::string>> kDrones = {
    {"drone-1", "127.0.0.1:50061"},
    {"drone-2", "127.0.0.1:50062"},
    {"drone-3", "127.0.0.1:50063"},
};

/// @brief Build a timestamped CSV path next to the executable.
fs::path MakeTelemetryPath(const char* argv0) {
    std::error_code error_code;
    auto exe_path = fs::weakly_canonical(fs::path(argv0), error_code);

    fs::path output_dir =
        (!error_code && exe_path.has_parent_path()) ? exe_path.parent_path() : fs::current_path();

    const auto kNowTp = std::chrono::system_clock::now();
    const auto kEpochSec = std::chrono::system_clock::to_time_t(kNowTp);
    const std::tm* local_tm = std::localtime(&kEpochSec);

    std::ostringstream oss;
    oss << "telemetry_" << std::put_time(local_tm, "%Y%m%d_%H%M%S") << ".csv";
    return output_dir / oss.str();
}

/// @brief Parsed user command ready for dispatch.
struct ParsedCommand {
    cmd::Command command;
    std::string description;
};

/// @brief Parse a single action token and its trailing arguments from the stream.
std::optional<ParsedCommand> ParseAction(const std::string& action, std::istringstream& tokens) {
    ParsedCommand out;

    if (action == "arm") {
        out.command = cmd::FlightCmd{cmd::CmdArm{}};
        out.description = "ARM";
    } else if (action == "disarm") {
        out.command = cmd::FlightCmd{cmd::CmdDisarm{}};
        out.description = "DISARM";
    } else if (action == "land") {
        out.command = cmd::FlightCmd{cmd::CmdLand{}};
        out.description = "LAND";
    } else if (action == "hold") {
        out.command = cmd::NavCmd{cmd::CmdHoldPosition{}};
        out.description = "HOLD";
    } else if (action == "return-home") {
        out.command = cmd::NavCmd{cmd::CmdReturnHome{}};
        out.description = "RETURN_HOME";
    } else if (action == "takeoff") {
        double alt_m = kDefaultTakeoffAlt;
        tokens >> alt_m;
        out.command = cmd::FlightCmd{cmd::CmdTakeoff{alt_m}};
        out.description = "TAKEOFF(" + std::to_string(static_cast<int>(alt_m)) + "m)";
    } else if (action == "waypoint") {
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        double alt_m = kDefaultWaypointAlt;
        tokens >> lat_deg >> lon_deg >> alt_m;
        out.command = cmd::NavCmd{cmd::CmdSetWaypoint{
            .lat_deg = lat_deg,
            .lon_deg = lon_deg,
            .alt_m = alt_m,
            .speed_mps = 0.0F,
        }};
        out.description = "WAYPOINT(" + std::to_string(lat_deg) + "," + std::to_string(lon_deg) +
                          "," + std::to_string(static_cast<int>(alt_m)) + "m)";
    } else {
        cor::Logger::ErrorFmt(
            "Unknown action: {}  (arm|disarm|land|hold|return-home|takeoff|waypoint)", action);
        return std::nullopt;
    }

    return out;
}

[[nodiscard]] bool IsAuthorityAction(std::string_view action) {
    return action == kLockAction || action == kUnlockAction;
}

[[nodiscard]] bool IsAllTarget(std::string_view target_id) {
    return target_id == kAllTarget;
}

[[nodiscard]] cmd::CommandContext MakeServerCommandContext() {
    cmd::CommandContext context;
    context.client_id = std::string(kServerClientId);
    context.priority = cmd::CommandPriority::kOverride;
    return context;
}

void PrintCommandResultLine(std::string_view target_id, std::string_view description,
                            const sc::CommandResult& result) {
    std::cout << "  [" << target_id << "] " << description << ": "
              << (result.ok ? "OK" : ("REJECTED -- " + result.message)) << "\n";
}

void PrintAuthorityResultLine(std::string_view target_id, std::string_view action,
                              const sc::CommandResult& result) {
    std::cout << "  [" << target_id << "] " << action << ": "
              << (result.ok ? "OK" : ("REJECTED -- " + result.message)) << "\n";
}

void HandleAuthorityForAll(sc::SwarmClient& swarm, std::string_view action) {
    if (action == kLockAction) {
        const auto kResults = swarm.LockAll();
        for (const auto& [drone_id, result] : kResults) {
            PrintAuthorityResultLine(drone_id, "LOCK", result);
        }
        return;
    }

    swarm.UnlockAll();
    std::cout << "  [all] UNLOCK: OK\n";
}

void HandleAuthorityForDrone(sc::SwarmClient& swarm, const std::string& target_id,
                             std::string_view action) {
    if (action == kLockAction) {
        const sc::CommandResult kResult = swarm.LockDrone(target_id);
        PrintAuthorityResultLine(target_id, "LOCK", kResult);
        return;
    }

    swarm.UnlockDrone(target_id);
    std::cout << "  [" << target_id << "] UNLOCK: OK\n";
}

void HandleAuthorityAction(sc::SwarmClient& swarm, const std::string& target_id,
                           std::string_view action) {
    if (IsAllTarget(target_id)) {
        HandleAuthorityForAll(swarm, action);
        return;
    }

    HandleAuthorityForDrone(swarm, target_id, action);
}

void DispatchParsedCommand(sc::SwarmClient& swarm, const std::string& target_id,
                           const ParsedCommand& parsed_command) {
    cmd::CommandContext context = MakeServerCommandContext();

    if (IsAllTarget(target_id)) {
        const auto kResults = swarm.BroadcastCommand(parsed_command.command, context);
        for (const auto& [drone_id, result] : kResults) {
            PrintCommandResultLine(drone_id, parsed_command.description, result);
        }
        return;
    }

    context.drone_id = target_id;
    cmd::CommandEnvelope envelope;
    envelope.context = std::move(context);
    envelope.command = parsed_command.command;

    const sc::CommandResult kResult = swarm.SendCommand(envelope);
    PrintCommandResultLine(target_id, parsed_command.description, kResult);
}

/// @brief Dispatch a single interactive command line to the swarm.
void ProcessLine(sc::SwarmClient& swarm, const std::string& line) {
    if (line.empty()) {
        return;
    }

    std::istringstream tokens(line);
    std::string target_id;
    std::string action;
    tokens >> target_id >> action;

    if (target_id.empty() || action.empty()) {
        return;
    }

    if (IsAuthorityAction(action)) {
        HandleAuthorityAction(swarm, target_id, action);
        return;
    }

    const auto kParsedCommand = ParseAction(action, tokens);
    if (!kParsedCommand) {
        return;
    }

    DispatchParsedCommand(swarm, target_id, *kParsedCommand);
}

}  // namespace

int main(int argc, char** argv) {
    cor::LoggerConfig log_cfg;
    log_cfg.sink_type = cor::LogSinkType::kStdout;
    log_cfg.level = cor::LogLevel::kInfo;

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
            log_cfg.sink_type = cor::LogSinkType::kRotatingFile;
        }
    }

    const auto kParsedLevel = TryParseLogLevel(GetArg(argc, argv, "--log-level", "info"));
    if (!kParsedLevel) {
        std::cerr << "Invalid --log-level value\n";
        return EXIT_FAILURE;
    }
    log_cfg.level = *kParsedLevel;

    if (log_cfg.sink_type == cor::LogSinkType::kRotatingFile && log_cfg.log_file_path.empty()) {
        std::cerr << "Invalid logger configuration: log file path is required for file sink\n";
        return EXIT_FAILURE;
    }

    cor::Logger::SetBackend(
        std::make_shared<TestServerLogBackend>(cor::Logger::CreateDefaultBackend(log_cfg)));

    ResetStopRequested();
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    sc::ClientConfig default_cfg;
    std::vector<std::pair<std::string, std::string>> drone_endpoints = kDrones;

    const std::string kSwarmConfigPath = GetArg(argc, argv, "--swarm-config", "");
    if (!kSwarmConfigPath.empty()) {
        const auto kSwarmConfig = sc::LoadSwarmConfigFromFile(kSwarmConfigPath);
        if (!kSwarmConfig.has_value()) {
            cor::Logger::ErrorFmt("Failed to load swarm config '{}': {}", kSwarmConfigPath,
                                  kSwarmConfig.error().message);
            return EXIT_FAILURE;
        }

        default_cfg = kSwarmConfig->default_client_config;
        drone_endpoints.clear();
        drone_endpoints.reserve(kSwarmConfig->drones.size());

        const auto kAddressPreference =
            ParseAddressPreference(GetArg(argc, argv, "--address-mode", "primary"));
        for (const auto& drone : kSwarmConfig->drones) {
            const bool kUseLocal =
                kAddressPreference == sc::SwarmAddressPreference::kPreferLocal &&
                !drone.local_address.empty();
            drone_endpoints.emplace_back(drone.drone_id,
                                         kUseLocal ? drone.local_address : drone.address);
        }
    }

    default_cfg.client_id = "test-server";
    default_cfg.deadline_ms = kDeadlineMs;
    default_cfg.priority = cmd::CommandPriority::kOverride;

    sc::SwarmClient swarm(default_cfg);
    for (const auto& [drone_id, addr] : drone_endpoints) {
        swarm.AddDrone(drone_id, addr);
    }

    const fs::path kTelemetryPath = MakeTelemetryPath(argv[0]);
    std::ofstream telemetry_file(kTelemetryPath);
    std::mutex file_mutex;

    if (!telemetry_file.is_open()) {
        cor::Logger::ErrorFmt("Failed to open telemetry file: {}", kTelemetryPath.string());
        return EXIT_FAILURE;
    }

    telemetry_file << "timestamp_ms,drone_id,lat_deg,lon_deg,rel_alt_m,"
                      "battery_pct,mode\n";
    telemetry_file.flush();

    std::cout << "SwarmKit Test Server -- kOverride priority (20)\n"
              << "Telemetry log: " << kTelemetryPath << "\n"
              << "Configured drones: " << drone_endpoints.size() << "\n\n"
              << "Commands: <drone-id> lock              (acquire exclusive authority)\n"
              << "          <drone-id> unlock            (release authority)\n"
              << "          <drone-id> arm|disarm|land|hold|return-home\n"
              << "          <drone-id> takeoff <alt_m>\n"
              << "          <drone-id> waypoint <lat> <lon> <alt>\n"
              << "          all <command>    (broadcast to all drones)\n"
              << "          quit\n\n";

    constexpr int kTelemetryRateHz = 2;

    swarm.SubscribeAllTelemetry(
        kTelemetryRateHz,
        [&file_mutex, &telemetry_file](const cor::TelemetryFrame& frame) {
            std::lock_guard<std::mutex> lock(file_mutex);
            telemetry_file << frame.unix_time_ms << "," << frame.drone_id << "," << std::fixed
                           << std::setprecision(kCsvCoordPrecision) << frame.lat_deg << ","
                           << frame.lon_deg << "," << std::setprecision(kCsvValuePrecision)
                           << frame.rel_alt_m << "," << frame.battery_percent << "," << frame.mode
                           << "\n";
            telemetry_file.flush();
        },
        [](const std::string& error_msg) {
            cor::Logger::ErrorFmt("[TEL] stream error: {}", error_msg);
        });

    std::string line;
    while (!IsStopRequested()) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)) {
            break;
        }
        if (line == "quit" || line == "exit") {
            break;
        }
        ProcessLine(swarm, line);
    }

    std::cout << "\nShutting down...\n";
    swarm.StopAllTelemetry();
    telemetry_file.close();
    return EXIT_SUCCESS;
}
