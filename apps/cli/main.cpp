// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace {

std::atomic<bool> g_running{true};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
}

/// @brief Retrieve a keyed argument from the command line, or return a default.
[[nodiscard]] std::string GetArg(int argc, char** argv, std::string_view key,
                                 std::string_view default_value) {
    for (int idx = 1; idx + 1 < argc; ++idx) {
        if (std::string_view{argv[idx]} == key) {
            return {argv[idx + 1]};
        }
    }
    return std::string{default_value};
}

/// @brief Check whether a boolean flag is present on the command line.
[[nodiscard]] bool HasFlag(int argc, char** argv, std::string_view flag) {
    for (int idx = 1; idx < argc; ++idx) {
        if (std::string_view{argv[idx]} == flag) {
            return true;
        }
    }
    return false;
}

/// @brief Print CLI usage information to stdout.
void PrintUsage() {
    std::cout << "Usage: swarmkit-cli [ADDRESS] COMMAND [OPTIONS]\n"
                 "\n"
                 "  ADDRESS   Agent address, default: 127.0.0.1:50061\n"
                 "\n"
                 "Commands:\n"
                 "  ping                   Send a ping and print the agent response.\n"
                 "  telemetry              Subscribe to telemetry and print frames.\n"
                 "                         Press Ctrl+C to stop.\n"
                 "  command                Send a flight/nav command (Supervisor priority).\n"
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

/**
 * @brief Parse and send a single flight/nav command to the agent.
 *
 * Accepted action tokens (subcommand after --drone):
 *   arm  disarm  land  hold  return-home
 *   takeoff [--alt M]
 *   waypoint --lat L --lon L --alt M [--speed S]
 */
int RunCommand(swarmkit::client::Client& client, const std::string& drone_id, int argc,
               char** argv) {
    using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

    std::string action;
    for (int idx = 1; idx < argc; ++idx) {
        const std::string kCurrentArg = argv[idx];
        if (kCurrentArg == "--drone" || kCurrentArg == "--alt" || kCurrentArg == "--lat" ||
            kCurrentArg == "--lon" || kCurrentArg == "--speed" || kCurrentArg == "--rate" ||
            kCurrentArg == "command") {
            ++idx;
            continue;
        }
        if (kCurrentArg.starts_with("--")) {
            continue;
        }
        if (kCurrentArg == "ping" || kCurrentArg == "telemetry") {
            continue;
        }
        action = kCurrentArg;
        break;
    }

    if (action.empty()) {
        std::cerr << "No action specified. See --help for options.\n";
        return EXIT_FAILURE;
    }

    std::optional<Command> maybe_cmd;

    if (action == "arm") {
        maybe_cmd = FlightCmd{CmdArm{}};
    } else if (action == "disarm") {
        maybe_cmd = FlightCmd{CmdDisarm{}};
    } else if (action == "land") {
        maybe_cmd = FlightCmd{CmdLand{}};
    } else if (action == "hold") {
        maybe_cmd = NavCmd{CmdHoldPosition{}};
    } else if (action == "return-home") {
        maybe_cmd = NavCmd{CmdReturnHome{}};
    } else if (action == "takeoff") {
        const std::string kAltStr = GetArg(argc, argv, "--alt", "10");
        try {
            CmdTakeoff takeoff_cmd;
            takeoff_cmd.alt_m = std::stof(kAltStr);
            maybe_cmd = FlightCmd{takeoff_cmd};
        } catch (const std::exception& exc) {
            std::cerr << "Invalid --alt value '" << kAltStr << "': " << exc.what() << "\n";
            return EXIT_FAILURE;
        }
    } else if (action == "waypoint") {
        const std::string kLatStr = GetArg(argc, argv, "--lat", "0");
        const std::string kLonStr = GetArg(argc, argv, "--lon", "0");
        const std::string kAltStr = GetArg(argc, argv, "--alt", "0");
        const std::string kSpeedStr = GetArg(argc, argv, "--speed", "0");
        try {
            CmdSetWaypoint waypoint_cmd;
            waypoint_cmd.lat_deg = std::stod(kLatStr);
            waypoint_cmd.lon_deg = std::stod(kLonStr);
            waypoint_cmd.alt_m = std::stof(kAltStr);
            waypoint_cmd.speed_mps = std::stof(kSpeedStr);
            maybe_cmd = NavCmd{waypoint_cmd};
        } catch (const std::exception& exc) {
            std::cerr << "Invalid numeric argument: " << exc.what() << "\n";
            return EXIT_FAILURE;
        }
    } else {
        std::cerr << "Unknown action: " << action << "\n\n";
        PrintUsage();
        return EXIT_FAILURE;
    }

    CommandEnvelope envelope;
    envelope.context.drone_id = drone_id;
    envelope.context.client_id = "swarmkit-cli";
    envelope.context.priority = CommandPriority::kSupervisor;
    envelope.command = std::move(*maybe_cmd);

    const swarmkit::client::CommandResult kResult = client.SendCommand(envelope);
    if (!kResult.ok) {
        std::cerr << "Command REJECTED: " << kResult.message << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Command OK" << (kResult.message.empty() ? "" : ": " + kResult.message) << "\n";
    return EXIT_SUCCESS;
}

/// @brief Send a ping RPC and print the response.
int RunPing(swarmkit::client::Client& client) {
    const swarmkit::client::PingResult kResult = client.Ping();

    if (!kResult.ok) {
        std::cerr << "Ping FAILED: " << kResult.error_message << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "Ping OK\n"
              << "  agent_id  : " << kResult.agent_id << "\n"
              << "  version   : " << kResult.version << "\n"
              << "  time_ms   : " << kResult.unix_time_ms << "\n";
    return EXIT_SUCCESS;
}

/// @brief Subscribe to telemetry and print frames until Ctrl+C.
int RunTelemetry(swarmkit::client::Client& client, const std::string& drone_id, int rate_hz) {
    static constexpr int kPollIntervalMs = 100;
    static constexpr int kCoordPrecision = 5;
    static constexpr int kValuePrecision = 1;

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::cout << "Subscribing to telemetry: drone=" << drone_id << " rate=" << rate_hz << " Hz\n"
              << "Press Ctrl+C to stop.\n\n";

    swarmkit::client::TelemetrySubscription subscription;
    subscription.drone_id = drone_id;
    subscription.rate_hertz = rate_hz;

    client.SubscribeTelemetry(
        subscription,
        [](const swarmkit::core::TelemetryFrame& frame) {
            std::cout << std::fixed << std::setprecision(kCoordPrecision) << "["
                      << frame.unix_time_ms << "]"
                      << " drone=" << frame.drone_id << " lat=" << frame.lat_deg
                      << " lon=" << frame.lon_deg << std::setprecision(kValuePrecision)
                      << " alt=" << frame.rel_alt_m << "m"
                      << " bat=" << frame.battery_percent << "%"
                      << " mode=" << frame.mode << "\n";
        },
        [](const std::string& error_msg) {
            std::cerr << "Telemetry stream error: " << error_msg << "\n";
        });

    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
    }

    client.StopTelemetry();
    std::cout << "\nStopped.\n";
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    if (HasFlag(argc, argv, "--help") || HasFlag(argc, argv, "-h")) {
        PrintUsage();
        return EXIT_SUCCESS;
    }

    static constexpr std::string_view kDefaultAddr = "127.0.0.1:50061";
    static constexpr std::string_view kDefaultCommand = "ping";

    std::string addr;
    std::string command;

    auto is_subcommand = [](std::string_view str) {
        return str == "ping" || str == "telemetry" || str == "command";
    };

    if (argc >= 3 && std::string(argv[1]).find(':') != std::string::npos) {
        addr = argv[1];
        command = argv[2];
    } else if (argc >= 2) {
        const std::string kFirstArg = argv[1];
        if (is_subcommand(kFirstArg)) {
            addr = kDefaultAddr;
            command = kFirstArg;
        } else {
            addr = kFirstArg;
            command = kDefaultCommand;
        }
    } else {
        addr = kDefaultAddr;
        command = kDefaultCommand;
    }

    swarmkit::core::LoggerConfig log_cfg;
    log_cfg.sink_type = swarmkit::core::LogSinkType::kStdout;
    log_cfg.level = swarmkit::core::LogLevel::kWarn;  // CLI uses stdout directly
    swarmkit::core::Logger::Init(log_cfg);

    swarmkit::client::ClientConfig client_cfg;
    client_cfg.address = addr;
    client_cfg.client_id = "swarmkit-cli";

    swarmkit::client::Client client(client_cfg);

    if (command == "ping") {
        return RunPing(client);
    }

    if (command == "telemetry") {
        const std::string kDroneId = GetArg(argc, argv, "--drone", "default");
        const std::string kRateStr = GetArg(argc, argv, "--rate", "1");
        try {
            const int kRateHz = std::stoi(kRateStr);
            return RunTelemetry(client, kDroneId, kRateHz);
        } catch (const std::exception& exc) {
            std::cerr << "Invalid --rate value '" << kRateStr << "': " << exc.what() << "\n";
            return EXIT_FAILURE;
        }
    }

    if (command == "command") {
        const std::string kDroneId = GetArg(argc, argv, "--drone", "default");
        return RunCommand(client, kDroneId, argc, argv);
    }

    std::cerr << "Unknown command: " << command << "\n\n";
    PrintUsage();
    return EXIT_FAILURE;
}
