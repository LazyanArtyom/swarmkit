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

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace sc  = swarmkit::client;
namespace cmd = swarmkit::commands;
namespace cor = swarmkit::core;
namespace fs  = std::filesystem;

namespace {

constexpr int    kDeadlineMs         = 3000;
constexpr double kDefaultTakeoffAlt  = 10.0;
constexpr double kDefaultWaypointAlt = 10.0;
constexpr int    kCsvCoordPrecision  = 6;
constexpr int    kCsvValuePrecision  = 2;

std::atomic<bool> g_running{true};

extern "C" void OnSignal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
    close(STDIN_FILENO);
}

const std::vector<std::pair<std::string, std::string>> kDrones = {
    {"drone-1", "127.0.0.1:50061"},
    {"drone-2", "127.0.0.1:50062"},
    {"drone-3", "127.0.0.1:50063"},
};

/// @brief Build a timestamped CSV path next to the executable.
fs::path MakeTelemetryPath(const char* argv0) {
    std::error_code error_code;
    auto            exe_path = fs::weakly_canonical(fs::path(argv0), error_code);

    fs::path output_dir = (!error_code && exe_path.has_parent_path())
                              ? exe_path.parent_path()
                              : fs::current_path();

    const auto     now_tp    = std::chrono::system_clock::now();
    const auto     epoch_sec = std::chrono::system_clock::to_time_t(now_tp);
    const std::tm* local_tm  = std::localtime(&epoch_sec);

    std::ostringstream oss;
    oss << "telemetry_" << std::put_time(local_tm, "%Y%m%d_%H%M%S") << ".csv";
    return output_dir / oss.str();
}

/// @brief Parsed user command ready for dispatch.
struct ParsedCommand {
    cmd::Command command;
    std::string  description;
};

/// @brief Parse a single action token and its trailing arguments from the stream.
std::optional<ParsedCommand> ParseAction(const std::string& action,
                                         std::istringstream& tokens) {
    ParsedCommand out;

    if (action == "arm") {
        out.command     = cmd::FlightCmd{cmd::CmdArm{}};
        out.description = "ARM";
    } else if (action == "disarm") {
        out.command     = cmd::FlightCmd{cmd::CmdDisarm{}};
        out.description = "DISARM";
    } else if (action == "land") {
        out.command     = cmd::FlightCmd{cmd::CmdLand{}};
        out.description = "LAND";
    } else if (action == "hold") {
        out.command     = cmd::NavCmd{cmd::CmdHoldPosition{}};
        out.description = "HOLD";
    } else if (action == "return-home") {
        out.command     = cmd::NavCmd{cmd::CmdReturnHome{}};
        out.description = "RETURN_HOME";
    } else if (action == "takeoff") {
        double alt_m = kDefaultTakeoffAlt;
        tokens >> alt_m;
        out.command     = cmd::FlightCmd{cmd::CmdTakeoff{alt_m}};
        out.description = "TAKEOFF(" + std::to_string(static_cast<int>(alt_m)) + "m)";
    } else if (action == "waypoint") {
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        double alt_m   = kDefaultWaypointAlt;
        tokens >> lat_deg >> lon_deg >> alt_m;
        out.command     = cmd::NavCmd{cmd::CmdSetWaypoint{
            .lat_deg   = lat_deg,
            .lon_deg   = lon_deg,
            .alt_m     = alt_m,
            .speed_mps = 0.0F,
        }};
        out.description = "WAYPOINT(" + std::to_string(lat_deg) + ","
                        + std::to_string(lon_deg) + ","
                        + std::to_string(static_cast<int>(alt_m)) + "m)";
    } else {
        cor::Logger::ErrorFmt(
            "Unknown action: {}  (arm|disarm|land|hold|return-home|takeoff|waypoint)", action);
        return std::nullopt;
    }

    return out;
}

/// @brief Dispatch a single interactive command line to the swarm.
void ProcessLine(sc::SwarmClient& swarm, const std::string& line) {
    if (line.empty()) {
        return;
    }

    std::istringstream tokens(line);
    std::string        target_id;
    std::string        action;
    tokens >> target_id >> action;

    if (target_id.empty() || action.empty()) {
        return;
    }

    if (action == "lock" || action == "unlock") {
        if (target_id == "all") {
            if (action == "lock") {
                const auto results = swarm.LockAll();
                for (const auto& [drone_id, res] : results) {
                    std::cout << "  [" << drone_id << "] LOCK: "
                              << (res.ok ? "OK" : ("REJECTED -- " + res.message)) << "\n";
                }
            } else {
                swarm.UnlockAll();
                std::cout << "  [all] UNLOCK: OK\n";
            }
        } else {
            if (action == "lock") {
                const sc::CommandResult result = swarm.LockDrone(target_id);
                std::cout << "  [" << target_id << "] LOCK: "
                          << (result.ok ? "OK" : ("REJECTED -- " + result.message)) << "\n";
            } else {
                swarm.UnlockDrone(target_id);
                std::cout << "  [" << target_id << "] UNLOCK: OK\n";
            }
        }
        return;
    }

    auto parsed = ParseAction(action, tokens);
    if (!parsed) {
        return;
    }

    cmd::CommandContext ctx;
    ctx.client_id = "test-server";
    ctx.priority  = cmd::CommandPriority::kOverride;

    if (target_id == "all") {
        const auto results = swarm.BroadcastCommand(parsed->command, ctx);
        for (const auto& [drone_id, res] : results) {
            std::cout << "  [" << drone_id << "] " << parsed->description << ": "
                      << (res.ok ? "OK" : ("REJECTED -- " + res.message)) << "\n";
        }
    } else {
        ctx.drone_id = target_id;
        cmd::CommandEnvelope envelope;
        envelope.context = ctx;
        envelope.command = parsed->command;
        const sc::CommandResult result = swarm.SendCommand(envelope);
        std::cout << "  [" << target_id << "] " << parsed->description << ": "
                  << (result.ok ? "OK" : ("REJECTED -- " + result.message)) << "\n";
    }
}

}  // namespace

int main(int /*argc*/, char** argv) {
    cor::LoggerConfig log_cfg;
    log_cfg.sink_type = cor::LogSinkType::kStdout;
    log_cfg.level = cor::LogLevel::kInfo;
    cor::Logger::Init(log_cfg);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    sc::ClientConfig default_cfg;
    default_cfg.client_id   = "test-server";
    default_cfg.deadline_ms = kDeadlineMs;
    default_cfg.priority    = cmd::CommandPriority::kOverride;

    sc::SwarmClient swarm(default_cfg);
    for (const auto& [drone_id, addr] : kDrones) {
        swarm.AddDrone(drone_id, addr);
    }

    const fs::path telemetry_path = MakeTelemetryPath(argv[0]);
    std::ofstream  telemetry_file(telemetry_path);
    std::mutex     file_mutex;

    if (!telemetry_file.is_open()) {
        cor::Logger::ErrorFmt("Failed to open telemetry file: {}", telemetry_path.string());
        return EXIT_FAILURE;
    }

    telemetry_file << "timestamp_ms,drone_id,lat_deg,lon_deg,rel_alt_m,"
                      "battery_pct,mode\n";
    telemetry_file.flush();

    std::cout << "SwarmKit Test Server -- kOverride priority (20)\n"
              << "Telemetry log: " << telemetry_path << "\n"
              << "Drones: drone-1 (50061)  drone-2 (50062)  drone-3 (50063)\n\n"
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
            telemetry_file << frame.unix_time_ms << ","
                           << frame.drone_id << ","
                           << std::fixed << std::setprecision(kCsvCoordPrecision)
                           << frame.lat_deg << ","
                           << frame.lon_deg << ","
                           << std::setprecision(kCsvValuePrecision)
                           << frame.rel_alt_m << ","
                           << frame.battery_percent << ","
                           << frame.mode << "\n";
            telemetry_file.flush();
        },
        [](const std::string& error_msg) {
            cor::Logger::ErrorFmt("[TEL] stream error: {}", error_msg);
        });

    std::string line;
    while (g_running.load(std::memory_order_relaxed)) {
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
