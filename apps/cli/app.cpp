// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "app.h"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "common/arg_utils.h"
#include "swarmkit/client/client.h"
#include "swarmkit/client/swarm_client.h"
#include "swarmkit/commands.h"
#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::apps::cli {
namespace {

using swarmkit::client::Client;
using swarmkit::client::ClientConfig;
using swarmkit::commands::CmdArm;
using swarmkit::commands::CmdClearMission;
using swarmkit::commands::CmdDisarm;
using swarmkit::commands::CmdFlightTerminate;
using swarmkit::commands::CmdForceDisarm;
using swarmkit::commands::CmdGimbalPoint;
using swarmkit::commands::CmdGoto;
using swarmkit::commands::CmdGripper;
using swarmkit::commands::CmdHoldPosition;
using swarmkit::commands::CmdLand;
using swarmkit::commands::CmdPause;
using swarmkit::commands::CmdPauseMission;
using swarmkit::commands::CmdPhoto;
using swarmkit::commands::CmdPhotoIntervalStart;
using swarmkit::commands::CmdPhotoIntervalStop;
using swarmkit::commands::CmdRelay;
using swarmkit::commands::CmdResume;
using swarmkit::commands::CmdResumeMission;
using swarmkit::commands::CmdReturnHome;
using swarmkit::commands::CmdRoiClear;
using swarmkit::commands::CmdRoiLocation;
using swarmkit::commands::CmdServo;
using swarmkit::commands::CmdSetCurrentMissionItem;
using swarmkit::commands::CmdSetHome;
using swarmkit::commands::CmdSetMode;
using swarmkit::commands::CmdSetSpeed;
using swarmkit::commands::CmdSetWaypoint;
using swarmkit::commands::CmdSetYaw;
using swarmkit::commands::CmdStartMission;
using swarmkit::commands::CmdTakeoff;
using swarmkit::commands::CmdUploadMission;
using swarmkit::commands::CmdVelocity;
using swarmkit::commands::CmdVideoStart;
using swarmkit::commands::CmdVideoStop;
using swarmkit::commands::Command;
using swarmkit::commands::CommandEnvelope;
using swarmkit::commands::CommandPriority;
using swarmkit::commands::FlightCmd;
using swarmkit::commands::MissionCmd;
using swarmkit::commands::MissionItem;
using swarmkit::commands::NavCmd;
using swarmkit::commands::PayloadCmd;

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
constexpr std::string_view kDefaultAddressMode = "primary";
constexpr std::string_view kDefaultDurationMs = "0";
constexpr std::string_view kDefaultZero = "0";
constexpr std::string_view kDefaultCustomMode = "-1";
constexpr std::string_view kDefaultPriority = "supervisor";
constexpr std::uint16_t kMavCmdNavWaypoint = 16;
constexpr std::uint16_t kMavCmdNavTakeoff = 22;
constexpr std::uint16_t kMavCmdNavLand = 21;
constexpr std::uint16_t kMavCmdNavReturnToLaunch = 20;
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
           value == "command" || value == "lock" || value == "unlock" ||
           value == "watch-authority" || value == "swarm";
}

[[nodiscard]] bool IsOptionWithValue(std::string_view value) {
    return value == "--config" || value == "--drone" || value == "--rate" || value == "--alt" ||
           value == "--lat" || value == "--lon" || value == "--speed" || value == "--log-sink" ||
           value == "--log-file" || value == "--log-level" || value == "--ca-cert" ||
           value == "--client-cert" || value == "--client-key" || value == "--server-name" ||
           value == "--priority" || value == "--mode" || value == "--custom-mode" || value == "--ground" ||
           value == "--deg" || value == "--yaw" || value == "--vx" || value == "--vy" ||
           value == "--vz" || value == "--duration-ms" || value == "--ttl-ms" ||
           value == "--swarm-config" || value == "--address-mode" || value == "--file" ||
           value == "--seq" || value == "--first" || value == "--last" ||
           value == "--camera" || value == "--stream" || value == "--interval" ||
           value == "--count" || value == "--pitch" || value == "--roll" ||
           value == "--gimbal" || value == "--servo" || value == "--pwm" ||
           value == "--relay" || value == "--gripper";
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
                 "  command                Send a command with selected priority.\n"
                 "  lock                   Acquire command authority for one drone.\n"
                 "  unlock                 Release command authority for one drone.\n"
                 "  watch-authority        Watch authority changes for one drone.\n"
                 "  swarm                  Use a swarm config for broadcast/lock-all workflows.\n"
                 "\n"
                 "Global options:\n"
                 "  --config PATH          Load client config from YAML file\n"
                 "  --ca-cert PATH         mTLS CA certificate path\n"
                 "  --client-cert PATH     mTLS client certificate path\n"
                 "  --client-key PATH      mTLS client private key path\n"
                 "  --server-name NAME     TLS server name / authority override\n"
                 "  --priority LEVEL       operator|supervisor|override|emergency\n"
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
                 "  set-mode --mode guided|loiter|auto|rtl|land|brake [--custom-mode N]\n"
                 "  set-speed --ground MPS\n"
                 "  goto --lat L --lon L --alt M [--speed S] [--yaw DEG]\n"
                 "  pause | resume\n"
                 "  set-yaw --deg DEG [--rate DEG_S] [--relative]\n"
                 "  velocity --vx N --vy E --vz D [--duration-ms MS] [--body-frame]\n"
                 "  emergency force-disarm | emergency flight-terminate\n"
                 "  set-home current | set-home --lat L --lon L --alt M\n"
                 "  mission upload --file mission.yaml | mission clear | mission start\n"
                 "  mission pause | mission resume | mission set-current --seq N\n"
                 "  photo | photo-interval-start | photo-interval-stop | video-start | video-stop\n"
                 "  gimbal-point | roi-location | roi-clear | servo | relay | gripper\n"
                 "\n"
                 "Authority options:\n"
                 "  --ttl-ms MS            Lock TTL; 0 means no automatic expiry\n"
                 "\n"
                 "Swarm options:\n"
                 "  --swarm-config PATH    YAML fleet config for swarm commands\n"
                 "  --address-mode MODE    primary|local (default: primary)\n"
                 "\n"
                 "Examples:\n"
                 "  swarmkit-cli ping\n"
                 "  swarmkit-cli 192.168.1.10:50061 ping\n"
                 "  swarmkit-cli telemetry --drone uav-1 --rate 2\n"
                 "  swarmkit-cli command --drone uav-1 arm\n"
                 "  swarmkit-cli command --drone uav-1 takeoff --alt 30\n"
                 "  swarmkit-cli command --drone uav-1 goto --lat 40.18 --lon 44.51 --alt 50\n"
                 "  swarmkit-cli lock --drone uav-1 --ttl-ms 10000\n"
                 "  swarmkit-cli swarm --swarm-config testdata/swarm_config.yaml broadcast takeoff --alt 5\n";
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

[[nodiscard]] std::expected<int, std::string> ParseIntArg(std::string_view value,
                                                          std::string_view key) {
    try {
        return std::stoi(std::string(value));
    } catch (const std::exception& exc) {
        return std::unexpected("Invalid " + std::string(key) + " value '" + std::string(value) +
                               "': " + exc.what());
    }
}

[[nodiscard]] std::expected<CommandPriority, std::string> ParseCliPriority(int argc, char** argv) {
    const std::string priority_text =
        common::GetOptionValue(argc, argv, "--priority", kDefaultPriority);

    if (priority_text == "operator") {
        return CommandPriority::kOperator;
    }
    if (priority_text == "supervisor") {
        return CommandPriority::kSupervisor;
    }
    if (priority_text == "override") {
        return CommandPriority::kOverride;
    }
    if (priority_text == "emergency") {
        return CommandPriority::kEmergency;
    }
    return std::unexpected("Invalid --priority value '" + priority_text +
                           "': expected operator|supervisor|override|emergency");
}

[[nodiscard]] std::vector<std::string> FindCommandActions(int argc, char** argv) {
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
        return {};
    }

    std::vector<std::string> actions;
    for (int index = command_index + 1; index < argc; ++index) {
        const std::string_view kCurrentArg = argv[index];
        if (IsOptionWithValue(kCurrentArg)) {
            ++index;
            continue;
        }
        if (kCurrentArg.starts_with("-") || IsSubcommand(kCurrentArg)) {
            continue;
        }
        actions.emplace_back(kCurrentArg);
    }
    return actions;
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

[[nodiscard]] std::expected<Command, std::string> BuildSetModeCommand(int argc, char** argv) {
    CmdSetMode mode;
    mode.mode = common::GetOptionValue(argc, argv, "--mode");
    const auto custom_mode =
        ParseIntArg(common::GetOptionValue(argc, argv, "--custom-mode", kDefaultCustomMode),
                    "--custom-mode");
    if (!custom_mode.has_value()) {
        return std::unexpected(custom_mode.error());
    }
    mode.custom_mode = *custom_mode;
    if (mode.mode.empty() && mode.custom_mode < 0) {
        return std::unexpected("set-mode requires --mode or --custom-mode");
    }
    return FlightCmd{mode};
}

[[nodiscard]] std::expected<Command, std::string> BuildSetSpeedCommand(int argc, char** argv) {
    const auto speed =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--ground", kDefaultZero), "--ground");
    if (!speed.has_value()) {
        return std::unexpected(speed.error());
    }
    return NavCmd{CmdSetSpeed{*speed}};
}

[[nodiscard]] std::expected<Command, std::string> BuildGotoCommand(int argc, char** argv) {
    const auto lat =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat", kDefaultWaypointCoord), "--lat");
    const auto lon =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon", kDefaultWaypointCoord), "--lon");
    const auto alt =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--alt", kDefaultWaypointCoord), "--alt");
    const auto speed = ParseFloatArg(
        common::GetOptionValue(argc, argv, "--speed", kDefaultWaypointSpeed), "--speed");
    if (!lat.has_value()) {
        return std::unexpected(lat.error());
    }
    if (!lon.has_value()) {
        return std::unexpected(lon.error());
    }
    if (!alt.has_value()) {
        return std::unexpected(alt.error());
    }
    if (!speed.has_value()) {
        return std::unexpected(speed.error());
    }

    CmdGoto go_to;
    go_to.lat_deg = *lat;
    go_to.lon_deg = *lon;
    go_to.alt_m = *alt;
    go_to.speed_mps = *speed;
    if (const std::string yaw = common::GetOptionValue(argc, argv, "--yaw"); !yaw.empty()) {
        const auto parsed_yaw = ParseFloatArg(yaw, "--yaw");
        if (!parsed_yaw.has_value()) {
            return std::unexpected(parsed_yaw.error());
        }
        go_to.yaw_deg = *parsed_yaw;
        go_to.use_yaw = true;
    }
    return NavCmd{go_to};
}

[[nodiscard]] std::expected<Command, std::string> BuildSetYawCommand(int argc, char** argv) {
    const auto yaw =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--deg", kDefaultZero), "--deg");
    const auto rate =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--rate", kDefaultZero), "--rate");
    if (!yaw.has_value()) {
        return std::unexpected(yaw.error());
    }
    if (!rate.has_value()) {
        return std::unexpected(rate.error());
    }
    return NavCmd{CmdSetYaw{*yaw, *rate, common::HasFlag(argc, argv, "--relative")}};
}

[[nodiscard]] std::expected<Command, std::string> BuildVelocityCommand(int argc, char** argv) {
    const auto vx = ParseFloatArg(common::GetOptionValue(argc, argv, "--vx", kDefaultZero), "--vx");
    const auto vy = ParseFloatArg(common::GetOptionValue(argc, argv, "--vy", kDefaultZero), "--vy");
    const auto vz = ParseFloatArg(common::GetOptionValue(argc, argv, "--vz", kDefaultZero), "--vz");
    const auto duration =
        ParseIntArg(common::GetOptionValue(argc, argv, "--duration-ms", kDefaultDurationMs),
                    "--duration-ms");
    if (!vx.has_value()) {
        return std::unexpected(vx.error());
    }
    if (!vy.has_value()) {
        return std::unexpected(vy.error());
    }
    if (!vz.has_value()) {
        return std::unexpected(vz.error());
    }
    if (!duration.has_value()) {
        return std::unexpected(duration.error());
    }
    return NavCmd{CmdVelocity{*vx, *vy, *vz, *duration, common::HasFlag(argc, argv, "--body-frame")}};
}

[[nodiscard]] std::expected<Command, std::string> BuildSetHomeCommand(int argc, char** argv,
                                                                      const std::vector<std::string>& actions) {
    CmdSetHome home;
    home.use_current = actions.size() >= 2 && actions[1] == "current";
    if (!home.use_current) {
        const auto lat = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat"), "--lat");
        const auto lon = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon"), "--lon");
        const auto alt = ParseDoubleArg(common::GetOptionValue(argc, argv, "--alt"), "--alt");
        if (!lat.has_value()) {
            return std::unexpected(lat.error());
        }
        if (!lon.has_value()) {
            return std::unexpected(lon.error());
        }
        if (!alt.has_value()) {
            return std::unexpected(alt.error());
        }
        home.lat_deg = *lat;
        home.lon_deg = *lon;
        home.alt_m = *alt;
    }
    return NavCmd{home};
}

[[nodiscard]] std::uint16_t MissionCommandFromName(std::string_view name) {
    if (name == "takeoff") {
        return kMavCmdNavTakeoff;
    }
    if (name == "land") {
        return kMavCmdNavLand;
    }
    if (name == "rtl" || name == "return-home") {
        return kMavCmdNavReturnToLaunch;
    }
    return kMavCmdNavWaypoint;
}

[[nodiscard]] std::expected<Command, std::string> BuildMissionUploadCommand(const std::string& path) {
    if (path.empty()) {
        return std::unexpected("mission upload requires --file PATH or positional file path");
    }
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node items = root["items"] ? root["items"] : root["mission"];
        if (!items || !items.IsSequence()) {
            return std::unexpected("mission file must contain an items: sequence");
        }

        CmdUploadMission upload;
        for (const auto& node : items) {
            MissionItem item;
            const std::string command = node["command"] ? node["command"].as<std::string>() : "waypoint";
            item.command = node["mav_cmd"] ? node["mav_cmd"].as<std::uint16_t>()
                                           : MissionCommandFromName(command);
            item.lat_deg = node["lat"] ? node["lat"].as<double>() : 0.0;
            item.lon_deg = node["lon"] ? node["lon"].as<double>() : 0.0;
            item.alt_m = node["alt"] ? node["alt"].as<double>() : 0.0;
            item.param1 = node["param1"] ? node["param1"].as<float>() : 0.0F;
            item.param2 = node["param2"] ? node["param2"].as<float>() : 0.0F;
            item.param3 = node["param3"] ? node["param3"].as<float>() : 0.0F;
            item.param4 = node["param4"] ? node["param4"].as<float>() : 0.0F;
            item.current = node["current"] ? node["current"].as<bool>() : upload.items.empty();
            item.autocontinue = node["autocontinue"] ? node["autocontinue"].as<bool>() : true;
            upload.items.push_back(item);
        }
        return MissionCmd{std::move(upload)};
    } catch (const YAML::Exception& exc) {
        return std::unexpected("failed to load mission file '" + path + "': " + exc.what());
    }
}

[[nodiscard]] std::expected<Command, std::string> BuildPayloadCommand(
    const std::vector<std::string>& actions, int argc, char** argv) {
    const std::string& action = actions.front();
    const auto camera =
        ParseIntArg(common::GetOptionValue(argc, argv, "--camera", kDefaultZero), "--camera");
    if (!camera.has_value()) {
        return std::unexpected(camera.error());
    }

    if (action == "photo") {
        return PayloadCmd{CmdPhoto{*camera}};
    }
    if (action == "photo-interval-start") {
        const auto interval =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--interval", kDefaultZero),
                          "--interval");
        const auto count =
            ParseIntArg(common::GetOptionValue(argc, argv, "--count", kDefaultZero), "--count");
        if (!interval.has_value()) {
            return std::unexpected(interval.error());
        }
        if (!count.has_value()) {
            return std::unexpected(count.error());
        }
        return PayloadCmd{CmdPhotoIntervalStart{*interval, *count, *camera}};
    }
    if (action == "photo-interval-stop") {
        return PayloadCmd{CmdPhotoIntervalStop{*camera}};
    }
    if (action == "video-start" || action == "video-stop") {
        const auto stream =
            ParseIntArg(common::GetOptionValue(argc, argv, "--stream", kDefaultZero), "--stream");
        if (!stream.has_value()) {
            return std::unexpected(stream.error());
        }
        if (action == "video-start") {
            return PayloadCmd{CmdVideoStart{*stream, *camera}};
        }
        return PayloadCmd{CmdVideoStop{*stream, *camera}};
    }
    if (action == "gimbal-point") {
        const auto pitch =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--pitch", kDefaultZero), "--pitch");
        const auto roll =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--roll", kDefaultZero), "--roll");
        const auto yaw =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--yaw", kDefaultZero), "--yaw");
        if (!pitch.has_value()) {
            return std::unexpected(pitch.error());
        }
        if (!roll.has_value()) {
            return std::unexpected(roll.error());
        }
        if (!yaw.has_value()) {
            return std::unexpected(yaw.error());
        }
        return PayloadCmd{CmdGimbalPoint{*pitch, *roll, *yaw}};
    }
    if (action == "roi-location") {
        const auto lat = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat"), "--lat");
        const auto lon = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon"), "--lon");
        const auto alt = ParseDoubleArg(common::GetOptionValue(argc, argv, "--alt"), "--alt");
        const auto gimbal =
            ParseIntArg(common::GetOptionValue(argc, argv, "--gimbal", kDefaultZero), "--gimbal");
        if (!lat.has_value()) {
            return std::unexpected(lat.error());
        }
        if (!lon.has_value()) {
            return std::unexpected(lon.error());
        }
        if (!alt.has_value()) {
            return std::unexpected(alt.error());
        }
        if (!gimbal.has_value()) {
            return std::unexpected(gimbal.error());
        }
        return PayloadCmd{CmdRoiLocation{*lat, *lon, *alt, *gimbal}};
    }
    if (action == "roi-clear") {
        const auto gimbal =
            ParseIntArg(common::GetOptionValue(argc, argv, "--gimbal", kDefaultZero), "--gimbal");
        if (!gimbal.has_value()) {
            return std::unexpected(gimbal.error());
        }
        return PayloadCmd{CmdRoiClear{*gimbal}};
    }
    if (action == "servo") {
        const auto servo =
            ParseIntArg(common::GetOptionValue(argc, argv, "--servo", kDefaultZero), "--servo");
        const auto pwm =
            ParseIntArg(common::GetOptionValue(argc, argv, "--pwm", kDefaultZero), "--pwm");
        if (!servo.has_value()) {
            return std::unexpected(servo.error());
        }
        if (!pwm.has_value()) {
            return std::unexpected(pwm.error());
        }
        return PayloadCmd{CmdServo{*servo, *pwm}};
    }
    if (action == "relay") {
        const auto relay =
            ParseIntArg(common::GetOptionValue(argc, argv, "--relay", kDefaultZero), "--relay");
        if (!relay.has_value()) {
            return std::unexpected(relay.error());
        }
        return PayloadCmd{CmdRelay{*relay, common::HasFlag(argc, argv, "--on")}};
    }
    if (action == "gripper") {
        const auto gripper = ParseIntArg(
            common::GetOptionValue(argc, argv, "--gripper", kDefaultZero), "--gripper");
        if (!gripper.has_value()) {
            return std::unexpected(gripper.error());
        }
        return PayloadCmd{CmdGripper{*gripper, common::HasFlag(argc, argv, "--release")}};
    }
    return std::unexpected("Unknown payload action: " + action);
}

[[nodiscard]] std::expected<Command, std::string> BuildCommandFromActions(
    const std::vector<std::string>& actions, int argc, char** argv) {
    if (actions.empty()) {
        return std::unexpected("No action specified. See --help for options.");
    }
    const std::string& action = actions.front();

    if (action == "arm") {
        return FlightCmd{CmdArm{}};
    }
    if (action == "disarm") {
        return FlightCmd{CmdDisarm{}};
    }
    if (action == "land") {
        return FlightCmd{CmdLand{}};
    }
    if (action == "hold") {
        return NavCmd{CmdHoldPosition{}};
    }
    if (action == "return-home" || action == "rtl") {
        return NavCmd{CmdReturnHome{}};
    }
    if (action == "takeoff") {
        return BuildTakeoffCommand(argc, argv);
    }
    if (action == "waypoint") {
        return BuildWaypointCommand(argc, argv);
    }
    if (action == "set-mode") {
        return BuildSetModeCommand(argc, argv);
    }
    if (action == "set-speed") {
        return BuildSetSpeedCommand(argc, argv);
    }
    if (action == "goto") {
        return BuildGotoCommand(argc, argv);
    }
    if (action == "pause") {
        return NavCmd{CmdPause{}};
    }
    if (action == "resume") {
        return NavCmd{CmdResume{}};
    }
    if (action == "set-yaw") {
        return BuildSetYawCommand(argc, argv);
    }
    if (action == "velocity") {
        return BuildVelocityCommand(argc, argv);
    }
    if (action == "set-home") {
        return BuildSetHomeCommand(argc, argv, actions);
    }
    if (action == "emergency") {
        if (actions.size() < 2) {
            return std::unexpected("emergency requires force-disarm or flight-terminate");
        }
        if (actions[1] == "force-disarm") {
            return FlightCmd{CmdForceDisarm{}};
        }
        if (actions[1] == "flight-terminate") {
            return FlightCmd{CmdFlightTerminate{}};
        }
        return std::unexpected("Unknown emergency action: " + actions[1]);
    }
    if (action == "mission") {
        if (actions.size() < 2) {
            return std::unexpected("mission requires upload, clear, start, pause, resume, or set-current");
        }
        const std::string& mission_action = actions[1];
        if (mission_action == "upload") {
            const std::string file =
                common::GetOptionValue(argc, argv, "--file",
                                       actions.size() >= 3 ? actions[2] : std::string{});
            return BuildMissionUploadCommand(file);
        }
        if (mission_action == "clear") {
            return MissionCmd{CmdClearMission{}};
        }
        if (mission_action == "start") {
            const auto first =
                ParseIntArg(common::GetOptionValue(argc, argv, "--first", kDefaultZero), "--first");
            const auto last =
                ParseIntArg(common::GetOptionValue(argc, argv, "--last", kDefaultZero), "--last");
            if (!first.has_value()) {
                return std::unexpected(first.error());
            }
            if (!last.has_value()) {
                return std::unexpected(last.error());
            }
            return MissionCmd{CmdStartMission{*first, *last}};
        }
        if (mission_action == "pause") {
            return MissionCmd{CmdPauseMission{}};
        }
        if (mission_action == "resume") {
            return MissionCmd{CmdResumeMission{}};
        }
        if (mission_action == "set-current") {
            const auto seq =
                ParseIntArg(common::GetOptionValue(argc, argv, "--seq", kDefaultZero), "--seq");
            if (!seq.has_value()) {
                return std::unexpected(seq.error());
            }
            return MissionCmd{CmdSetCurrentMissionItem{*seq}};
        }
        return std::unexpected("Unknown mission action: " + mission_action);
    }
    if (action == "photo" || action == "photo-interval-start" ||
        action == "photo-interval-stop" || action == "video-start" ||
        action == "video-stop" || action == "gimbal-point" || action == "roi-location" ||
        action == "roi-clear" || action == "servo" || action == "relay" ||
        action == "gripper") {
        return BuildPayloadCommand(actions, argc, argv);
    }
    return std::unexpected("Unknown action: " + action);
}

[[nodiscard]] std::expected<Command, std::string> BuildCommandFromArgs(int argc, char** argv) {
    return BuildCommandFromActions(FindCommandActions(argc, argv), argc, argv);
}

[[nodiscard]] CommandEnvelope MakeCommandEnvelope(std::string_view drone_id, Command command,
                                                  CommandPriority priority =
                                                      CommandPriority::kSupervisor) {
    CommandEnvelope envelope;
    envelope.context.drone_id = std::string(drone_id);
    envelope.context.client_id = std::string(kCliClientId);
    envelope.context.priority = priority;
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
    const auto priority = ParseCliPriority(argc, argv);
    if (!priority.has_value()) {
        std::cerr << priority.error() << "\n";
        return std::unexpected(EXIT_FAILURE);
    }
    client_cfg.priority = *priority;
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
    const auto ttl = ParseIntArg(common::GetOptionValue(argc, argv, "--ttl-ms", kDefaultZero),
                                 "--ttl-ms");
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

void PrintSwarmResults(const std::unordered_map<std::string, swarmkit::client::CommandResult>& results) {
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

    Command command;
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
    return DispatchCommand(*kInvocation, *kClientConfig, client, argc, argv);
}

}  // namespace swarmkit::apps::cli
