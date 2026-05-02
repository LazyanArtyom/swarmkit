// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "command_builder.h"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <utility>

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"

namespace swarmkit::apps::cli::internal {
namespace {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

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
    const auto custom_mode = ParseIntArg(
        common::GetOptionValue(argc, argv, "--custom-mode", kDefaultCustomMode), "--custom-mode");
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
    return NavCmd{CmdSetYaw{
        .yaw_deg = *yaw,
        .rate_deg_s = *rate,
        .relative = common::HasFlag(argc, argv, "--relative"),
    }};
}

[[nodiscard]] std::expected<Command, std::string> BuildVelocityCommand(int argc, char** argv) {
    const auto velocity_x =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--vx", kDefaultZero), "--vx");
    const auto velocity_y =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--vy", kDefaultZero), "--vy");
    const auto velocity_z =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--vz", kDefaultZero), "--vz");
    const auto duration = ParseIntArg(
        common::GetOptionValue(argc, argv, "--duration-ms", kDefaultDurationMs), "--duration-ms");
    if (!velocity_x.has_value()) {
        return std::unexpected(velocity_x.error());
    }
    if (!velocity_y.has_value()) {
        return std::unexpected(velocity_y.error());
    }
    if (!velocity_z.has_value()) {
        return std::unexpected(velocity_z.error());
    }
    if (!duration.has_value()) {
        return std::unexpected(duration.error());
    }
    return NavCmd{CmdVelocity{
        .vx_mps = *velocity_x,
        .vy_mps = *velocity_y,
        .vz_mps = *velocity_z,
        .duration_ms = *duration,
        .body_frame = common::HasFlag(argc, argv, "--body-frame"),
    }};
}

[[nodiscard]] std::expected<Command, std::string> BuildSetHomeCommand(
    int argc, char** argv, const std::vector<std::string>& actions) {
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

[[nodiscard]] std::expected<Command, std::string> BuildMissionUploadCommand(
    const std::string& path) {
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
            const std::string command =
                node["command"] ? node["command"].as<std::string>() : "waypoint";
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
        const auto interval = ParseFloatArg(
            common::GetOptionValue(argc, argv, "--interval", kDefaultZero), "--interval");
        const auto count =
            ParseIntArg(common::GetOptionValue(argc, argv, "--count", kDefaultZero), "--count");
        if (!interval.has_value()) {
            return std::unexpected(interval.error());
        }
        if (!count.has_value()) {
            return std::unexpected(count.error());
        }
        return PayloadCmd{CmdPhotoIntervalStart{
            .interval_s = *interval,
            .count = *count,
            .camera_id = *camera,
        }};
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
            return PayloadCmd{CmdVideoStart{
                .stream_id = *stream,
                .camera_id = *camera,
            }};
        }
        return PayloadCmd{CmdVideoStop{
            .stream_id = *stream,
            .camera_id = *camera,
        }};
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
        return PayloadCmd{CmdGimbalPoint{
            .pitch_deg = *pitch,
            .roll_deg = *roll,
            .yaw_deg = *yaw,
        }};
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
        return PayloadCmd{CmdRoiLocation{
            .lat_deg = *lat,
            .lon_deg = *lon,
            .alt_m = *alt,
            .gimbal_id = *gimbal,
        }};
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
        return PayloadCmd{CmdServo{
            .servo = *servo,
            .pwm = *pwm,
        }};
    }
    if (action == "relay") {
        const auto relay =
            ParseIntArg(common::GetOptionValue(argc, argv, "--relay", kDefaultZero), "--relay");
        if (!relay.has_value()) {
            return std::unexpected(relay.error());
        }
        return PayloadCmd{CmdRelay{
            .relay = *relay,
            .enabled = common::HasFlag(argc, argv, "--on"),
        }};
    }
    if (action == "gripper") {
        const auto gripper =
            ParseIntArg(common::GetOptionValue(argc, argv, "--gripper", kDefaultZero), "--gripper");
        if (!gripper.has_value()) {
            return std::unexpected(gripper.error());
        }
        return PayloadCmd{CmdGripper{
            .gripper = *gripper,
            .release = common::HasFlag(argc, argv, "--release"),
        }};
    }
    return std::unexpected("Unknown payload action: " + action);
}

}  // namespace

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

[[nodiscard]] std::expected<commands::Command, std::string> BuildCommandFromActions(
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
            return std::unexpected(
                "mission requires upload, clear, start, pause, resume, or set-current");
        }
        const std::string& mission_action = actions[1];
        if (mission_action == "upload") {
            const std::string file = common::GetOptionValue(
                argc, argv, "--file", actions.size() >= 3 ? actions[2] : std::string{});
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
            return MissionCmd{CmdStartMission{
                .first_item = *first,
                .last_item = *last,
            }};
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
    if (action == "photo" || action == "photo-interval-start" || action == "photo-interval-stop" ||
        action == "video-start" || action == "video-stop" || action == "gimbal-point" ||
        action == "roi-location" || action == "roi-clear" || action == "servo" ||
        action == "relay" || action == "gripper") {
        return BuildPayloadCommand(actions, argc, argv);
    }
    return std::unexpected("Unknown action: " + action);
}

[[nodiscard]] std::expected<commands::Command, std::string> BuildCommandFromArgs(int argc,
                                                                                 char** argv) {
    return BuildCommandFromActions(FindCommandActions(argc, argv), argc, argv);
}

[[nodiscard]] commands::CommandEnvelope MakeCommandEnvelope(std::string_view drone_id,
                                                            commands::Command command,
                                                            commands::CommandPriority priority) {
    commands::CommandEnvelope envelope;
    envelope.context.drone_id = std::string(drone_id);
    envelope.context.client_id = std::string(kCliClientId);
    envelope.context.priority = priority;
    envelope.command = std::move(command);
    return envelope;
}

}  // namespace swarmkit::apps::cli::internal
