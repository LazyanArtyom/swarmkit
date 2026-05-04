// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "nav_command_parser.h"

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"

namespace swarmkit::apps::cli::internal {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

[[nodiscard]] std::expected<Command, std::string> BuildWaypointCommand(int argc, char** argv) {
    const auto lat =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat", kDefaultWaypointCoord), "--lat");
    const auto lon =
        ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon", kDefaultWaypointCoord), "--lon");
    const auto alt =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--alt", kDefaultWaypointCoord), "--alt");
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
    return NavCmd{CmdSetWaypoint{
        .lat_deg = *lat,
        .lon_deg = *lon,
        .alt_m = *alt,
        .speed_mps = *speed,
    }};
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

}  // namespace

std::expected<Command, std::string> BuildNavCommand(const std::vector<std::string>& actions,
                                                    int argc, char** argv) {
    if (actions.empty()) {
        return std::unexpected("missing nav action");
    }
    const std::string& action = actions.front();
    if (action == "hold") {
        return NavCmd{CmdHoldPosition{}};
    }
    if (action == "return-home" || action == "rtl") {
        return NavCmd{CmdReturnHome{}};
    }
    if (action == "waypoint") {
        return BuildWaypointCommand(argc, argv);
    }
    if (action == "set-speed") {
        const auto speed =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--ground", kDefaultZero), "--ground");
        if (!speed.has_value()) {
            return std::unexpected(speed.error());
        }
        return NavCmd{CmdSetSpeed{*speed}};
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
    return std::unexpected("not a nav action");
}

}  // namespace swarmkit::apps::cli::internal
