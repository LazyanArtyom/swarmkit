// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mission_command_parser.h"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <utility>

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"

namespace swarmkit::apps::cli::internal {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

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

}  // namespace

std::expected<Command, std::string> BuildMissionCommand(const std::vector<std::string>& actions,
                                                        int argc, char** argv) {
    if (actions.size() < 2 || actions.front() != "mission") {
        return std::unexpected("not a mission action");
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

}  // namespace swarmkit::apps::cli::internal
