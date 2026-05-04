// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "command_builder.h"

#include <utility>

#include "common/arg_utils.h"
#include "constants.h"
#include "flight_command_parser.h"
#include "mission_command_parser.h"
#include "nav_command_parser.h"
#include "options.h"
#include "payload_command_parser.h"

namespace swarmkit::apps::cli::internal {

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

    if (action == "arm" || action == "disarm" || action == "land" || action == "takeoff" ||
        action == "set-mode" || action == "emergency") {
        return BuildFlightCommand(actions, argc, argv);
    }
    if (action == "hold" || action == "return-home" || action == "rtl" || action == "waypoint" ||
        action == "set-speed" || action == "goto" || action == "pause" || action == "resume" ||
        action == "set-yaw" || action == "velocity" || action == "set-home") {
        return BuildNavCommand(actions, argc, argv);
    }
    if (action == "mission") {
        if (actions.size() < 2) {
            return std::unexpected(
                "mission requires upload, clear, start, pause, resume, or set-current");
        }
        return BuildMissionCommand(actions, argc, argv);
    }
    if (action == "photo" || action == "photo-interval-start" || action == "photo-interval-stop" ||
        action == "video-start" || action == "video-stop" || action == "gimbal-point" ||
        action == "roi-location" || action == "roi-clear" || action == "servo" ||
        action == "relay" || action == "gripper") {
        return BuildPayloadCommand(actions, argc, argv);
    }
    return std::unexpected("Unknown action: " + action);
}

[[nodiscard]] std::expected<commands::Command, std::string> BuildCommandFromTokens(
    const std::vector<std::string>& tokens) {
    if (tokens.empty()) {
        return std::unexpected("command sequence step requires non-empty args");
    }

    std::vector<std::string> storage;
    storage.reserve(tokens.size() + 2);
    storage.emplace_back("swarmkit-cli");
    storage.emplace_back("command");
    storage.insert(storage.end(), tokens.begin(), tokens.end());

    std::vector<char*> argv;
    argv.reserve(storage.size());
    for (std::string& arg : storage) {
        argv.push_back(arg.data());
    }

    std::vector<std::string> actions;
    for (std::size_t index = 0; index < tokens.size(); ++index) {
        const std::string_view arg = tokens[index];
        if (IsOptionWithValue(arg)) {
            ++index;
            continue;
        }
        if (!arg.starts_with("-")) {
            actions.emplace_back(arg);
        }
    }

    return BuildCommandFromActions(actions, static_cast<int>(argv.size()), argv.data());
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
