// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "flight_command_parser.h"

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"

namespace swarmkit::apps::cli::internal {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

namespace {

[[nodiscard]] std::expected<Command, std::string> BuildTakeoffCommand(int argc, char** argv) {
    const auto altitude =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--alt", kDefaultTakeoffAlt), "--alt");
    if (!altitude.has_value()) {
        return std::unexpected(altitude.error());
    }

    return FlightCmd{CmdTakeoff{.alt_m = *altitude}};
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

}  // namespace

std::expected<Command, std::string> BuildFlightCommand(const std::vector<std::string>& actions,
                                                       int argc, char** argv) {
    if (actions.empty()) {
        return std::unexpected("missing flight action");
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
    if (action == "takeoff") {
        return BuildTakeoffCommand(argc, argv);
    }
    if (action == "set-mode") {
        return BuildSetModeCommand(argc, argv);
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
    return std::unexpected("not a flight action");
}

}  // namespace swarmkit::apps::cli::internal
