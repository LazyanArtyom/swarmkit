// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "runtime_goal.h"

#include "runtime_common.h"

namespace swarmkit::apps::cli::internal {

[[nodiscard]] std::expected<std::unordered_map<std::string, std::string>, std::string>
ParseKeyValueLabels(int argc, char** argv, std::string_view option_name) {
    std::unordered_map<std::string, std::string> labels;
    for (int index = 1; index < argc; ++index) {
        if (std::string_view(argv[index]) != option_name) {
            continue;
        }
        if (index + 1 >= argc) {
            return std::unexpected(std::string(option_name) + " requires KEY=VALUE");
        }
        const std::string entry = argv[++index];
        const std::size_t separator = entry.find('=');
        if (separator == std::string::npos || separator == 0) {
            return std::unexpected(std::string(option_name) + " requires KEY=VALUE");
        }
        labels[entry.substr(0, separator)] = entry.substr(separator + 1);
    }
    return labels;
}

[[nodiscard]] std::expected<swarmkit::client::ActiveGoal, std::string> ParseActiveGoal(
    std::string_view drone_id, int argc, char** argv) {
    const std::string goal_id = common::GetOptionValue(argc, argv, "--goal-id");
    if (goal_id.empty()) {
        return std::unexpected("goal set requires --goal-id ID");
    }
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
    const swarmkit::client::GeoPoint target{
        .lat_deg = *lat,
        .lon_deg = *lon,
        .alt_m = *alt,
    };

    auto revision = static_cast<std::uint64_t>(NowUnixMs());
    if (const std::string revision_value = common::GetOptionValue(argc, argv, "--revision");
        !revision_value.empty()) {
        const auto parsed_revision = ParseIntArg(revision_value, "--revision");
        if (!parsed_revision.has_value()) {
            return std::unexpected(parsed_revision.error());
        }
        if (*parsed_revision <= 0) {
            return std::unexpected("--revision must be > 0");
        }
        revision = static_cast<std::uint64_t>(*parsed_revision);
    }

    const auto speed =
        ParseFloatArg(common::GetOptionValue(argc, argv, "--speed", kDefaultZero), "--speed");
    const auto acceptance_radius = ParseFloatArg(
        common::GetOptionValue(argc, argv, "--accept-radius", "2"), "--accept-radius");
    const auto deviation_radius = ParseFloatArg(
        common::GetOptionValue(argc, argv, "--deviation-radius", "8"), "--deviation-radius");
    const auto timeout_ms = ParseIntArg(
        common::GetOptionValue(argc, argv, "--timeout-ms", kDefaultZero), "--timeout-ms");
    if (!speed.has_value()) {
        return std::unexpected(speed.error());
    }
    if (!acceptance_radius.has_value()) {
        return std::unexpected(acceptance_radius.error());
    }
    if (!deviation_radius.has_value()) {
        return std::unexpected(deviation_radius.error());
    }
    if (!timeout_ms.has_value()) {
        return std::unexpected(timeout_ms.error());
    }
    auto labels = ParseKeyValueLabels(argc, argv, "--label");
    if (!labels.has_value()) {
        return std::unexpected(labels.error());
    }

    return swarmkit::client::ActiveGoal{
        .drone_id = std::string(drone_id),
        .goal_id = goal_id,
        .revision = revision,
        .target = target,
        .speed_mps = *speed,
        .acceptance_radius_m = *acceptance_radius,
        .deviation_radius_m = *deviation_radius,
        .timeout_ms = *timeout_ms,
        .labels = std::move(*labels),
    };
}

int RunGoal(Client& client, std::string_view drone_id, int argc, char** argv) {
    const std::vector<std::string> actions = FindActionsAfterCommand(argc, argv, "goal");
    if (actions.empty()) {
        std::cerr << "goal requires set, cancel, or get\n";
        return EXIT_FAILURE;
    }

    if (actions[0] == "set") {
        const auto goal = ParseActiveGoal(drone_id, argc, argv);
        if (!goal.has_value()) {
            std::cerr << goal.error() << "\n";
            return EXIT_FAILURE;
        }
        const auto result = client.SetActiveGoal({.goal = *goal});
        if (!result.ok) {
            std::cerr << "Goal set FAILED: " << result.message;
            if (!result.correlation_id.empty()) {
                std::cerr << " [corr=" << result.correlation_id << "]";
            }
            std::cerr << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "Goal set OK"
                  << " goal_id=" << result.goal.goal_id << " revision=" << result.goal.revision
                  << " computed_timeout_ms=" << result.computed_timeout_ms;
        if (!result.goal.labels.empty()) {
            std::cout << " labels=" << result.goal.labels.size();
        }
        std::cout << "\n";
        return EXIT_SUCCESS;
    }

    if (actions[0] == "cancel") {
        const auto current = client.GetActiveGoal(std::string(drone_id));
        if (!current.ok || !current.active_goal.has_value()) {
            std::cerr << "Goal cancel FAILED: "
                      << (current.message.empty() ? "no active goal" : current.message) << "\n";
            return EXIT_FAILURE;
        }
        const std::string requested_goal_id = common::GetOptionValue(argc, argv, "--goal-id");
        if (!requested_goal_id.empty() &&
            requested_goal_id != current.active_goal->execution_handle.goal_id) {
            std::cerr << "Goal cancel FAILED: active goal identity does not match --goal-id\n";
            return EXIT_FAILURE;
        }
        const auto result = client.CancelGoal(current.active_goal->execution_handle);
        if (!result.ok) {
            std::cerr << "Goal cancel FAILED: " << result.message;
            if (!result.correlation_id.empty()) {
                std::cerr << " [corr=" << result.correlation_id << "]";
            }
            std::cerr << "\n";
            return EXIT_FAILURE;
        }
        std::cout << "Goal cancel OK" << (result.message.empty() ? "" : ": " + result.message)
                  << "\n";
        return EXIT_SUCCESS;
    }

    if (actions[0] == "get") {
        const auto status = client.GetActiveGoal(std::string(drone_id));
        if (!status.ok) {
            std::cerr << "Goal get FAILED: " << status.message << "\n";
            return EXIT_FAILURE;
        }
        if (!status.active_goal.has_value()) {
            std::cout << "No active goal for drone=" << drone_id << "\n";
            return EXIT_SUCCESS;
        }
        const auto& active = *status.active_goal;
        std::cout << "Active goal\n"
                  << "  drone_id            : " << active.goal.drone_id << "\n"
                  << "  goal_id             : " << active.goal.goal_id << "\n"
                  << "  revision            : " << active.goal.revision << "\n"
                  << "  status              : " << GoalStatusName(active.status) << "\n"
                  << "  lat                 : " << active.goal.target.lat_deg << "\n"
                  << "  lon                 : " << active.goal.target.lon_deg << "\n"
                  << "  alt_m               : " << active.goal.target.alt_m << "\n"
                  << "  speed_mps           : " << active.goal.speed_mps << "\n"
                  << "  accept_radius_m     : " << active.goal.acceptance_radius_m << "\n"
                  << "  deviation_radius_m  : " << active.goal.deviation_radius_m << "\n"
                  << "  computed_timeout_ms : " << active.computed_timeout_ms << "\n";
        if (!active.goal.labels.empty()) {
            std::cout << "  labels              :\n";
            for (const auto& [key, value] : active.goal.labels) {
                std::cout << "    " << key << "=" << value << "\n";
            }
        }
        return EXIT_SUCCESS;
    }

    std::cerr << "Unknown goal action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}

}  // namespace swarmkit::apps::cli::internal
