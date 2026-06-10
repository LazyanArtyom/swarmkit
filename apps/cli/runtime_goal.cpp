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
    std::string target_frame = common::GetOptionValue(argc, argv, "--frame", "global");
    target_frame = ToLowerAscii(std::move(target_frame));
    const bool use_local_target =
        target_frame == "local-ned" || !common::GetOptionValue(argc, argv, "--x").empty() ||
        !common::GetOptionValue(argc, argv, "--y").empty() ||
        !common::GetOptionValue(argc, argv, "--z").empty();
    if (use_local_target) {
        target_frame = "local-ned";
    }
    if (target_frame != "global" && target_frame != "local-ned") {
        return std::unexpected("--frame must be global or local-ned");
    }

    swarmkit::client::GeoPoint target;
    swarmkit::client::LocalPoint local_target;
    if (use_local_target) {
        const auto x = ParseDoubleArg(common::GetOptionValue(argc, argv, "--x"), "--x");
        const auto y = ParseDoubleArg(common::GetOptionValue(argc, argv, "--y"), "--y");
        const auto z = ParseDoubleArg(common::GetOptionValue(argc, argv, "--z"), "--z");
        if (!x.has_value()) {
            return std::unexpected(x.error());
        }
        if (!y.has_value()) {
            return std::unexpected(y.error());
        }
        if (!z.has_value()) {
            return std::unexpected(z.error());
        }
        local_target = swarmkit::client::LocalPoint{.x_m = *x, .y_m = *y, .z_m = *z};
    } else {
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
        target = swarmkit::client::GeoPoint{.lat_deg = *lat, .lon_deg = *lon, .alt_m = *alt};
    }

    auto revision = static_cast<std::uint64_t>(NowUnixMs());
    if (const std::string revision_value = common::GetOptionValue(argc, argv, "--revision");
        !revision_value.empty()) {
        const auto parsed_revision = ParseIntArg(revision_value, "--revision");
        if (!parsed_revision.has_value()) {
            return std::unexpected(parsed_revision.error());
        }
        if (*parsed_revision < 0) {
            return std::unexpected("--revision must be >= 0");
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
        .local_target = local_target,
        .use_local_target = use_local_target,
        .target_frame = target_frame,
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
        const auto result = client.SetActiveGoal(*goal);
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
        const auto result = client.CancelGoal(std::string(drone_id),
                                              common::GetOptionValue(argc, argv, "--goal-id"));
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
        if (status.error.code != swarmkit::client::RpcStatusCode::kOk) {
            std::cerr << "Goal get FAILED: " << status.message << "\n";
            return EXIT_FAILURE;
        }
        if (!status.has_goal) {
            std::cout << "No active goal for drone=" << drone_id << "\n";
            return EXIT_SUCCESS;
        }
        std::cout << "Active goal\n"
                  << "  drone_id            : " << status.goal.drone_id << "\n"
                  << "  goal_id             : " << status.goal.goal_id << "\n"
                  << "  revision            : " << status.goal.revision << "\n"
                  << "  status              : " << GoalStatusName(status.status) << "\n"
                  << "  target_frame        : " << status.goal.target_frame << "\n"
                  << "  lat                 : " << status.goal.target.lat_deg << "\n"
                  << "  lon                 : " << status.goal.target.lon_deg << "\n"
                  << "  alt_m               : " << status.goal.target.alt_m << "\n"
                  << "  local_x_m           : " << status.goal.local_target.x_m << "\n"
                  << "  local_y_m           : " << status.goal.local_target.y_m << "\n"
                  << "  local_z_m           : " << status.goal.local_target.z_m << "\n"
                  << "  speed_mps           : " << status.goal.speed_mps << "\n"
                  << "  accept_radius_m     : " << status.goal.acceptance_radius_m << "\n"
                  << "  deviation_radius_m  : " << status.goal.deviation_radius_m << "\n"
                  << "  computed_timeout_ms : " << status.computed_timeout_ms << "\n";
        if (!status.goal.labels.empty()) {
            std::cout << "  labels              :\n";
            for (const auto& [key, value] : status.goal.labels) {
                std::cout << "    " << key << "=" << value << "\n";
            }
        }
        return EXIT_SUCCESS;
    }

    std::cerr << "Unknown goal action: " << actions[0] << "\n";
    return EXIT_FAILURE;
}

}  // namespace swarmkit::apps::cli::internal
