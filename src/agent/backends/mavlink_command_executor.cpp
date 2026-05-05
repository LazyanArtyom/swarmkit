// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_command_executor.h"

namespace swarmkit::agent::mavlink {

BackendCapabilities MavlinkCommandExecutor::Capabilities(const MavlinkBackendConfig& config) {
    return {
        .backend_name = "mavlink",
        .protocol = "mavlink2",
        .vehicle_class = config.autopilot_profile == MavlinkAutopilotProfile::kArdupilotPlane
                             ? "fixed-wing"
                             : "multirotor",
        .supports_mission_upload = true,
        .supports_payload_control = true,
        .supports_velocity_control = true,
        .supports_flight_termination = config.allow_flight_termination,
        .supports_backend_commands = true,
        .supports_trajectory_upload = true,
        .autopilot_type = std::string(ToString(config.autopilot_profile)),
        .supported_modes = SupportedModes(config.autopilot_profile),
        .supported_commands =
            {
                "arm",
                "disarm",
                "takeoff",
                "land",
                "return-home",
                "hold",
                "set-mode",
                "set-speed",
                "goto",
                "set-yaw",
                "velocity",
                "pause",
                "resume",
                "set-home",
                "mission-upload",
                "mission-clear",
                "mission-start",
                "mission-pause",
                "mission-resume",
                "set-current-mission-item",
                "backend-command",
            },
        .supported_mission_items =
            {
                "waypoint",
                "takeoff",
                "land",
                "loiter",
                "delay",
            },
        .supported_payloads =
            {
                "photo",
                "photo-interval",
                "video",
                "gimbal",
                "roi",
                "servo",
                "relay",
                "gripper",
            },
        .supported_telemetry_fields =
            {
                "lat_deg",
                "lon_deg",
                "rel_alt_m",
                "abs_alt_m",
                "velocity",
                "attitude",
                "battery_percent",
                "mode",
                "armed",
                "failsafe",
                "gps",
            },
        .backend_command_names = {"mavlink.command-long"},
        .supported_payload_action_namespaces = {"mavlink"},
        .supported_payload_action_names = {"command-long"},
        .payload_timing_precision_ms = 50,
        .supports_payload_scheduling = true,
    };
}

core::Result MavlinkCommandExecutor::ResolveCustomMode(const MavlinkBackendConfig& config,
                                                       const commands::CmdSetMode& mode,
                                                       int* custom_mode) {
    if (custom_mode == nullptr) {
        return core::Result::Failed("custom_mode output pointer is null");
    }

    if (mode.custom_mode >= 0) {
        *custom_mode = mode.custom_mode;
        return core::Result::Success();
    }

    const auto mapped_mode = config.autopilot_profile == MavlinkAutopilotProfile::kArdupilotPlane
                                 ? ArduPlaneModeFromName(mode.mode)
                                 : ArduCopterModeFromName(mode.mode);
    if (!mapped_mode.has_value()) {
        return core::Result::Rejected("unknown mode '" + mode.mode + "' for autopilot " +
                                      std::string(ToString(config.autopilot_profile)) +
                                      "; use a known mode or --custom-mode");
    }

    *custom_mode = *mapped_mode;
    return core::Result::Success();
}

}  // namespace swarmkit::agent::mavlink
