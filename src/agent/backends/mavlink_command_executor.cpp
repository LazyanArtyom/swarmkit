// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_command_executor.h"

namespace swarmkit::agent::mavlink {

BackendCapabilities MavlinkCommandExecutor::Capabilities(const MavlinkBackendConfig& config) {
    return {
        .supports_mission_upload = true,
        .supports_payload_control = true,
        .supports_velocity_control = true,
        .supports_flight_termination = config.allow_flight_termination,
        .autopilot_type = std::string(ToString(config.autopilot_profile)),
        .supported_modes = SupportedModes(config.autopilot_profile),
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
