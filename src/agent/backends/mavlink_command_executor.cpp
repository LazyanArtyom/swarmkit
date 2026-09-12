// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_command_executor.h"

namespace swarmkit::agent::mavlink {

core::BackendCapabilities MavlinkCommandExecutor::Capabilities(const MavlinkBackendConfig& config) {
    core::BackendCapabilities capabilities{
        .backend_name = "mavlink",
        .protocol = "mavlink2",
        .vehicle_class = config.autopilot_profile == MavlinkAutopilotProfile::kArdupilotPlane
                             ? "fixed-wing"
                             : "multirotor",
        .supports_payload_control = false,
        .supports_velocity_control = true,
        .supports_flight_termination = config.allow_flight_termination,
        .supports_backend_commands = false,
        .autopilot_type = std::string(ToString(config.autopilot_profile)),
        .supported_modes = SupportedModes(config.autopilot_profile),
        .supported_commands =
            {
                "arm",
                "force-arm",
                "disarm",
                "force-disarm",
                "takeoff",
                "land",
                "return-home",
                "hold",
                "set-mode",
                "set-speed",
                "goto",
                "pause",
                "resume",
                "set-yaw",
                "velocity",
                "set-home",
            },
        // Peripheral presence cannot be inferred from generic MAVLink routing.
        // Add payload support only with vehicle-specific capability discovery.
        .supported_payloads = {},
        .backend_command_names = {},
        .evidence =
            {
                .source_timestamp = core::CapabilitySupport::kSupported,
                .source_clock_domains = {core::ClockDomain::kVehicleBoot,
                                         core::ClockDomain::kUnixEpoch},
                .position_estimate = core::CapabilitySupport::kSupported,
                .horizontal_position_uncertainty = core::CapabilitySupport::kSupported,
                .vertical_position_uncertainty = core::CapabilitySupport::kSupported,
                .horizontal_velocity = core::CapabilitySupport::kSupported,
                .vertical_velocity = core::CapabilitySupport::kSupported,
                .horizontal_velocity_uncertainty = core::CapabilitySupport::kUnsupported,
                .vertical_velocity_uncertainty = core::CapabilitySupport::kUnsupported,
                .speed_uncertainty = core::CapabilitySupport::kSupported,
                .uncertainty_semantics = core::CapabilitySupport::kSupported,
                .estimator_health = core::CapabilitySupport::kSupported,
                .failsafe_state = core::CapabilitySupport::kSupported,
            },
    };
    if (config.allow_flight_termination) {
        capabilities.supported_commands.emplace_back("flight-terminate");
    }
    return capabilities;
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

MavlinkCommandLongSpec MavlinkCommandExecutor::ArmDisarmCommand(bool arm, bool force) {
    MavlinkCommandLongSpec spec;
    spec.command = MAV_CMD_COMPONENT_ARM_DISARM;
    spec.params[0] = arm ? 1.0F : 0.0F;
    spec.params[1] = force ? kMavlinkForceArmDisarmMagic : 0.0F;
    return spec;
}

}  // namespace swarmkit::agent::mavlink
