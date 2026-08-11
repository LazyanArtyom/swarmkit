// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_command_executor.h"

namespace swarmkit::agent::mavlink {

core::BackendCapabilities MavlinkCommandExecutor::Capabilities(const MavlinkBackendConfig& config) {
    return {
        .backend_name = "mavlink",
        .protocol = "mavlink2",
        .vehicle_class = config.autopilot_profile == MavlinkAutopilotProfile::kArdupilotPlane
                             ? "fixed-wing"
                             : "multirotor",
        .supports_payload_control = false,
        .supports_velocity_control = true,
        .supports_flight_termination = config.allow_flight_termination,
        .supports_backend_commands = true,
        .autopilot_type = std::string(ToString(config.autopilot_profile)),
        .supported_modes = SupportedModes(config.autopilot_profile),
        .supported_commands =
            {
                "arm",
                "force-arm",
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
                "set-home",
                "backend-command",
            },
        // Peripheral presence cannot be inferred from generic MAVLink routing.
        // Payload encoders remain available as experimental commands, but are
        // not advertised until a vehicle-specific capability source exists.
        .supported_payloads = {},
        .backend_command_names = {"mavlink.command-long"},
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
