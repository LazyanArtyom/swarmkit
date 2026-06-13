// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "command_preconditions.h"

#include <cmath>
#include <string>
#include <string_view>
#include <variant>

#include "swarmkit/core/overloaded.h"

namespace swarmkit::agent {
namespace {

constexpr float kTakeoffAltitudeToleranceM = 0.5F;
constexpr float kLandedRelativeAltitudeToleranceM = 0.75F;

[[nodiscard]] CommandPreconditionDecision Execute() {
    return {};
}

[[nodiscard]] CommandPreconditionDecision AlreadySatisfied(std::string message) {
    return {.action = CommandPreconditionAction::kAlreadySatisfied,
            .result = core::Result::Success(std::move(message))};
}

[[nodiscard]] CommandPreconditionDecision Reject(std::string message) {
    return {.action = CommandPreconditionAction::kReject,
            .result = core::Result::Rejected(std::move(message))};
}

[[nodiscard]] bool HasVehicleState(const BackendHealth& health) {
    return health.last_heartbeat_unix_ms != 0 || health.last_telemetry_unix_ms != 0;
}

[[nodiscard]] bool RelativeAltitudeConfirmsLanded(const BackendHealth& health) {
    return health.has_relative_altitude &&
           std::fabs(health.relative_alt_m) <= kLandedRelativeAltitudeToleranceM;
}

[[nodiscard]] CommandPreconditionDecision RequireAutonomousReadiness(
    const BackendHealth& health, std::string_view operation, bool allow_unsafe_bench_commands,
    AutonomousReadinessRequirements requirements = {}) {
    const core::Result result =
        ValidateAutonomousReadiness(health, operation, allow_unsafe_bench_commands, requirements);
    return result.IsOk() ? Execute() : Reject(result.message);
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdArm& /*unused*/, const BackendHealth& health,
    bool /*allow_unsafe_bench_commands*/) {
    if (health.armed) {
        return AlreadySatisfied("arm already satisfied: vehicle is already armed");
    }
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdDisarm& /*unused*/, const BackendHealth& health,
    bool /*allow_unsafe_bench_commands*/) {
    if (!health.armed) {
        return AlreadySatisfied("disarm already satisfied: vehicle is already disarmed");
    }
    if (!health.landed && !RelativeAltitudeConfirmsLanded(health)) {
        std::string detail =
            "normal disarm refused while vehicle appears airborne; use emergency force-disarm";
        if (health.has_relative_altitude) {
            detail += " (relative_alt_m=" + std::to_string(health.relative_alt_m) + ")";
        } else {
            detail += " (relative altitude unknown)";
        }
        return Reject(std::move(detail));
    }
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(const commands::CmdTakeoff& takeoff,
                                                                const BackendHealth& health,
                                                                bool allow_unsafe_bench_commands) {
    if (!health.armed) {
        return RequireAutonomousReadiness(health, "takeoff", allow_unsafe_bench_commands);
    }
    if (health.has_relative_altitude &&
        health.relative_alt_m + kTakeoffAltitudeToleranceM >= static_cast<float>(takeoff.alt_m)) {
        return AlreadySatisfied(
            "takeoff already satisfied: vehicle is at or above target altitude");
    }
    if (!health.landed && health.has_relative_altitude && !RelativeAltitudeConfirmsLanded(health)) {
        return AlreadySatisfied("takeoff already satisfied: vehicle is already airborne");
    }
    return RequireAutonomousReadiness(health, "takeoff", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdLand& /*unused*/, const BackendHealth& health,
    bool /*allow_unsafe_bench_commands*/) {
    if (!health.armed && RelativeAltitudeConfirmsLanded(health)) {
        return AlreadySatisfied(
            "land already satisfied: vehicle is disarmed and relative altitude is near ground");
    }
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdForceDisarm& /*unused*/, const BackendHealth& /*unused*/,
    bool /*allow_unsafe_bench_commands*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdForceArm& /*unused*/, const BackendHealth& /*unused*/,
    bool /*allow_unsafe_bench_commands*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdFlightTerminate& /*unused*/, const BackendHealth& /*unused*/,
    bool /*allow_unsafe_bench_commands*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdSetMode& /*unused*/, const BackendHealth& /*unused*/,
    bool /*allow_unsafe_bench_commands*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(
    const commands::CmdSetWaypoint& /*unused*/, const BackendHealth& health,
    bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "waypoint", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(
    const commands::CmdReturnHome& /*unused*/, const BackendHealth& health,
    bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "return-home", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(
    const commands::CmdHoldPosition& /*unused*/, const BackendHealth& health,
    bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "hold", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(const commands::CmdGoto& /*unused*/,
                                                             const BackendHealth& health,
                                                             bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "goto", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(const commands::CmdPause& /*unused*/,
                                                             const BackendHealth& health,
                                                             bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "pause", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(const commands::CmdResume& /*unused*/,
                                                             const BackendHealth& health,
                                                             bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "resume", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(const commands::CmdSetYaw& /*unused*/,
                                                             const BackendHealth& health,
                                                             bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "set-yaw", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(
    const commands::CmdVelocity& /*unused*/, const BackendHealth& health,
    bool allow_unsafe_bench_commands) {
    return RequireAutonomousReadiness(health, "velocity", allow_unsafe_bench_commands);
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(
    const commands::CmdSetSpeed& /*unused*/, const BackendHealth& /*unused*/,
    bool /*allow_unsafe_bench_commands*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateNavCommand(const commands::CmdSetHome& /*unused*/,
                                                             const BackendHealth& /*unused*/,
                                                             bool /*allow_unsafe_bench_commands*/) {
    return Execute();
}

}  // namespace

core::Result ValidateAutonomousReadiness(const BackendHealth& health, std::string_view operation,
                                         bool allow_unsafe_bench_commands,
                                         AutonomousReadinessRequirements requirements) {
    if (allow_unsafe_bench_commands) {
        return core::Result::Success();
    }
    const std::string prefix = std::string(operation) + " rejected: ";
    if (!health.ready) {
        return core::Result::Rejected(prefix + "vehicle backend is not ready (" + health.message +
                                      ")");
    }
    if (health.last_heartbeat_unix_ms == 0 || health.last_telemetry_unix_ms == 0) {
        return core::Result::Rejected(prefix + "fresh vehicle telemetry is required");
    }
    if (requirements.require_armed && !health.armed) {
        return core::Result::Rejected(prefix + "vehicle must be armed");
    }
    if (requirements.require_gps && !health.gps_ok) {
        return core::Result::Rejected(prefix + "healthy GPS position is required");
    }
    if (requirements.require_ekf && !health.ekf_ok) {
        return core::Result::Rejected(prefix + "healthy EKF state is required");
    }
    return core::Result::Success();
}

CommandPreconditionDecision EvaluateCommandPreconditions(const CommandEnvelope& envelope,
                                                         const BackendHealth& health,
                                                         bool allow_unsafe_bench_commands) {
    if (!HasVehicleState(health)) {
        return Execute();
    }

    return std::visit(
        core::Overloaded{
            [&](const commands::FlightCmd& flight) {
                return std::visit(
                    [&](const auto& command) {
                        return EvaluateFlightCommand(command, health, allow_unsafe_bench_commands);
                    },
                    flight);
            },
            [&](const commands::NavCmd& nav) {
                return std::visit(
                    [&](const auto& command) {
                        return EvaluateNavCommand(command, health, allow_unsafe_bench_commands);
                    },
                    nav);
            },
            [](const auto&) { return Execute(); },
        },
        envelope.command);
}

}  // namespace swarmkit::agent
