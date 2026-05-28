// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "command_preconditions.h"

#include <cmath>
#include <string>
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

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(const commands::CmdArm& /*unused*/,
                                                                const BackendHealth& health) {
    if (health.armed) {
        return AlreadySatisfied("arm already satisfied: vehicle is already armed");
    }
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(const commands::CmdDisarm& /*unused*/,
                                                                const BackendHealth& health) {
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
                                                                const BackendHealth& health) {
    if (!health.armed) {
        return Execute();
    }
    if (health.has_relative_altitude &&
        health.relative_alt_m + kTakeoffAltitudeToleranceM >= static_cast<float>(takeoff.alt_m)) {
        return AlreadySatisfied("takeoff already satisfied: vehicle is at or above target altitude");
    }
    if (!health.landed) {
        return AlreadySatisfied("takeoff already satisfied: vehicle is already airborne");
    }
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(const commands::CmdLand& /*unused*/,
                                                                const BackendHealth& health) {
    if (!health.armed && RelativeAltitudeConfirmsLanded(health)) {
        return AlreadySatisfied(
            "land already satisfied: vehicle is disarmed and relative altitude is near ground");
    }
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdForceDisarm& /*unused*/, const BackendHealth& /*unused*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdForceArm& /*unused*/, const BackendHealth& /*unused*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdFlightTerminate& /*unused*/, const BackendHealth& /*unused*/) {
    return Execute();
}

[[nodiscard]] CommandPreconditionDecision EvaluateFlightCommand(
    const commands::CmdSetMode& /*unused*/, const BackendHealth& /*unused*/) {
    return Execute();
}

}  // namespace

CommandPreconditionDecision EvaluateCommandPreconditions(const CommandEnvelope& envelope,
                                                        const BackendHealth& health) {
    if (!HasVehicleState(health)) {
        return Execute();
    }

    return std::visit(
        core::Overloaded{
            [&](const commands::FlightCmd& flight) {
                return std::visit(
                    [&](const auto& command) { return EvaluateFlightCommand(command, health); },
                    flight);
            },
            [](const auto&) { return Execute(); },
        },
        envelope.command);
}

}  // namespace swarmkit::agent
