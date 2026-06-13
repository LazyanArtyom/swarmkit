// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <utility>

#include "../src/agent/command_preconditions.h"

namespace swarmkit::agent {
namespace {

[[nodiscard]] CommandEnvelope LandEnvelope() {
    CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.command = commands::FlightCmd{commands::CmdLand{}};
    return envelope;
}

[[nodiscard]] CommandEnvelope DisarmEnvelope() {
    CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.command = commands::FlightCmd{commands::CmdDisarm{}};
    return envelope;
}

template <typename T>
[[nodiscard]] CommandEnvelope Envelope(T command) {
    CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.command = std::move(command);
    return envelope;
}

[[nodiscard]] BackendHealth HealthWithVehicleState() {
    BackendHealth health;
    health.ready = true;
    health.last_heartbeat_unix_ms = 1;
    health.last_telemetry_unix_ms = 1;
    return health;
}

TEST_CASE("Autonomous commands require flight readiness by default",
          "[agent][commands][preconditions][safety]") {
    BackendHealth health = HealthWithVehicleState();
    health.armed = false;
    health.gps_ok = false;
    health.ekf_ok = false;

    const auto takeoff = EvaluateCommandPreconditions(
        Envelope(commands::FlightCmd{commands::CmdTakeoff{.alt_m = 5.0}}), health);
    CHECK(takeoff.action == CommandPreconditionAction::kReject);
    CHECK(takeoff.result.message.find("must be armed") != std::string::npos);

    health.armed = true;
    const auto waypoint =
        EvaluateCommandPreconditions(Envelope(commands::NavCmd{commands::CmdSetWaypoint{
                                         .lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 5.0}}),
                                     health);
    CHECK(waypoint.action == CommandPreconditionAction::kReject);
    CHECK(waypoint.result.message.find("healthy GPS") != std::string::npos);
}

TEST_CASE("Bench override explicitly permits autonomous readiness bypass",
          "[agent][commands][preconditions][safety]") {
    BackendHealth health = HealthWithVehicleState();
    health.armed = false;
    health.gps_ok = false;
    health.ekf_ok = false;

    const auto decision = EvaluateCommandPreconditions(
        Envelope(commands::NavCmd{commands::CmdVelocity{.vx_mps = 1.0F}}), health, true);
    CHECK(decision.action == CommandPreconditionAction::kExecute);
}

TEST_CASE("Land precondition only skips command when disarmed near ground",
          "[agent][commands][preconditions]") {
    const CommandEnvelope envelope = LandEnvelope();

    BackendHealth landed_but_armed = HealthWithVehicleState();
    landed_but_armed.armed = true;
    landed_but_armed.landed = true;
    landed_but_armed.has_relative_altitude = true;
    landed_but_armed.relative_alt_m = 0.1F;
    CHECK(EvaluateCommandPreconditions(envelope, landed_but_armed).action ==
          CommandPreconditionAction::kExecute);

    BackendHealth disarmed_without_altitude = HealthWithVehicleState();
    disarmed_without_altitude.armed = false;
    disarmed_without_altitude.landed = true;
    CHECK(EvaluateCommandPreconditions(envelope, disarmed_without_altitude).action ==
          CommandPreconditionAction::kExecute);

    BackendHealth disarmed_high = HealthWithVehicleState();
    disarmed_high.armed = false;
    disarmed_high.landed = true;
    disarmed_high.has_relative_altitude = true;
    disarmed_high.relative_alt_m = 8.0F;
    CHECK(EvaluateCommandPreconditions(envelope, disarmed_high).action ==
          CommandPreconditionAction::kExecute);

    BackendHealth disarmed_near_ground = HealthWithVehicleState();
    disarmed_near_ground.armed = false;
    disarmed_near_ground.landed = true;
    disarmed_near_ground.has_relative_altitude = true;
    disarmed_near_ground.relative_alt_m = 0.2F;
    const CommandPreconditionDecision decision =
        EvaluateCommandPreconditions(envelope, disarmed_near_ground);
    CHECK(decision.action == CommandPreconditionAction::kAlreadySatisfied);
    CHECK(decision.result.message.find("relative altitude is near ground") != std::string::npos);
}

TEST_CASE("Disarm precondition allows armed bench vehicle near ground",
          "[agent][commands][preconditions]") {
    const CommandEnvelope envelope = DisarmEnvelope();

    BackendHealth armed_near_ground = HealthWithVehicleState();
    armed_near_ground.armed = true;
    armed_near_ground.landed = false;
    armed_near_ground.has_relative_altitude = true;
    armed_near_ground.relative_alt_m = 0.2F;
    CHECK(EvaluateCommandPreconditions(envelope, armed_near_ground).action ==
          CommandPreconditionAction::kExecute);

    BackendHealth armed_high = HealthWithVehicleState();
    armed_high.armed = true;
    armed_high.landed = false;
    armed_high.has_relative_altitude = true;
    armed_high.relative_alt_m = 3.0F;
    const CommandPreconditionDecision decision = EvaluateCommandPreconditions(envelope, armed_high);
    CHECK(decision.action == CommandPreconditionAction::kReject);
    CHECK(decision.result.message.find("appears airborne") != std::string::npos);
}

TEST_CASE("Takeoff precondition does not skip armed near-ground bench state",
          "[agent][commands][preconditions]") {
    const CommandEnvelope envelope =
        Envelope(commands::FlightCmd{commands::CmdTakeoff{.alt_m = 1.0}});

    BackendHealth armed_near_ground = HealthWithVehicleState();
    armed_near_ground.armed = true;
    armed_near_ground.landed = false;
    armed_near_ground.gps_ok = true;
    armed_near_ground.ekf_ok = true;
    armed_near_ground.has_relative_altitude = true;
    armed_near_ground.relative_alt_m = 0.1F;

    CHECK(EvaluateCommandPreconditions(envelope, armed_near_ground).action ==
          CommandPreconditionAction::kExecute);
}

TEST_CASE("Takeoff precondition skips when relative altitude confirms airborne",
          "[agent][commands][preconditions]") {
    const CommandEnvelope envelope =
        Envelope(commands::FlightCmd{commands::CmdTakeoff{.alt_m = 5.0}});

    BackendHealth airborne = HealthWithVehicleState();
    airborne.armed = true;
    airborne.landed = false;
    airborne.gps_ok = true;
    airborne.ekf_ok = true;
    airborne.has_relative_altitude = true;
    airborne.relative_alt_m = 1.2F;

    const CommandPreconditionDecision decision = EvaluateCommandPreconditions(envelope, airborne);
    CHECK(decision.action == CommandPreconditionAction::kAlreadySatisfied);
    CHECK(decision.result.message.find("already airborne") != std::string::npos);
}

}  // namespace
}  // namespace swarmkit::agent
