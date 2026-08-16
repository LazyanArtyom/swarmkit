// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <numbers>
#include <string>
#include <type_traits>
#include <vector>

#include "swarmkit/agent/backend_factory.h"
#include "swarmkit/agent/sim_backend.h"

namespace swarmkit::agent {
namespace {

using Catch::Approx;
using namespace std::chrono_literals;

[[nodiscard]] commands::CommandEnvelope Flight(std::string drone_id, commands::FlightCmd command) {
    return {
        .context = {.drone_id = std::move(drone_id), .correlation_id = "sim-flight"},
        .command = std::move(command),
    };
}

[[nodiscard]] commands::CommandEnvelope Navigation(std::string drone_id, commands::NavCmd command) {
    return {
        .context = {.drone_id = std::move(drone_id), .correlation_id = "sim-navigation"},
        .command = std::move(command),
    };
}

TEST_CASE("SimBackend validates configuration and advertises enforced model bounds",
          "[agent][sim]") {
    SimBackendConfig invalid;
    invalid.default_cruise_speed_mps = invalid.max_horizontal_speed_mps + 1.0F;
    CHECK_FALSE(MakeSimBackend(invalid).has_value());

    SimBackendConfig config;
    config.clock_mode = SimulationClockMode::kManual;
    auto simulator = MakeSimBackend(config);
    REQUIRE(simulator.has_value());
    const core::BackendCapabilities capabilities = simulator->backend->GetCapabilities();
    REQUIRE(capabilities.max_horizontal_speed.has_value());
    REQUIRE(capabilities.max_climb_speed.has_value());
    REQUIRE(capabilities.max_descent_speed.has_value());
    CHECK(capabilities.max_horizontal_speed->value == config.max_horizontal_speed_mps);
    CHECK(capabilities.max_horizontal_speed->semantics ==
          core::MotionLimitSemantics::kValidatedBound);
    CHECK(capabilities.evidence.horizontal_velocity_uncertainty ==
          core::CapabilitySupport::kSupported);
}

TEST_CASE("SimBackend health only reports observed vehicle state", "[agent][sim][health][safety]") {
    SimBackendConfig config;
    config.clock_mode = SimulationClockMode::kManual;
    auto simulator = MakeSimBackend(config);
    REQUIRE(simulator.has_value());

    const BackendHealth before_vehicle = simulator->backend->GetHealth();
    CHECK_FALSE(before_vehicle.armed.has_value());
    CHECK_FALSE(before_vehicle.landed.has_value());
    CHECK_FALSE(before_vehicle.failsafe.has_value());
    CHECK_FALSE(before_vehicle.gps_ok.has_value());
    CHECK_FALSE(before_vehicle.ekf_ok.has_value());
    CHECK(before_vehicle.last_heartbeat_unix_ms == 0);
    CHECK(before_vehicle.last_telemetry_unix_ms == 0);

    REQUIRE(simulator->backend->StartTelemetry("drone-1", 1, [](const core::TelemetryFrame&) {})
                .IsOk());
    const BackendHealth before_frame = simulator->backend->GetHealth();
    CHECK(before_frame.armed == false);
    CHECK(before_frame.landed == true);
    CHECK(before_frame.failsafe == false);
    CHECK(before_frame.gps_ok == true);
    CHECK(before_frame.ekf_ok == true);
    CHECK(before_frame.last_heartbeat_unix_ms > 0);
    CHECK(before_frame.last_telemetry_unix_ms == 0);

    REQUIRE(simulator->control->Advance("drone-1", std::chrono::seconds{1}).IsOk());
    CHECK(simulator->backend->GetHealth().last_telemetry_unix_ms > 0);
    CHECK(simulator->backend->StopTelemetry("drone-1").IsOk());
}

TEST_CASE("Built-in simulator factory validates experiment options", "[agent][sim][factory]") {
    BackendRegistry registry;
    RegisterBuiltinBackends(&registry);
    auto backend = registry.Create({.backend_name = "sim",
                                    .options = {{"max_horizontal_speed_mps", "6.5"},
                                                {"default_cruise_speed_mps", "2.0"},
                                                {"stop_at_target", "false"}}});
    REQUIRE(backend.has_value());
    REQUIRE((*backend)->GetCapabilities().max_horizontal_speed.has_value());
    CHECK((*backend)->GetCapabilities().max_horizontal_speed->value == 6.5F);

    auto invalid = registry.Create(
        {.backend_name = "sim", .options = {{"max_horizontal_speed_mps", "not-a-number"}}});
    REQUIRE_FALSE(invalid.has_value());
    CHECK(invalid.error().code == core::StatusCode::kRejected);
}

TEST_CASE("SimBackend responds deterministically to takeoff velocity hold and failure",
          "[agent][sim]") {
    std::vector<SimulationTruthFrame> truth_log;
    SimBackendConfig config;
    config.clock_mode = SimulationClockMode::kManual;
    config.integration_step_ms = 20;
    config.truth_observer = [&truth_log](const SimulationTruthFrame& truth) {
        truth_log.push_back(truth);
    };
    auto simulator = MakeSimBackend(config);
    REQUIRE(simulator.has_value());

    std::vector<core::TelemetryFrame> telemetry;
    REQUIRE(simulator->backend
                ->StartTelemetry(
                    "drone-1", 10,
                    [&telemetry](const core::TelemetryFrame& frame) { telemetry.push_back(frame); })
                .IsOk());
    REQUIRE(simulator->backend->Execute(Flight("drone-1", commands::CmdArm{})).IsOk());
    REQUIRE(
        simulator->backend->Execute(Flight("drone-1", commands::CmdTakeoff{.alt_m = 5.0})).IsOk());
    REQUIRE(simulator->control->Advance("drone-1", 1s).IsOk());

    auto truth = simulator->control->Truth("drone-1");
    REQUIRE(truth.has_value());
    CHECK(truth->up_m == Approx(5.0));
    CHECK(truth->armed);
    CHECK_FALSE(truth->landed);
    REQUIRE(telemetry.size() == 10);
    CHECK(telemetry.back().rel_alt_m == Approx(5.0F));
    REQUIRE(telemetry.back().accuracy.horizontal_position.has_value());
    CHECK(telemetry.back().accuracy.horizontal_position->descriptor.semantics ==
          core::UncertaintySemantics::kDeterministicHardBound);
    CHECK(telemetry.back().provenance.position.source == "sim.estimated_position");
    static_assert(!std::is_same_v<SimulationTruthFrame, core::TelemetryFrame>);

    REQUIRE(simulator->backend
                ->Execute(Navigation("drone-1",
                                     commands::CmdVelocity{
                                         .vx_mps = 100.0F,
                                         .duration_ms = 1000,
                                     }))
                .IsOk());
    REQUIRE(simulator->control->Advance("drone-1", 1s).IsOk());
    truth = simulator->control->Truth("drone-1");
    REQUIRE(truth.has_value());
    CHECK(truth->north_m == Approx(config.max_horizontal_speed_mps));

    REQUIRE(simulator->backend->Execute(Navigation("drone-1", commands::CmdHoldPosition{})).IsOk());
    const double held_north = truth->north_m;
    REQUIRE(simulator->control->Advance("drone-1", 500ms).IsOk());
    CHECK(simulator->control->Truth("drone-1")->north_m == Approx(held_north));

    REQUIRE(simulator->backend->Execute(Navigation("drone-1", commands::CmdReturnHome{})).IsOk());
    REQUIRE(simulator->control->Advance("drone-1", 3s).IsOk());
    truth = simulator->control->Truth("drone-1");
    REQUIRE(truth.has_value());
    CHECK(truth->north_m == Approx(0.0));
    CHECK(truth->up_m == Approx(0.0));
    CHECK(truth->landed);
    CHECK_FALSE(truth->armed);

    simulator->control->SetFailure("drone-1", true, "scripted motor failure");
    CHECK_FALSE(simulator->backend
                    ->Execute(Navigation("drone-1",
                                         commands::CmdVelocity{
                                             .vx_mps = 1.0F,
                                             .duration_ms = 100,
                                         }))
                    .IsOk());
    REQUIRE(simulator->control->Advance("drone-1", 100ms).IsOk());
    truth = simulator->control->Truth("drone-1");
    REQUIRE(truth.has_value());
    CHECK(truth->failed);
    CHECK(simulator->backend->GetHealth().failsafe == true);
    CHECK_FALSE(truth_log.empty());
    CHECK(simulator->backend->StopTelemetry("drone-1").IsOk());
}

TEST_CASE("SimBackend can reproduce high-speed target crossing without exposing truth as telemetry",
          "[agent][sim][crossing]") {
    std::vector<SimulationTruthFrame> truth_log;
    SimBackendConfig config;
    config.clock_mode = SimulationClockMode::kManual;
    config.integration_step_ms = 20;
    config.stop_at_target = false;
    config.truth_observer = [&truth_log](const SimulationTruthFrame& truth) {
        truth_log.push_back(truth);
    };
    auto simulator = MakeSimBackend(config);
    REQUIRE(simulator.has_value());
    REQUIRE(simulator->backend->Execute(Flight("drone-1", commands::CmdArm{})).IsOk());

    constexpr double target_north_m = 0.95;
    const double target_lat =
        config.home_lat_deg + target_north_m / 6'371'000.0 * 180.0 / std::numbers::pi;
    REQUIRE(simulator->backend
                ->Execute(Navigation("drone-1",
                                     commands::CmdGoto{
                                         .lat_deg = target_lat,
                                         .lon_deg = config.home_lon_deg,
                                         .alt_m = 0.0,
                                         .speed_mps = 10.0F,
                                     }))
                .IsOk());
    REQUIRE(simulator->control->Advance("drone-1", 120ms).IsOk());
    CHECK(std::ranges::any_of(truth_log, [](const SimulationTruthFrame& truth) {
        return truth.north_m > target_north_m;
    }));
}

TEST_CASE("Two manual simulators produce identical trajectories", "[agent][sim][determinism]") {
    SimBackendConfig config;
    config.clock_mode = SimulationClockMode::kManual;
    auto first = MakeSimBackend(config);
    auto second = MakeSimBackend(config);
    REQUIRE(first.has_value());
    REQUIRE(second.has_value());

    for (auto* simulator : {&*first, &*second}) {
        REQUIRE(simulator->backend->Execute(Flight("drone-1", commands::CmdArm{})).IsOk());
        REQUIRE(simulator->backend->Execute(Flight("drone-1", commands::CmdTakeoff{.alt_m = 3.0}))
                    .IsOk());
        REQUIRE(simulator->control->Advance("drone-1", 1500ms).IsOk());
    }
    CHECK(first->control->Truth("drone-1") == second->control->Truth("drone-1"));
}

}  // namespace
}  // namespace swarmkit::agent
