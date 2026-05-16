// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <string>
#include <unordered_set>
#include <variant>

#include "swarmkit/client/swarm_client.h"
#include "test_support.h"

namespace swarmkit::client {
namespace {

constexpr auto kWaitTimeout = std::chrono::milliseconds{1000};

[[nodiscard]] ClientConfig MakeDefaultClientConfig() {
    ClientConfig config = testsupport::MakeMtlsClientConfig("");
    config.retry_policy.max_attempts = 2;
    config.retry_policy.initial_backoff_ms = 10;
    config.retry_policy.max_backoff_ms = 20;
    config.stream_reconnect_policy.initial_backoff_ms = 10;
    config.stream_reconnect_policy.max_backoff_ms = 20;
    return config;
}

TEST_CASE("SwarmClient apply config supports prefer local addresses", "[swarm][client]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmConfig config;
    config.default_client_config = MakeDefaultClientConfig();
    config.drones = {
        {.drone_id = "drone-1", .address = "127.0.0.1:1", .local_address = drone_one.Address()},
        {.drone_id = "drone-2", .address = "127.0.0.1:2", .local_address = drone_two.Address()},
    };

    SwarmClient swarm(MakeDefaultClientConfig());
    REQUIRE(swarm.ApplyConfig(config, SwarmAddressPreference::kPreferLocal).IsOk());

    const HealthStatus kDroneOneHealth = swarm.GetHealth("drone-1");
    const HealthStatus kDroneTwoHealth = swarm.GetHealth("drone-2");
    REQUIRE(kDroneOneHealth.ok);
    REQUIRE(kDroneTwoHealth.ok);
}

TEST_CASE("SwarmClient broadcasts commands and reports unknown drones", "[swarm][client]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    commands::CommandContext context;
    context.client_id = "test-client";
    context.priority = commands::CommandPriority::kSupervisor;
    const auto kResults = swarm.BroadcastCommand(commands::FlightCmd{commands::CmdArm{}}, context);

    REQUIRE(kResults.size() == 2);
    CHECK(kResults.at("drone-1").ok);
    CHECK(kResults.at("drone-2").ok);
    CHECK(drone_one.Backend().ExecuteCallCount() == 1);
    CHECK(drone_two.Backend().ExecuteCallCount() == 1);

    commands::CommandEnvelope envelope;
    envelope.context.drone_id = "missing";
    envelope.context.client_id = "test-client";
    envelope.command = commands::FlightCmd{commands::CmdArm{}};
    const CommandResult kMissing = swarm.SendCommand(envelope);
    CHECK_FALSE(kMissing.ok);
    CHECK(kMissing.message.find("not registered") != std::string::npos);
    CHECK(kMissing.error.domain == core::ErrorDomain::kSwarm);
    CHECK(kMissing.error.code == RpcStatusCode::kNotFound);
    CHECK(kMissing.error.severity == core::ErrorSeverity::kWarning);
    CHECK(kMissing.error.retryability == core::ErrorRetryability::kAfterRemediation);
}

TEST_CASE("SwarmClient consumes logical swarm commands without backend dispatch",
          "[swarm][client][manager]") {
    testsupport::AgentServerHarness drone_one;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());

    commands::CommandEnvelope role_envelope;
    role_envelope.context.drone_id = "drone-1";
    role_envelope.context.client_id = "test-client";
    role_envelope.command = commands::SwarmCmd{commands::CmdSetRole{.role = "leader"}};

    const CommandResult role_result = swarm.SendCommand(role_envelope);
    REQUIRE(role_result.ok);
    REQUIRE(swarm.GetDroneRole("drone-1").has_value());
    CHECK(*swarm.GetDroneRole("drone-1") == "leader");
    CHECK(drone_one.Backend().ExecuteCallCount() == 0);

    commands::CommandEnvelope formation_envelope;
    formation_envelope.context.drone_id = "drone-1";
    formation_envelope.context.client_id = "test-client";
    formation_envelope.command =
        commands::SwarmCmd{commands::CmdSetFormation{.formation_id = "line-a", .slot_index = 2}};

    const CommandResult formation_result = swarm.SendCommandAndWait(formation_envelope);
    REQUIRE(formation_result.ok);
    const auto assignment = swarm.GetFormationAssignment("drone-1");
    REQUIRE(assignment.has_value());
    CHECK(assignment->formation_id == "line-a");
    CHECK(assignment->slot_index == 2);
    CHECK(drone_one.Backend().ExecuteCallCount() == 0);
}

TEST_CASE("SwarmClient translates formation plans into per-drone goto commands",
          "[swarm][client][manager]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    SwarmFormationPlan plan;
    plan.formation_id = "line-a";
    plan.anchor = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    plan.speed_mps = 3.0F;
    plan.slots = {
        {.drone_id = "drone-1", .north_m = 0.0, .east_m = -5.0, .up_m = 0.0, .role = "left"},
        {.drone_id = "drone-2", .north_m = 0.0, .east_m = 5.0, .up_m = 0.0, .role = "right"},
    };

    commands::CommandContext context;
    context.client_id = "test-client";
    context.priority = commands::CommandPriority::kSupervisor;
    SwarmExecutionOptions options;
    options.synchronization = SwarmExecutionSynchronization::kParallelFanout;

    const SwarmExecutionReport report = swarm.ApplyFormation(plan, context, options);
    REQUIRE(report.ok);
    CHECK(report.succeeded == 2);
    CHECK(drone_one.Backend().ExecuteCallCount() == 1);
    CHECK(drone_two.Backend().ExecuteCallCount() == 1);

    const auto command_one = drone_one.Backend().ExecuteCallAt(0).envelope.command;
    const auto command_two = drone_two.Backend().ExecuteCallAt(0).envelope.command;
    REQUIRE(std::holds_alternative<commands::NavCmd>(command_one));
    REQUIRE(std::holds_alternative<commands::NavCmd>(command_two));
    const auto goto_one = std::get<commands::CmdGoto>(std::get<commands::NavCmd>(command_one));
    const auto goto_two = std::get<commands::CmdGoto>(std::get<commands::NavCmd>(command_two));
    CHECK(goto_one.lon_deg < 44.0);
    CHECK(goto_two.lon_deg > 44.0);
    CHECK(goto_one.speed_mps == 3.0F);
    CHECK(goto_two.speed_mps == 3.0F);
    CHECK(*swarm.GetDroneRole("drone-1") == "left");
    CHECK(*swarm.GetDroneRole("drone-2") == "right");
}

TEST_CASE("SwarmClient applies partial failure hold policy", "[swarm][client][manager]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;
    drone_two.Backend().SetExecuteHandler([](const commands::CommandEnvelope& envelope) {
        if (const auto* flight = std::get_if<commands::FlightCmd>(&envelope.command);
            flight != nullptr && std::holds_alternative<commands::CmdTakeoff>(*flight)) {
            return core::Result::Failed("simulated takeoff failure");
        }
        return core::Result::Success("recovery accepted");
    });

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    commands::CommandContext context;
    context.client_id = "test-client";
    context.priority = commands::CommandPriority::kSupervisor;
    SwarmExecutionOptions options;
    options.partial_failure_policy = SwarmPartialFailurePolicy::kHoldFailed;
    options.synchronization = SwarmExecutionSynchronization::kParallelFanout;

    const SwarmExecutionReport report = swarm.ExecuteSynchronizedCommand(
        commands::FlightCmd{commands::CmdTakeoff{.alt_m = 5.0}}, context, options);
    REQUIRE(report.ok);
    CHECK(report.error.code == RpcStatusCode::kOk);
    CHECK(report.succeeded == 1);
    CHECK(report.failed == 1);
    REQUIRE(report.recovery_results.contains("drone-2"));
    CHECK(report.recovery_results.at("drone-2").ok);
    REQUIRE(drone_two.Backend().ExecuteCallCount() == 2);
    const auto recovery_command = drone_two.Backend().ExecuteCallAt(1).envelope.command;
    REQUIRE(std::holds_alternative<commands::NavCmd>(recovery_command));
    CHECK(std::holds_alternative<commands::CmdHoldPosition>(
        std::get<commands::NavCmd>(recovery_command)));
}

TEST_CASE("SwarmClient lock all and unlock all operate on every drone", "[swarm][client]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    const auto kLockResults = swarm.LockAll(500);
    REQUIRE(kLockResults.size() == 2);
    CHECK(kLockResults.at("drone-1").ok);
    CHECK(kLockResults.at("drone-2").ok);

    swarm.UnlockAll();
}

TEST_CASE("SwarmClient subscribes to telemetry from all drones", "[swarm][client][telemetry]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    std::mutex mutex;
    std::unordered_set<std::string> seen_drones;
    swarm.SubscribeAllTelemetry(5, [&](const core::TelemetryFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        seen_drones.insert(frame.drone_id);
    });

    REQUIRE(testsupport::WaitUntil(
        [&] {
            return drone_one.Backend().HasTelemetryStream("drone-1") &&
                   drone_two.Backend().HasTelemetryStream("drone-2");
        },
        kWaitTimeout));

    core::TelemetryFrame frame_one;
    frame_one.drone_id = "drone-1";
    frame_one.unix_time_ms = 1;
    frame_one.mode = "guided";

    core::TelemetryFrame frame_two;
    frame_two.drone_id = "drone-2";
    frame_two.unix_time_ms = 2;
    frame_two.mode = "guided";

    REQUIRE(testsupport::WaitUntil(
        [&] {
            drone_one.Backend().EmitTelemetry("drone-1", frame_one);
            drone_two.Backend().EmitTelemetry("drone-2", frame_two);
            std::lock_guard<std::mutex> lock(mutex);
            return seen_drones.size() == 2;
        },
        kWaitTimeout, std::chrono::milliseconds{50}));

    swarm.StopAllTelemetry();
}

}  // namespace
}  // namespace swarmkit::client
