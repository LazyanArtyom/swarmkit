// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
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
    CHECK(kMissing.error.code == core::ErrorCode::kNotFound);
    CHECK(kMissing.error.severity == core::ErrorSeverity::kWarning);
    CHECK(kMissing.error.retryability == core::ErrorRetryability::kAfterRemediation);
}

TEST_CASE("SwarmClient fans out active goals", "[swarm][client][goal]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    ActiveGoal base_goal;
    base_goal.goal_id = "route-step";
    base_goal.revision = 11;
    base_goal.target = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    base_goal.acceptance_radius_m = 3.0F;
    base_goal.deviation_radius_m = 20.0F;
    base_goal.labels = {{"edge_id", "edge-a"}};

    std::unordered_map<std::string, ActiveGoalRequest> goals;
    goals.emplace("drone-1", ActiveGoalRequest{.goal = base_goal});
    goals.emplace("drone-2", ActiveGoalRequest{.goal = base_goal});

    const auto results = swarm.SetActiveGoals(goals);
    REQUIRE(results.size() == 2);
    REQUIRE(results.at("drone-1").ok);
    REQUIRE(results.at("drone-2").ok);
    CHECK(results.at("drone-1").goal.drone_id == "drone-1");
    CHECK(results.at("drone-2").goal.drone_id == "drone-2");
    CHECK(results.at("drone-1").goal.labels.at("edge_id") == "edge-a");
    CHECK(drone_one.Backend().ExecuteCallCount() == 1);
    CHECK(drone_two.Backend().ExecuteCallCount() == 1);

    const auto command_one = drone_one.Backend().ExecuteCallAt(0).envelope.command;
    REQUIRE(std::holds_alternative<commands::NavCmd>(command_one));
    CHECK(std::holds_alternative<commands::CmdGoto>(std::get<commands::NavCmd>(command_one)));
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

    const auto kUnlockResults = swarm.UnlockAll();
    REQUIRE(kUnlockResults.size() == 2);
    CHECK(kUnlockResults.at("drone-1").ok);
    CHECK(kUnlockResults.at("drone-2").ok);
}

TEST_CASE("SwarmClient subscribes to telemetry from all drones", "[swarm][client][telemetry]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.Address());
    swarm.AddDrone("drone-2", drone_two.Address());

    std::mutex mutex;
    std::unordered_set<std::string> seen_drones;
    auto telemetry_streams = swarm.StartAllTelemetry(
        5, testsupport::OnTelemetryFrame([&](const TelemetryDelivery& delivery) {
            std::lock_guard<std::mutex> lock(mutex);
            seen_drones.insert(delivery.frame.drone_id);
        }));
    REQUIRE(telemetry_streams.size() == 2);
    REQUIRE(telemetry_streams.at("drone-1").has_value());
    REQUIRE(telemetry_streams.at("drone-2").has_value());

    REQUIRE(testsupport::WaitUntil(
        [&] {
            return drone_one.Backend().HasTelemetryStream("drone-1") &&
                   drone_two.Backend().HasTelemetryStream("drone-2");
        },
        kWaitTimeout));

    core::TelemetryFrame frame_one;
    frame_one.drone_id = "drone-1";
    frame_one.agent_receive_unix_time_ms = 1;
    frame_one.mode = "guided";

    core::TelemetryFrame frame_two;
    frame_two.drone_id = "drone-2";
    frame_two.agent_receive_unix_time_ms = 2;
    frame_two.mode = "guided";

    REQUIRE(testsupport::WaitUntil(
        [&] {
            drone_one.Backend().EmitTelemetry("drone-1", frame_one);
            drone_two.Backend().EmitTelemetry("drone-2", frame_two);
            std::lock_guard<std::mutex> lock(mutex);
            return seen_drones.size() == 2;
        },
        kWaitTimeout, std::chrono::milliseconds{50}));

    for (auto& entry : telemetry_streams) {
        entry.second->Stop();
    }
}

}  // namespace
}  // namespace swarmkit::client
