// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <string>
#include <unordered_set>

#include "test_support.h"
#include "swarmkit/client/swarm_client.h"

namespace swarmkit::client {
namespace {

constexpr auto kWaitTimeout = std::chrono::milliseconds{1000};

[[nodiscard]] ClientConfig MakeDefaultClientConfig() {
    ClientConfig config;
    config.client_id = "swarm-test-client";
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
        {.drone_id = "drone-1", .address = "127.0.0.1:1", .local_address = drone_one.address()},
        {.drone_id = "drone-2", .address = "127.0.0.1:2", .local_address = drone_two.address()},
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
    swarm.AddDrone("drone-1", drone_one.address());
    swarm.AddDrone("drone-2", drone_two.address());

    commands::CommandContext context;
    context.client_id = "swarm-test-client";
    context.priority = commands::CommandPriority::kSupervisor;
    const auto kResults =
        swarm.BroadcastCommand(commands::FlightCmd{commands::CmdArm{}}, context);

    REQUIRE(kResults.size() == 2);
    CHECK(kResults.at("drone-1").ok);
    CHECK(kResults.at("drone-2").ok);
    CHECK(drone_one.backend().ExecuteCallCount() == 1);
    CHECK(drone_two.backend().ExecuteCallCount() == 1);

    commands::CommandEnvelope envelope;
    envelope.context.drone_id = "missing";
    envelope.context.client_id = "swarm-test-client";
    envelope.command = commands::FlightCmd{commands::CmdArm{}};
    const CommandResult kMissing = swarm.SendCommand(envelope);
    CHECK_FALSE(kMissing.ok);
    CHECK(kMissing.message.find("not registered") != std::string::npos);
}

TEST_CASE("SwarmClient lock all and unlock all operate on every drone", "[swarm][client]") {
    testsupport::AgentServerHarness drone_one;
    testsupport::AgentServerHarness drone_two;

    SwarmClient swarm(MakeDefaultClientConfig());
    swarm.AddDrone("drone-1", drone_one.address());
    swarm.AddDrone("drone-2", drone_two.address());

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
    swarm.AddDrone("drone-1", drone_one.address());
    swarm.AddDrone("drone-2", drone_two.address());

    std::mutex mutex;
    std::unordered_set<std::string> seen_drones;
    swarm.SubscribeAllTelemetry(
        5, [&](const core::TelemetryFrame& frame) {
            std::lock_guard<std::mutex> lock(mutex);
            seen_drones.insert(frame.drone_id);
        });

    REQUIRE(testsupport::WaitUntil(
        [&] {
            return drone_one.backend().HasTelemetryStream("drone-1") &&
                   drone_two.backend().HasTelemetryStream("drone-2");
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
            drone_one.backend().EmitTelemetry("drone-1", frame_one);
            drone_two.backend().EmitTelemetry("drone-2", frame_two);
            std::lock_guard<std::mutex> lock(mutex);
            return seen_drones.size() == 2;
        },
        kWaitTimeout, std::chrono::milliseconds{50}));

    swarm.StopAllTelemetry();
}

}  // namespace
}  // namespace swarmkit::client
