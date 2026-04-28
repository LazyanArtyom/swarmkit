// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "test_support.h"

namespace swarmkit::client {
namespace {

constexpr auto kWaitTimeout = std::chrono::milliseconds{1000};

[[nodiscard]] Client MakeClient(const std::string& address) {
    ClientConfig config = testsupport::MakeMtlsClientConfig(address);
    config.retry_policy.max_attempts = 3;
    config.retry_policy.initial_backoff_ms = 10;
    config.retry_policy.max_backoff_ms = 20;
    config.stream_reconnect_policy.initial_backoff_ms = 10;
    config.stream_reconnect_policy.max_backoff_ms = 20;
    return Client(std::move(config));
}

TEST_CASE("Client integrates with agent service for ping health stats and command execution",
          "[client][integration]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    const PingResult kPing = client.Ping();
    REQUIRE(kPing.ok);
    CHECK(kPing.agent_id == "test-agent");
    CHECK_FALSE(kPing.version.empty());
    CHECK_FALSE(kPing.correlation_id.empty());

    const HealthStatus kHealth = client.GetHealth();
    REQUIRE(kHealth.ok);
    CHECK(kHealth.ready);
    CHECK(kHealth.agent_id == "test-agent");

    commands::CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.context.priority = commands::CommandPriority::kSupervisor;
    envelope.command = commands::FlightCmd{commands::CmdArm{}};

    const CommandResult kCommand = client.SendCommand(envelope);
    REQUIRE(kCommand.ok);
    CHECK(harness.Backend().ExecuteCallCount() == 1);
    CHECK(harness.Backend().ExecuteCallAt(0).envelope.context.drone_id == "drone-1");

    const RuntimeStats kStats = client.GetRuntimeStats();
    REQUIRE(kStats.ok);
    CHECK(kStats.ping_requests_total >= 1);
    CHECK(kStats.health_requests_total >= 1);
    CHECK(kStats.command_requests_total >= 1);
}

TEST_CASE("Client authority session auto releases lock and emits watch events",
          "[client][integration][authority]") {
    testsupport::AgentServerHarness harness;
    Client operator_client = MakeClient(harness.Address());
    Client override_client = MakeClient(harness.Address());

    std::mutex events_mutex;
    std::vector<AuthorityEventInfo> operator_events;
    operator_client.WatchAuthority(
        {.drone_id = "drone-1", .priority = commands::CommandPriority::kOperator},
        [&](const AuthorityEventInfo& event) {
            std::lock_guard<std::mutex> lock(events_mutex);
            operator_events.push_back(event);
        });

    {
        const auto kSession = operator_client.AcquireAuthoritySession("drone-1", 500);
        REQUIRE(kSession.has_value());
    }

    const CommandResult kRelock = override_client.LockAuthority("drone-1", 500);
    REQUIRE(kRelock.ok);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(events_mutex);
            return !operator_events.empty();
        },
        kWaitTimeout));

    operator_client.StopAuthorityWatch();
}

TEST_CASE("Client telemetry subscription receives frames and can stop cleanly",
          "[client][integration][telemetry]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::atomic<int> frame_count{0};
    client.SubscribeTelemetry({.drone_id = "drone-1", .rate_hertz = 5},
                              [&](const core::TelemetryFrame& frame) {
                                  if (frame.drone_id == "drone-1") {
                                      frame_count.fetch_add(1, std::memory_order_relaxed);
                                  }
                              });

    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    core::TelemetryFrame frame;
    frame.drone_id = "drone-1";
    frame.unix_time_ms = 123;
    frame.lat_deg = 40.0;
    frame.lon_deg = 44.0;
    frame.rel_alt_m = 10.0F;
    frame.battery_percent = 80.0F;
    frame.mode = "guided";

    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", frame);
            return frame_count.load(std::memory_order_relaxed) >= 1;
        },
        kWaitTimeout, std::chrono::milliseconds{50}));

    client.StopTelemetry();
    REQUIRE(testsupport::WaitUntil([&] { return !harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));
}

TEST_CASE("Client reports backend execution failure and telemetry counters",
          "[client][integration]") {
    testsupport::AgentServerHarness harness;
    harness.Backend().SetExecuteHandler([](const commands::CommandEnvelope&) {
        return core::Result::Failed("simulated backend failure");
    });

    Client client = MakeClient(harness.Address());

    commands::CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.context.priority = commands::CommandPriority::kSupervisor;
    envelope.command = commands::FlightCmd{commands::CmdLand{}};

    const CommandResult kCommand = client.SendCommand(envelope);
    CHECK_FALSE(kCommand.ok);
    CHECK(kCommand.error.code == RpcStatusCode::kInternal);

    const RuntimeStats kStats = client.GetRuntimeStats();
    REQUIRE(kStats.ok);
    CHECK(kStats.command_failed_total >= 1);
    CHECK(kStats.backend_failures_total >= 1);
}

}  // namespace
}  // namespace swarmkit::client
