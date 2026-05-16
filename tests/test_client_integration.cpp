// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <ranges>
#include <string>
#include <thread>
#include <vector>

#include "swarmkit/client/client.h"
#include "swarmkit/commands.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"
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

    const BackendCapabilities capabilities = client.GetCapabilities();
    REQUIRE(capabilities.ok);
    CHECK(capabilities.agent_id == "test-agent");
    CHECK(capabilities.autopilot_type == "recording");
    CHECK(capabilities.supports_mission_upload);
    CHECK(capabilities.supports_velocity_control);
    CHECK_FALSE(capabilities.supports_payload_control);
    CHECK(std::ranges::contains(capabilities.supported_modes, "guided"));
}

TEST_CASE("Client verified command helpers use agent health and telemetry",
          "[client][integration][commands]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    swarmkit::agent::BackendHealth armed_health;
    armed_health.ready = true;
    armed_health.message = "armed";
    armed_health.last_heartbeat_unix_ms = 1;
    armed_health.last_telemetry_unix_ms = 1;
    armed_health.armed = true;
    armed_health.landed = false;
    harness.Backend().SetHealth(armed_health);

    CommandWaitOptions options;
    options.timeout_ms = 500;
    options.poll_interval_ms = 10;
    options.telemetry_rate_hz = 10;

    const CommandResult kArm = client.ArmAndWait("drone-1", options);
    REQUIRE(kArm.ok);
    CHECK(kArm.message.find("verified") != std::string::npos);
    CHECK(harness.Backend().ExecuteCallCount() == 0);

    harness.Backend().SetHealth({});
    std::atomic<bool> done{false};
    std::atomic<bool> stream_started{false};
    std::thread emitter([&] {
        stream_started.store(testsupport::WaitUntil(
                                 [&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                 kWaitTimeout),
                             std::memory_order_relaxed);
        if (!stream_started.load(std::memory_order_relaxed)) {
            return;
        }
        core::TelemetryFrame frame;
        frame.drone_id = "drone-1";
        frame.unix_time_ms = 123;
        frame.armed = true;
        frame.landed = false;
        frame.rel_alt_m = 5.1F;
        while (!done.load(std::memory_order_relaxed)) {
            harness.Backend().EmitTelemetry("drone-1", frame);
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
    });

    const CommandResult kTakeoff = client.TakeoffAndWait("drone-1", 5.0, options);
    done.store(true, std::memory_order_relaxed);
    emitter.join();

    REQUIRE(stream_started.load(std::memory_order_relaxed));
    REQUIRE(kTakeoff.ok);
    CHECK(kTakeoff.message.find("takeoff verified") != std::string::npos);
    CHECK(harness.Backend().ExecuteCallCount() == 1);
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

TEST_CASE("Agent validates already-satisfied commands before granting authority",
          "[agent][authority][commands]") {
    auto backend = std::make_unique<testsupport::RecordingBackend>();
    testsupport::RecordingBackend* backend_ptr = backend.get();

    swarmkit::agent::AgentConfig config;
    config.agent_id = "test-agent";
    config.security.transport_security = core::TransportSecurityMode::kInsecure;

    auto service =
        swarmkit::agent::internal::MakeAgentServiceForTesting(config, std::move(backend));
    auto* agent_service = dynamic_cast<swarmkit::v1::AgentService::Service*>(service.get());
    REQUIRE(agent_service != nullptr);

    grpc::ServerContext lock_context;
    swarmkit::v1::LockAuthorityRequest lock_request;
    auto* lock_ctx = lock_request.mutable_ctx();
    lock_ctx->set_drone_id("drone-1");
    lock_ctx->set_client_id("operator-client");
    lock_ctx->set_priority(static_cast<std::int32_t>(commands::CommandPriority::kOperator));
    lock_request.set_ttl_ms(5000);
    swarmkit::v1::LockAuthorityReply lock_reply;
    REQUIRE(agent_service->LockAuthority(&lock_context, &lock_request, &lock_reply).ok());
    REQUIRE(lock_reply.ok());

    swarmkit::agent::BackendHealth armed_health;
    armed_health.ready = true;
    armed_health.last_heartbeat_unix_ms = 1;
    armed_health.last_telemetry_unix_ms = 1;
    armed_health.armed = true;
    armed_health.landed = false;
    backend_ptr->SetHealth(armed_health);

    grpc::ServerContext satisfied_context;
    swarmkit::v1::CommandRequest satisfied_request;
    auto* satisfied_ctx = satisfied_request.mutable_ctx();
    satisfied_ctx->set_drone_id("drone-1");
    satisfied_ctx->set_client_id("override-client");
    satisfied_ctx->set_priority(static_cast<std::int32_t>(commands::CommandPriority::kOverride));
    satisfied_request.mutable_cmd()->mutable_arm();
    swarmkit::v1::CommandReply satisfied_reply;
    REQUIRE(agent_service
                ->SendCommand(&satisfied_context, &satisfied_request, &satisfied_reply)
                .ok());
    REQUIRE(satisfied_reply.status() == swarmkit::v1::CommandReply::OK);
    CHECK(satisfied_reply.message().find("already satisfied") != std::string::npos);
    CHECK(backend_ptr->ExecuteCallCount() == 0);

    grpc::ServerContext execute_context;
    swarmkit::v1::CommandRequest execute_request;
    auto* execute_ctx = execute_request.mutable_ctx();
    execute_ctx->set_drone_id("drone-1");
    execute_ctx->set_client_id("operator-client");
    execute_ctx->set_priority(static_cast<std::int32_t>(commands::CommandPriority::kOperator));
    execute_request.mutable_cmd()->mutable_set_mode()->set_mode("guided");
    swarmkit::v1::CommandReply execute_reply;
    REQUIRE(agent_service->SendCommand(&execute_context, &execute_request, &execute_reply).ok());
    REQUIRE(execute_reply.status() == swarmkit::v1::CommandReply::OK);
    CHECK(backend_ptr->ExecuteCallCount() == 1);
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

TEST_CASE("Client active goal emits active and reached reports", "[client][integration][goal]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    client.SubscribeReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
        std::lock_guard<std::mutex> lock(reports_mutex);
        reports.push_back(report);
    });

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-reached";
    goal.revision = 7;
    goal.target = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    goal.acceptance_radius_m = 5.0F;
    goal.deviation_radius_m = 25.0F;
    goal.timeout_ms = 5000;

    const GoalResult result = client.SetActiveGoal(goal);
    REQUIRE(result.ok);
    CHECK(result.goal.goal_id == "goal-reached");
    CHECK(result.computed_timeout_ms == 5000);
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
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [](const AgentReport& report) {
                return report.goal.has_value() &&
                       report.goal->status == GoalStatus::kReached &&
                       report.goal->goal_id == "goal-reached";
            });
        },
        kWaitTimeout, std::chrono::milliseconds{50}));

    const ActiveGoalStatus status = client.GetActiveGoal("drone-1");
    REQUIRE(status.has_goal);
    CHECK(status.status == GoalStatus::kReached);

    client.StopReports();
}

TEST_CASE("Client can cancel active goal and receive cancellation report",
          "[client][integration][goal]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    client.SubscribeReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
        std::lock_guard<std::mutex> lock(reports_mutex);
        reports.push_back(report);
    });

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-cancel";
    goal.revision = 8;
    goal.target = {.lat_deg = 41.0, .lon_deg = 45.0, .alt_m = 20.0};
    goal.acceptance_radius_m = 2.0F;
    goal.deviation_radius_m = 10.0F;
    goal.timeout_ms = 5000;

    const GoalResult set_result = client.SetActiveGoal(goal);
    REQUIRE(set_result.ok);
    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    const CommandResult cancel_result = client.CancelGoal("drone-1", "goal-cancel");
    REQUIRE(cancel_result.ok);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [](const AgentReport& report) {
                return report.goal.has_value() &&
                       report.goal->status == GoalStatus::kCancelled &&
                       report.goal->goal_id == "goal-cancel";
            });
        },
        kWaitTimeout));

    const ActiveGoalStatus status = client.GetActiveGoal("drone-1");
    CHECK_FALSE(status.has_goal);

    client.StopReports();
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
    CHECK(kCommand.error.domain == core::ErrorDomain::kBackend);
    CHECK(kCommand.error.code == RpcStatusCode::kBackendFailure);
    CHECK(kCommand.error.severity == core::ErrorSeverity::kError);
    CHECK(kCommand.error.retryability == core::ErrorRetryability::kUnknown);
    CHECK(kCommand.error.remediation.find("backend") != std::string::npos);

    const RuntimeStats kStats = client.GetRuntimeStats();
    REQUIRE(kStats.ok);
    CHECK(kStats.command_failed_total >= 1);
    CHECK(kStats.backend_failures_total >= 1);
}

}  // namespace
}  // namespace swarmkit::client
