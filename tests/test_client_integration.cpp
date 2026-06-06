// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <stdexcept>
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
    swarmkit::agent::BackendHealth healthy;
    healthy.ready = true;
    healthy.message = "ready";
    healthy.last_heartbeat_unix_ms = 1;
    healthy.last_telemetry_unix_ms = 1;
    healthy.armed = false;
    healthy.landed = true;
    healthy.gps_ok = true;
    healthy.ekf_ok = true;
    harness.Backend().SetHealth(healthy);
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
    CHECK_FALSE(kHealth.link_quality_percent.has_value());
    CHECK(kHealth.autonomous_ready);
    CHECK(kHealth.arming_blockers.empty());
    CHECK_FALSE(kHealth.readiness_checks.empty());
    CHECK(std::ranges::any_of(kHealth.readiness_checks, [](const ReadinessCheck& check) {
        return check.name == "gps" && check.ok;
    }));

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
    CHECK_FALSE(capabilities.max_horizontal_speed_mps.has_value());
    CHECK_FALSE(capabilities.max_climb_speed_mps.has_value());
    CHECK_FALSE(capabilities.max_descent_speed_mps.has_value());
    CHECK_FALSE(capabilities.max_altitude_m.has_value());
}

TEST_CASE("Client command reports are replayable and delivered live", "[client][integration][reports]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    commands::CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.context.priority = commands::CommandPriority::kSupervisor;
    envelope.command = commands::FlightCmd{commands::CmdSetMode{.mode = "guided"}};

    const CommandResult replayed_command = client.SendCommand(envelope);
    REQUIRE(replayed_command.ok);

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto reports_stream =
        client.StartReports({.drone_id = "drone-1", .after_sequence = 0},
                            [&](const AgentReport& report) {
                                std::lock_guard<std::mutex> lock(reports_mutex);
                                reports.push_back(report);
                            });
    REQUIRE(reports_stream.has_value());

    const auto has_report = [&](const std::string& correlation_id, AgentReportType type) {
        std::lock_guard<std::mutex> lock(reports_mutex);
        return std::ranges::any_of(reports, [&](const AgentReport& report) {
            return report.correlation_id == correlation_id && report.type == type;
        });
    };

    REQUIRE(testsupport::WaitUntil(
        [&] {
            return has_report(replayed_command.correlation_id, AgentReportType::kCommandAccepted) &&
                   has_report(replayed_command.correlation_id, AgentReportType::kCommandAcked);
        },
        kWaitTimeout));

    envelope.command = commands::FlightCmd{commands::CmdSetMode{.mode = "loiter"}};
    const CommandResult live_command = client.SendCommand(envelope);
    REQUIRE(live_command.ok);

    REQUIRE(testsupport::WaitUntil(
        [&] { return has_report(live_command.correlation_id, AgentReportType::kCommandAcked); },
        kWaitTimeout));

    reports_stream->Stop();
}

TEST_CASE("Client data messages publish and subscribe independently",
          "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    Client publisher = MakeClient(harness.Address());
    Client subscriber = MakeClient(harness.Address());

    std::mutex mutex;
    std::vector<DataMessage> messages;
    auto stream = subscriber.StartMessages(
        {.subscriber_id = "subscriber-1", .topics = {"rotor.event"}},
        [&](const DataMessage& message) {
            std::lock_guard<std::mutex> lock(mutex);
            messages.push_back(message);
        });
    REQUIRE(stream.has_value());

    DataMessage message;
    message.topic = "rotor.event";
    message.labels["edge_id"] = "edge-7";
    message.payload = R"({"state":"completed"})";
    const PublishMessageResult published = publisher.PublishMessage(std::move(message));
    REQUIRE(published.ok);
    CHECK(published.sequence > 0);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(mutex);
            return !messages.empty();
        },
        kWaitTimeout));

    {
        std::lock_guard<std::mutex> lock(mutex);
        REQUIRE(messages.size() == 1);
        CHECK(messages.front().topic == "rotor.event");
        CHECK(messages.front().source_id == "test-client");
        CHECK(messages.front().labels.at("edge_id") == "edge-7");
        CHECK(messages.front().payload == R"({"state":"completed"})");
    }
    const RuntimeStats active_stats = publisher.GetRuntimeStats();
    REQUIRE(active_stats.ok);
    CHECK(active_stats.data_messages_published_total >= 1);
    CHECK(active_stats.current_message_subscribers >= 1);
    stream->Stop();
}

TEST_CASE("Client artifacts upload download and verify hash", "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-artifact-input.bin";
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-artifact-output.bin";
    const std::string payload = "image-bytes-that-stand-in-for-a-frame";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }

    ArtifactUpload upload;
    upload.file_path = input_path.string();
    upload.chunk_bytes = 8;
    upload.descriptor.content_type = "image/jpeg";
    upload.descriptor.labels["camera_id"] = "front";

    const ArtifactTransferResult uploaded = client.UploadArtifact(upload);
    REQUIRE(uploaded.ok);
    REQUIRE_FALSE(uploaded.descriptor.artifact_id.empty());
    CHECK(uploaded.descriptor.size_bytes == static_cast<std::int64_t>(payload.size()));
    CHECK(uploaded.descriptor.content_type == "image/jpeg");
    CHECK(uploaded.descriptor.labels.at("camera_id") == "front");
    CHECK_FALSE(uploaded.descriptor.sha256_hex.empty());

    const ArtifactTransferResult downloaded =
        client.DownloadArtifact(uploaded.descriptor.artifact_id, output_path.string());
    REQUIRE(downloaded.ok);
    CHECK(downloaded.descriptor.artifact_id == uploaded.descriptor.artifact_id);
    CHECK(downloaded.descriptor.sha256_hex == uploaded.descriptor.sha256_hex);

    std::ifstream input(output_path, std::ios::binary);
    const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                         std::istreambuf_iterator<char>()};
    CHECK(downloaded_payload == payload);

    const RuntimeStats stats = client.GetRuntimeStats();
    REQUIRE(stats.ok);
    CHECK(stats.artifact_uploads_total >= 1);
    CHECK(stats.artifact_downloads_total >= 1);
    CHECK(stats.artifact_bytes_received_total >= payload.size());
    CHECK(stats.artifact_bytes_sent_total >= payload.size());

    std::error_code error;
    std::filesystem::remove(input_path, error);
    std::filesystem::remove(output_path, error);
}

TEST_CASE("Client artifact download rejects expired artifacts", "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-artifact-expiring.bin";
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-artifact-expired-output.bin";
    const std::string payload = "short-lived-artifact";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }

    ArtifactUpload upload;
    upload.file_path = input_path.string();
    upload.chunk_bytes = 8;
    upload.descriptor.content_type = "application/octet-stream";
    upload.descriptor.ttl_ms = 1;

    const ArtifactTransferResult uploaded = client.UploadArtifact(upload);
    REQUIRE(uploaded.ok);
    std::this_thread::sleep_for(std::chrono::milliseconds{10});

    const ArtifactTransferResult downloaded =
        client.DownloadArtifact(uploaded.descriptor.artifact_id, output_path.string());
    CHECK_FALSE(downloaded.ok);
    CHECK(downloaded.error.code == RpcStatusCode::kNotFound);
    CHECK(downloaded.error.user_message.find("expired") != std::string::npos);

    const RuntimeStats stats = client.GetRuntimeStats();
    REQUIRE(stats.ok);
    CHECK(stats.artifact_failures_total >= 1);

    std::error_code error;
    std::filesystem::remove(input_path, error);
    std::filesystem::remove(output_path, error);
}

TEST_CASE("Client routes data messages to configured peer agent", "[client][integration][data]") {
    const testsupport::DevMtlsPaths paths = testsupport::MakeDevMtlsPaths();
    swarmkit::agent::AgentConfig target_config;
    target_config.agent_id = "drone-2";
    target_config.safety.allow_unsafe_bench_commands = true;
    target_config.security.root_ca_cert_path = paths.root_ca_cert_path;
    target_config.security.cert_chain_path = paths.server_cert_chain_path;
    target_config.security.private_key_path = paths.server_private_key_path;
    testsupport::AgentServerHarness target(target_config);

    swarmkit::agent::AgentConfig source_config;
    source_config.agent_id = "drone-1";
    source_config.safety.allow_unsafe_bench_commands = true;
    source_config.security.root_ca_cert_path = paths.root_ca_cert_path;
    source_config.security.cert_chain_path = paths.server_cert_chain_path;
    source_config.security.private_key_path = paths.server_private_key_path;
    swarmkit::agent::DataPeerConfig peer;
    peer.drone_id = "drone-2";
    peer.address = target.Address();
    peer.transport_security = swarmkit::core::TransportSecurityMode::kMutualTls;
    peer.root_ca_cert_path = paths.root_ca_cert_path;
    peer.cert_chain_path = paths.client_cert_chain_path;
    peer.private_key_path = paths.client_private_key_path;
    peer.server_authority_override = paths.server_authority_override;
    source_config.data.peers.push_back(peer);
    testsupport::AgentServerHarness source(source_config);

    Client source_client = MakeClient(source.Address());
    Client target_client = MakeClient(target.Address());

    std::mutex mutex;
    std::vector<DataMessage> messages;
    auto stream = target_client.StartMessages(
        {.subscriber_id = "drone-2", .topics = {"rotor.event"}, .target_id = "drone-2"},
        [&](const DataMessage& message) {
            std::lock_guard<std::mutex> lock(mutex);
            messages.push_back(message);
        });
    REQUIRE(stream.has_value());

    DataMessage message;
    message.topic = "rotor.event";
    message.target_id = "drone-2";
    message.labels["edge_id"] = "edge-routed";
    message.payload = R"({"event":"routed"})";
    const PublishMessageResult routed = source_client.SendMessageToDrone(std::move(message));
    REQUIRE(routed.ok);
    CHECK(routed.message.find("delivered") != std::string::npos);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(mutex);
            return !messages.empty();
        },
        kWaitTimeout));

    {
        std::lock_guard<std::mutex> lock(mutex);
        REQUIRE(messages.size() == 1);
        CHECK(messages.front().topic == "rotor.event");
        CHECK(messages.front().source_id == "test-client");
        CHECK(messages.front().target_id == "drone-2");
        CHECK(messages.front().labels.at("edge_id") == "edge-routed");
        CHECK(messages.front().payload == R"({"event":"routed"})");
    }
    stream->Stop();
}

TEST_CASE("Client routes artifacts to configured peer agent", "[client][integration][data]") {
    const testsupport::DevMtlsPaths paths = testsupport::MakeDevMtlsPaths();
    const std::filesystem::path target_store =
        std::filesystem::temp_directory_path() / "swarmkit-peer-artifacts-drone2";
    std::error_code cleanup_error;
    std::filesystem::remove_all(target_store, cleanup_error);

    swarmkit::agent::AgentConfig target_config;
    target_config.agent_id = "drone-2";
    target_config.data.artifact_dir = target_store.string();
    target_config.safety.allow_unsafe_bench_commands = true;
    target_config.security.root_ca_cert_path = paths.root_ca_cert_path;
    target_config.security.cert_chain_path = paths.server_cert_chain_path;
    target_config.security.private_key_path = paths.server_private_key_path;
    testsupport::AgentServerHarness target(target_config);

    swarmkit::agent::AgentConfig source_config;
    source_config.agent_id = "drone-1";
    source_config.safety.allow_unsafe_bench_commands = true;
    source_config.security.root_ca_cert_path = paths.root_ca_cert_path;
    source_config.security.cert_chain_path = paths.server_cert_chain_path;
    source_config.security.private_key_path = paths.server_private_key_path;
    swarmkit::agent::DataPeerConfig peer;
    peer.drone_id = "drone-2";
    peer.address = target.Address();
    peer.transport_security = swarmkit::core::TransportSecurityMode::kMutualTls;
    peer.root_ca_cert_path = paths.root_ca_cert_path;
    peer.cert_chain_path = paths.client_cert_chain_path;
    peer.private_key_path = paths.client_private_key_path;
    peer.server_authority_override = paths.server_authority_override;
    source_config.data.peers.push_back(peer);
    testsupport::AgentServerHarness source(source_config);

    Client source_client = MakeClient(source.Address());
    Client target_client = MakeClient(target.Address());

    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-peer-camera-frame.jpg";
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-peer-camera-frame-out.jpg";
    const std::string payload = "camera-frame-from-drone-1";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }

    ArtifactUpload upload;
    upload.file_path = input_path.string();
    upload.chunk_bytes = 7;
    upload.descriptor.target_id = "drone-2";
    upload.descriptor.content_type = "image/jpeg";
    upload.descriptor.labels["camera_id"] = "front";
    upload.descriptor.labels["frame_id"] = "123";

    const ArtifactTransferResult sent = source_client.SendArtifactToDrone(upload);
    REQUIRE(sent.ok);
    REQUIRE_FALSE(sent.descriptor.artifact_id.empty());
    CHECK(sent.descriptor.target_id == "drone-2");
    CHECK(sent.descriptor.labels.at("frame_id") == "123");

    const ArtifactTransferResult downloaded =
        target_client.DownloadArtifact(sent.descriptor.artifact_id, output_path.string());
    REQUIRE(downloaded.ok);
    CHECK(downloaded.descriptor.sha256_hex == sent.descriptor.sha256_hex);

    std::ifstream input(output_path, std::ios::binary);
    const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                         std::istreambuf_iterator<char>()};
    CHECK(downloaded_payload == payload);

    std::filesystem::remove(input_path, cleanup_error);
    std::filesystem::remove(output_path, cleanup_error);
    std::filesystem::remove_all(target_store, cleanup_error);
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
        stream_started.store(
            testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
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
        frame.validity.armed = true;
        frame.validity.landed = true;
        frame.validity.relative_altitude = true;
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
    auto authority_watch = operator_client.StartAuthorityWatch(
        {.drone_id = "drone-1", .priority = commands::CommandPriority::kOperator},
        [&](const AuthorityEventInfo& event) {
            std::lock_guard<std::mutex> lock(events_mutex);
            operator_events.push_back(event);
        });
    REQUIRE(authority_watch.has_value());

    {
        const auto kSession = operator_client.AcquireAuthoritySession("drone-1", 500);
        REQUIRE(kSession.has_value());
    }

    const CommandResult kRelock = override_client.LockAuthority("drone-1", 500);
    REQUIRE(kRelock.ok);

    const ReleaseAuthorityResult kRelease = override_client.ReleaseAuthority("drone-1");
    REQUIRE(kRelease.ok);
    CHECK_FALSE(kRelease.correlation_id.empty());
    CHECK(kRelease.error.code == RpcStatusCode::kOk);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(events_mutex);
            return !operator_events.empty();
        },
        kWaitTimeout));

    authority_watch->Stop();
}

TEST_CASE("Client release authority returns checked transport failures",
          "[client][integration][authority]") {
    ClientConfig config = testsupport::MakeMtlsClientConfig("127.0.0.1:1");
    config.deadline_ms = 100;
    config.retry_policy.max_attempts = 1;
    Client client(std::move(config));

    const ReleaseAuthorityResult result = client.ReleaseAuthority("drone-1");

    CHECK_FALSE(result.ok);
    CHECK_FALSE(result.correlation_id.empty());
    CHECK(result.error.domain == core::ErrorDomain::kTransport);
    CHECK(result.error.code != RpcStatusCode::kOk);
    CHECK_FALSE(result.message.empty());
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
    REQUIRE(
        agent_service->SendCommand(&satisfied_context, &satisfied_request, &satisfied_reply).ok());
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

    grpc::ServerContext supervisor_context;
    swarmkit::v1::CommandRequest supervisor_request;
    auto* supervisor_ctx = supervisor_request.mutable_ctx();
    supervisor_ctx->set_drone_id("drone-1");
    supervisor_ctx->set_client_id("supervisor-client");
    supervisor_ctx->set_priority(static_cast<std::int32_t>(commands::CommandPriority::kSupervisor));
    supervisor_request.mutable_cmd()->mutable_set_mode()->set_mode("loiter");
    swarmkit::v1::CommandReply supervisor_reply;
    REQUIRE(agent_service->SendCommand(&supervisor_context, &supervisor_request, &supervisor_reply)
                .ok());
    REQUIRE(supervisor_reply.status() == swarmkit::v1::CommandReply::OK);
    CHECK(backend_ptr->ExecuteCallCount() == 2);

    grpc::ServerContext resumed_context;
    swarmkit::v1::CommandRequest resumed_request;
    auto* resumed_ctx = resumed_request.mutable_ctx();
    resumed_ctx->set_drone_id("drone-1");
    resumed_ctx->set_client_id("operator-client");
    resumed_ctx->set_priority(static_cast<std::int32_t>(commands::CommandPriority::kOperator));
    resumed_request.mutable_cmd()->mutable_set_mode()->set_mode("auto");
    swarmkit::v1::CommandReply resumed_reply;
    REQUIRE(agent_service->SendCommand(&resumed_context, &resumed_request, &resumed_reply).ok());
    REQUIRE(resumed_reply.status() == swarmkit::v1::CommandReply::OK);
    CHECK(backend_ptr->ExecuteCallCount() == 3);
}

TEST_CASE("Client telemetry subscription receives frames and can stop cleanly",
          "[client][integration][telemetry]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::atomic<int> frame_count{0};
    std::mutex received_mutex;
    std::optional<core::TelemetryFrame> received_frame;
    auto telemetry_stream = client.StartTelemetry(
        {.drone_id = "drone-1", .rate_hertz = 5}, [&](const core::TelemetryFrame& frame) {
            if (frame.drone_id == "drone-1") {
                std::lock_guard<std::mutex> lock(received_mutex);
                received_frame = frame;
                frame_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    REQUIRE(telemetry_stream.has_value());

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
    frame.validity.position = true;
    frame.validity.relative_altitude = true;
    frame.validity.battery = true;
    frame.validity.mode = true;
    frame.source_unix_time_ms = 100;
    frame.position_frame = core::CoordinateFrame::kWgs84;
    frame.velocity_frame = core::CoordinateFrame::kLocalNed;
    frame.gps_quality = core::GpsQuality::kFix3D;
    frame.validity.gps = true;
    frame.accuracy.horizontal_position_valid = true;
    frame.accuracy.horizontal_position_m = 1.5F;
    frame.estimator_state = core::EstimatorState::kHealthy;
    frame.validity.estimator = true;
    frame.active_goal_id = "goal-123";
    frame.correlation_id = "corr-123";

    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", frame);
            return frame_count.load(std::memory_order_relaxed) >= 1;
        },
        kWaitTimeout, std::chrono::milliseconds{50}));
    {
        std::lock_guard<std::mutex> lock(received_mutex);
        REQUIRE(received_frame.has_value());
        const core::TelemetryFrame frame = received_frame.value_or(core::TelemetryFrame{});
        CHECK(frame.HasPosition());
        CHECK(frame.source_unix_time_ms == 100);
        CHECK(frame.position_frame == core::CoordinateFrame::kWgs84);
        CHECK(frame.gps_quality == core::GpsQuality::kFix3D);
        CHECK(frame.accuracy.horizontal_position_valid);
        CHECK(frame.accuracy.horizontal_position_m == 1.5F);
        CHECK(frame.estimator_state == core::EstimatorState::kHealthy);
        CHECK(frame.active_goal_id == "goal-123");
        CHECK(frame.correlation_id == "corr-123");
    }

    telemetry_stream->Stop();
    REQUIRE(testsupport::WaitUntil([&] { return !harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));
}

TEST_CASE("Client telemetry subscription handle contains callback exceptions",
          "[client][integration][telemetry]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    auto invalid_subscription = client.StartTelemetry({.drone_id = "drone-1", .rate_hertz = 0},
                                                      [](const core::TelemetryFrame&) {});
    REQUIRE_FALSE(invalid_subscription.has_value());
    CHECK(invalid_subscription.error().code == RpcStatusCode::kInvalidArgument);

    std::atomic<bool> callback_error_seen{false};
    std::atomic<bool> connected_seen{false};

    SubscriptionOptions options;
    options.backpressure.max_pending_callbacks = 4;
    options.backpressure.policy = StreamBackpressurePolicy::kDropOldest;

    auto subscription = client.StartTelemetry(
        {.drone_id = "drone-1", .rate_hertz = 5},
        [](const core::TelemetryFrame&) { throw std::runtime_error("boom"); },
        [&](const std::string& message) {
            if (message.find("boom") != std::string::npos) {
                callback_error_seen.store(true, std::memory_order_relaxed);
            }
        },
        [&](const SubscriptionEvent& event) {
            if (event.state == SubscriptionLifecycleState::kConnected) {
                connected_seen.store(true, std::memory_order_relaxed);
            }
        },
        options);

    REQUIRE(subscription.has_value());
    REQUIRE(subscription->IsActive());
    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    core::TelemetryFrame frame;
    frame.drone_id = "drone-1";
    frame.unix_time_ms = 456;
    harness.Backend().EmitTelemetry("drone-1", frame);

    REQUIRE(testsupport::WaitUntil(
        [&] { return callback_error_seen.load(std::memory_order_relaxed); }, kWaitTimeout));
    REQUIRE(testsupport::WaitUntil([&] { return connected_seen.load(std::memory_order_relaxed); },
                                   kWaitTimeout));

    subscription->Stop();
    CHECK_FALSE(subscription->IsActive());
}

TEST_CASE("Client active goal emits active and reached reports", "[client][integration][goal]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto reports_stream =
        client.StartReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
            std::lock_guard<std::mutex> lock(reports_mutex);
            reports.push_back(report);
        });
    REQUIRE(reports_stream.has_value());

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-reached";
    goal.revision = 7;
    goal.target = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    goal.acceptance_radius_m = 5.0F;
    goal.deviation_radius_m = 25.0F;
    goal.timeout_ms = 5000;
    goal.labels = {
        {"from_node", "node-a"},
        {"to_node", "node-b"},
        {"edge_id", "edge-42"},
    };

    const GoalResult result = client.SetActiveGoal(goal);
    REQUIRE(result.ok);
    CHECK(result.goal.goal_id == "goal-reached");
    CHECK(result.goal.labels.at("from_node") == "node-a");
    CHECK(result.goal.labels.at("to_node") == "node-b");
    CHECK(result.goal.labels.at("edge_id") == "edge-42");
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
    frame.validity.position = true;
    frame.validity.relative_altitude = true;
    frame.validity.battery = true;
    frame.validity.mode = true;

    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", frame);
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [](const AgentReport& report) {
                return report.goal.has_value() && report.goal->status == GoalStatus::kReached &&
                       report.goal->goal_id == "goal-reached";
            });
        },
        kWaitTimeout, std::chrono::milliseconds{50}));

    const ActiveGoalStatus status = client.GetActiveGoal("drone-1");
    REQUIRE(status.has_goal);
    CHECK(status.status == GoalStatus::kReached);
    CHECK(status.goal.labels.at("from_node") == "node-a");
    CHECK(status.goal.labels.at("to_node") == "node-b");
    CHECK(status.goal.labels.at("edge_id") == "edge-42");

    reports_stream->Stop();
}

TEST_CASE("Client can cancel active goal and receive cancellation report",
          "[client][integration][goal]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto reports_stream =
        client.StartReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
            std::lock_guard<std::mutex> lock(reports_mutex);
            reports.push_back(report);
        });
    REQUIRE(reports_stream.has_value());

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
                return report.goal.has_value() && report.goal->status == GoalStatus::kCancelled &&
                       report.goal->goal_id == "goal-cancel";
            });
        },
        kWaitTimeout));

    const ActiveGoalStatus status = client.GetActiveGoal("drone-1");
    CHECK_FALSE(status.has_goal);

    reports_stream->Stop();
}

TEST_CASE("Client active goal accepts local-NED shape and reports unsupported execution",
          "[client][integration][goal]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "local-goal";
    goal.revision = 9;
    goal.use_local_target = true;
    goal.target_frame = "local-ned";
    goal.local_target = {.x_m = 5.0, .y_m = -2.0, .z_m = -1.0};
    goal.acceptance_radius_m = 1.0F;
    goal.deviation_radius_m = 5.0F;

    const GoalResult result = client.SetActiveGoal(goal);
    REQUIRE_FALSE(result.ok);
    CHECK(result.goal.use_local_target);
    CHECK(result.goal.target_frame == "local-ned");
    CHECK(result.goal.local_target.x_m == 5.0);
    CHECK(result.message.find("local-ned") != std::string::npos);
    CHECK(harness.Backend().ExecuteCallCount() == 0);
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
