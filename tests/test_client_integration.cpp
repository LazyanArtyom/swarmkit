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
#include <utility>
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

[[nodiscard]] std::unique_ptr<swarmkit::v1::DataService::Stub> MakeDataServiceStub(
    const std::string& address) {
    const testsupport::DevMtlsPaths paths = testsupport::MakeDevMtlsPaths();
    grpc::SslCredentialsOptions options;
    static_cast<void>(
        core::internal::ReadTextFile(paths.root_ca_cert_path, &options.pem_root_certs));
    static_cast<void>(
        core::internal::ReadTextFile(paths.client_private_key_path, &options.pem_private_key));
    static_cast<void>(
        core::internal::ReadTextFile(paths.client_cert_chain_path, &options.pem_cert_chain));
    grpc::ChannelArguments arguments;
    arguments.SetSslTargetNameOverride(paths.server_authority_override);
    return swarmkit::v1::DataService::NewStub(
        grpc::CreateCustomChannel(address, grpc::SslCredentials(options), arguments));
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

    const CapabilitiesResult capabilities = client.GetCapabilities();
    REQUIRE(capabilities.ok);
    CHECK(capabilities.agent_id == "test-agent");
    CHECK(capabilities.backend.autopilot_type == "recording");
    CHECK(capabilities.backend.supports_velocity_control);
    CHECK_FALSE(capabilities.backend.supports_payload_control);
    CHECK(std::ranges::contains(capabilities.backend.supported_modes, "guided"));
    CHECK_FALSE(capabilities.backend.max_horizontal_speed.has_value());
    CHECK_FALSE(capabilities.backend.max_climb_speed.has_value());
    CHECK_FALSE(capabilities.backend.max_descent_speed.has_value());
    CHECK_FALSE(capabilities.backend.max_altitude.has_value());
    CHECK(capabilities.backend.evidence.position_estimate == core::CapabilitySupport::kUnknown);
    CHECK(capabilities.backend.evidence.active_goal_lineage == core::CapabilitySupport::kSupported);
    CHECK(capabilities.backend.evidence.telemetry_sequence == core::CapabilitySupport::kSupported);
    CHECK(capabilities.backend.evidence.telemetry_replay == core::CapabilitySupport::kUnsupported);
}

TEST_CASE("Client command reports are replayable and delivered live",
          "[client][integration][reports]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    commands::CommandEnvelope envelope;
    envelope.context.drone_id = "drone-1";
    envelope.context.client_id = "test-client";
    envelope.context.priority = commands::CommandPriority::kSupervisor;
    envelope.context.execution_context = core::ExecutionContext{
        .mission_id = "mission-command-reports",
        .mission_revision = 1,
        .model_hash = "sha256:command-model",
        .operation_id = "command-operation",
        .operation_attempt_revision = 1,
    };
    envelope.command = commands::FlightCmd{commands::CmdSetMode{.mode = "guided"}};

    const CommandResult replayed_command = client.SendCommand(envelope);
    REQUIRE(replayed_command.ok);

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto reports_stream = client.StartReports({.drone_id = "drone-1", .after_sequence = 0},
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
    {
        std::lock_guard<std::mutex> lock(reports_mutex);
        const auto report = std::ranges::find_if(reports, [&](const AgentReport& candidate) {
            return candidate.correlation_id == replayed_command.correlation_id &&
                   candidate.type == AgentReportType::kCommandAcked;
        });
        REQUIRE(report != reports.end());
        const auto* context = std::get_if<core::ExecutionContext>(&report->execution_binding);
        REQUIRE(context != nullptr);
        CHECK(*context == *envelope.context.execution_context);
    }

    envelope.command = commands::FlightCmd{commands::CmdSetMode{.mode = "loiter"}};
    const CommandResult live_command = client.SendCommand(envelope);
    REQUIRE(live_command.ok);

    REQUIRE(testsupport::WaitUntil(
        [&] { return has_report(live_command.correlation_id, AgentReportType::kCommandAcked); },
        kWaitTimeout));

    reports_stream->Stop();
}

TEST_CASE("Typed evidence capabilities and motion provenance survive the SDK boundary",
          "[client][integration][capabilities]") {
    testsupport::AgentServerHarness harness;
    core::BackendCapabilities declared;
    declared.backend_name = "scripted";
    declared.evidence.source_timestamp = core::CapabilitySupport::kSupported;
    declared.evidence.source_clock_domains = {core::ClockDomain::kVehicleBoot};
    declared.evidence.position_estimate = core::CapabilitySupport::kSupported;
    declared.evidence.horizontal_position_uncertainty = core::CapabilitySupport::kUnsupported;
    declared.evidence.vertical_position_uncertainty = core::CapabilitySupport::kUnknown;
    declared.evidence.horizontal_velocity = core::CapabilitySupport::kSupported;
    declared.evidence.vertical_velocity = core::CapabilitySupport::kSupported;
    declared.evidence.uncertainty_semantics = core::CapabilitySupport::kSupported;
    declared.evidence.estimator_health = core::CapabilitySupport::kUnsupported;
    declared.evidence.failsafe_state = core::CapabilitySupport::kSupported;
    declared.max_horizontal_speed = core::MotionLimit{
        .value = 4.5F,
        .semantics = core::MotionLimitSemantics::kConfiguredCommandLimit,
        .source = "vehicle-profile.speed_xy",
        .profile_id = "quad-a",
        .profile_version = "3",
    };
    harness.Backend().SetCapabilities(std::move(declared));

    Client client = MakeClient(harness.Address());
    const CapabilitiesResult actual = client.GetCapabilities();
    REQUIRE(actual.ok);
    CHECK(actual.backend.backend_name == "scripted");
    CHECK(actual.backend.evidence.source_timestamp == core::CapabilitySupport::kSupported);
    REQUIRE(actual.backend.evidence.source_clock_domains.size() == 1);
    CHECK(actual.backend.evidence.source_clock_domains.front() == core::ClockDomain::kVehicleBoot);
    CHECK(actual.backend.evidence.horizontal_position_uncertainty ==
          core::CapabilitySupport::kUnsupported);
    CHECK(actual.backend.evidence.vertical_position_uncertainty ==
          core::CapabilitySupport::kUnknown);
    CHECK(actual.backend.evidence.estimator_health == core::CapabilitySupport::kUnsupported);
    CHECK(actual.backend.evidence.active_goal_lineage == core::CapabilitySupport::kSupported);
    CHECK(actual.backend.evidence.telemetry_sequence == core::CapabilitySupport::kSupported);
    CHECK(actual.backend.evidence.telemetry_replay == core::CapabilitySupport::kUnsupported);
    REQUIRE(actual.backend.max_horizontal_speed.has_value());
    CHECK(actual.backend.max_horizontal_speed->value == 4.5F);
    CHECK(actual.backend.max_horizontal_speed->semantics ==
          core::MotionLimitSemantics::kConfiguredCommandLimit);
    CHECK(actual.backend.max_horizontal_speed->source == "vehicle-profile.speed_xy");
    CHECK(actual.backend.max_horizontal_speed->profile_id == "quad-a");
    CHECK(actual.backend.max_horizontal_speed->profile_version == "3");
}

TEST_CASE("Client data messages publish and subscribe independently",
          "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    Client publisher = MakeClient(harness.Address());
    Client subscriber = MakeClient(harness.Address());

    std::mutex mutex;
    std::vector<DataMessage> messages;
    auto stream =
        subscriber.StartMessages({.subscriber_id = "subscriber-1", .topics = {"rotor.event"}},
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
        CHECK(messages.front().message_id.rfind("msg-test-client-", 0) == 0);
        CHECK(messages.front().message_id.find("msg-1") == std::string::npos);
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
    Client subscriber = MakeClient(harness.Address());

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

    std::mutex mutex;
    std::vector<DataMessage> events;
    auto stream = subscriber.StartMessages(
        {.subscriber_id = "artifact-listener", .topics = {"swarmkit.artifact.received"}},
        [&](const DataMessage& message) {
            std::lock_guard<std::mutex> lock(mutex);
            events.push_back(message);
        });
    REQUIRE(stream.has_value());

    const ArtifactTransferResult uploaded = client.UploadArtifact(upload);
    REQUIRE(uploaded.ok);
    REQUIRE_FALSE(uploaded.descriptor.artifact_id.empty());
    CHECK(std::cmp_equal(uploaded.descriptor.size_bytes, payload.size()));
    CHECK(uploaded.descriptor.content_type == "image/jpeg");
    CHECK(uploaded.descriptor.labels.at("camera_id") == "front");
    CHECK_FALSE(uploaded.descriptor.sha256_hex.empty());

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(mutex);
            return std::ranges::any_of(events, [&](const DataMessage& event) {
                const auto iter = event.labels.find("artifact_id");
                return event.topic == "swarmkit.artifact.received" && iter != event.labels.end() &&
                       iter->second == uploaded.descriptor.artifact_id;
            });
        },
        kWaitTimeout));
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto iter = std::ranges::find_if(events, [&](const DataMessage& event) {
            const auto artifact = event.labels.find("artifact_id");
            return artifact != event.labels.end() &&
                   artifact->second == uploaded.descriptor.artifact_id;
        });
        REQUIRE(iter != events.end());
        CHECK(iter->labels.at("content_type") == "image/jpeg");
        CHECK(iter->labels.at("source_id") == "test-client");
        CHECK(iter->labels.at("artifact.label.camera_id") == "front");
    }

    const ArtifactListResult listed = client.ListArtifacts();
    REQUIRE(listed.ok);
    CHECK(std::ranges::any_of(listed.artifacts, [&](const ArtifactDescriptor& descriptor) {
        return descriptor.artifact_id == uploaded.descriptor.artifact_id &&
               descriptor.content_type == "image/jpeg";
    }));

    const ArtifactTransferResult info = client.GetArtifact(uploaded.descriptor.artifact_id);
    REQUIRE(info.ok);
    CHECK(info.descriptor.artifact_id == uploaded.descriptor.artifact_id);
    CHECK(info.descriptor.sha256_hex == uploaded.descriptor.sha256_hex);

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

    stream->Stop();
    std::error_code error;
    std::filesystem::remove(input_path, error);
    std::filesystem::remove(output_path, error);
}

TEST_CASE("Agent reloads durable artifact index from storage", "[client][integration][data]") {
    const std::filesystem::path store =
        std::filesystem::temp_directory_path() / "swarmkit-durable-artifacts";
    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-durable-input.bin";
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-durable-output.bin";
    std::error_code error;
    std::filesystem::remove_all(store, error);
    const std::string payload = "durable-frame-bytes";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }

    swarmkit::agent::AgentConfig config;
    config.agent_id = "durable-agent";
    config.data.artifact_dir = store.string();
    config.safety.allow_unsafe_bench_commands = true;
    const testsupport::DevMtlsPaths paths = testsupport::MakeDevMtlsPaths();
    config.security.root_ca_cert_path = paths.root_ca_cert_path;
    config.security.cert_chain_path = paths.server_cert_chain_path;
    config.security.private_key_path = paths.server_private_key_path;

    std::string artifact_id;
    {
        testsupport::AgentServerHarness harness(config);
        Client client = MakeClient(harness.Address());
        ArtifactUpload upload;
        upload.file_path = input_path.string();
        upload.descriptor.content_type = "application/octet-stream";
        const ArtifactTransferResult uploaded = client.UploadArtifact(upload);
        REQUIRE(uploaded.ok);
        artifact_id = uploaded.descriptor.artifact_id;
    }

    {
        testsupport::AgentServerHarness harness(config);
        Client client = MakeClient(harness.Address());
        const ArtifactListResult listed = client.ListArtifacts();
        REQUIRE(listed.ok);
        CHECK(std::ranges::any_of(listed.artifacts, [&](const ArtifactDescriptor& descriptor) {
            return descriptor.artifact_id == artifact_id;
        }));
        const ArtifactTransferResult downloaded =
            client.DownloadArtifact(artifact_id, output_path.string());
        REQUIRE(downloaded.ok);
        std::ifstream input(output_path, std::ios::binary);
        const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                             std::istreambuf_iterator<char>()};
        CHECK(downloaded_payload == payload);
    }

    std::filesystem::remove(input_path, error);
    std::filesystem::remove(output_path, error);
    std::filesystem::remove_all(store, error);
}

TEST_CASE("Client resumes artifact download from partial file", "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());
    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-resume-input.bin";
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-resume-output.bin";
    const std::filesystem::path partial_path =
        output_path.parent_path() / (output_path.filename().string() + ".part");
    const std::string payload = "resumable-artifact-payload";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }
    {
        std::ofstream partial(partial_path, std::ios::binary | std::ios::trunc);
        partial << payload.substr(0, 9);
    }
    ArtifactUpload upload;
    upload.file_path = input_path.string();
    upload.chunk_bytes = 5;
    const ArtifactTransferResult uploaded = client.UploadArtifact(upload);
    REQUIRE(uploaded.ok);

    const ArtifactTransferResult resumed =
        client.ResumeArtifactDownload(uploaded.descriptor.artifact_id, output_path.string());
    REQUIRE(resumed.ok);
    std::ifstream input(output_path, std::ios::binary);
    const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                         std::istreambuf_iterator<char>()};
    CHECK(downloaded_payload == payload);

    std::error_code error;
    std::filesystem::remove(input_path, error);
    std::filesystem::remove(output_path, error);
    std::filesystem::remove(partial_path, error);
}

TEST_CASE("Artifact upload rejects malformed chunk ordering", "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    auto stub = MakeDataServiceStub(harness.Address());

    grpc::ClientContext context;
    swarmkit::v1::ArtifactDescriptor reply;
    auto writer = stub->UploadArtifact(&context, &reply);

    swarmkit::v1::ArtifactChunk chunk;
    chunk.set_transfer_id("bad-chunk-order-test");
    chunk.set_offset(0);
    chunk.set_chunk_index(1);
    chunk.set_data("bad-order");
    chunk.set_final_chunk(true);
    auto* descriptor = chunk.mutable_artifact();
    descriptor->set_source_id("test-client");
    descriptor->set_content_type("application/octet-stream");
    descriptor->set_filename("bad-order.bin");
    descriptor->set_size_bytes(static_cast<std::int64_t>(chunk.data().size()));

    REQUIRE(writer->Write(chunk));
    writer->WritesDone();
    const grpc::Status status = writer->Finish();

    CHECK_FALSE(status.ok());
    CHECK(status.error_code() == grpc::StatusCode::INVALID_ARGUMENT);
    CHECK(status.error_message().find("matching indexes") != std::string::npos);
}

TEST_CASE("Client resumes artifact upload sessions and commits artifact",
          "[client][integration][data]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-upload-session-output.bin";
    const std::string payload = "session-upload-resume-payload";

    ArtifactDescriptor descriptor;
    descriptor.source_id = "test-client";
    descriptor.content_type = "application/octet-stream";
    descriptor.filename = "session-upload.bin";
    descriptor.size_bytes = static_cast<std::int64_t>(payload.size());

    const ArtifactUploadSessionResult created = client.CreateArtifactUpload(descriptor);
    REQUIRE(created.ok);
    REQUIRE_FALSE(created.upload.upload_id.empty());
    CHECK(created.upload.bytes_received == 0);

    const std::string first = payload.substr(0, 9);
    const ArtifactUploadSessionResult first_chunk =
        client.UploadArtifactChunk(created.upload.upload_id, first, 0, 0);
    REQUIRE(first_chunk.ok);
    CHECK(std::cmp_equal(first_chunk.upload.bytes_received, first.size()));
    CHECK(first_chunk.upload.next_chunk_index == 1);

    const ArtifactUploadSessionResult status = client.GetUploadStatus(created.upload.upload_id);
    REQUIRE(status.ok);
    CHECK(std::cmp_equal(status.upload.bytes_received, first.size()));

    const std::string second = payload.substr(first.size());
    const ArtifactUploadSessionResult second_chunk = client.UploadArtifactChunk(
        created.upload.upload_id, second, static_cast<std::int64_t>(first.size()), 1);
    REQUIRE(second_chunk.ok);
    CHECK(std::cmp_equal(second_chunk.upload.bytes_received, payload.size()));

    const ArtifactUploadSessionResult committed =
        client.CommitArtifactUpload(created.upload.upload_id);
    REQUIRE(committed.ok);
    REQUIRE_FALSE(committed.descriptor.artifact_id.empty());
    CHECK(std::cmp_equal(committed.descriptor.size_bytes, payload.size()));

    const ArtifactTransferResult downloaded =
        client.DownloadArtifact(committed.descriptor.artifact_id, output_path.string());
    REQUIRE(downloaded.ok);
    std::ifstream input(output_path, std::ios::binary);
    const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                         std::istreambuf_iterator<char>()};
    CHECK(downloaded_payload == payload);

    std::error_code error;
    std::filesystem::remove(output_path, error);
}

TEST_CASE("Client parallel duplicate artifact uploads are idempotent",
          "[client][integration][data]") {
    swarmkit::agent::AgentConfig config;
    const testsupport::DevMtlsPaths paths = testsupport::MakeDevMtlsPaths();
    config.agent_id = "artifact-agent";
    config.data.artifact_dir =
        (std::filesystem::temp_directory_path() / "swarmkit-idempotent-artifacts").string();
    config.safety.allow_unsafe_bench_commands = true;
    config.security.root_ca_cert_path = paths.root_ca_cert_path;
    config.security.cert_chain_path = paths.server_cert_chain_path;
    config.security.private_key_path = paths.server_private_key_path;
    std::error_code error;
    std::filesystem::remove_all(config.data.artifact_dir, error);
    testsupport::AgentServerHarness harness(config);

    Client first = MakeClient(harness.Address());
    Client second = MakeClient(harness.Address());
    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-idempotent-frame.jpg";
    const std::string payload = "same-camera-frame";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }

    ArtifactUpload upload;
    upload.file_path = input_path.string();
    upload.chunk_bytes = 4;
    upload.descriptor.content_type = "image/jpeg";

    ArtifactTransferResult first_result;
    ArtifactTransferResult second_result;
    std::thread first_thread([&] { first_result = first.UploadArtifact(upload); });
    std::thread second_thread([&] { second_result = second.UploadArtifact(upload); });
    first_thread.join();
    second_thread.join();

    REQUIRE(first_result.ok);
    REQUIRE(second_result.ok);
    CHECK(first_result.descriptor.artifact_id == second_result.descriptor.artifact_id);
    CHECK(first_result.descriptor.sha256_hex == second_result.descriptor.sha256_hex);

    std::filesystem::remove(input_path, error);
    std::filesystem::remove_all(config.data.artifact_dir, error);
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
    CHECK(downloaded.error.code == core::ErrorCode::kNotFound);
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

TEST_CASE("Client refreshes configured data peer reachability", "[client][integration][data]") {
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
    const DataPeerListResult peers = source_client.RefreshDataPeers();
    REQUIRE(peers.ok);
    REQUIRE(peers.peers.size() == 1);
    CHECK(peers.peers.front().drone_id == "drone-2");
    CHECK(peers.peers.front().state == DataPeerState::kReady);
    CHECK(peers.peers.front().last_success_unix_ms > 0);
    CHECK(peers.peers.front().message.find("ready") != std::string::npos);
}

TEST_CASE("Client updates runtime data peer registry", "[client][integration][data]") {
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
    testsupport::AgentServerHarness source(source_config);

    Client source_client = MakeClient(source.Address());
    DataPeerConfig peer;
    peer.drone_id = "drone-2";
    peer.address = target.Address();
    peer.transport_security = "mtls";
    peer.root_ca_cert_path = paths.root_ca_cert_path;
    peer.cert_chain_path = paths.client_cert_chain_path;
    peer.private_key_path = paths.client_private_key_path;
    peer.server_authority_override = paths.server_authority_override;

    const DataPeerListResult upserted = source_client.UpsertDataPeer(peer);
    REQUIRE(upserted.ok);
    REQUIRE(upserted.peers.size() == 1);
    CHECK(upserted.peers.front().drone_id == "drone-2");

    const DataPeerListResult refreshed = source_client.RefreshDataPeers({"drone-2"});
    REQUIRE(refreshed.ok);
    REQUIRE(refreshed.peers.size() == 1);
    CHECK(refreshed.peers.front().state == DataPeerState::kReady);

    const DataPeerListResult removed = source_client.RemoveDataPeer("drone-2");
    REQUIRE(removed.ok);
    CHECK(removed.peers.empty());
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

    std::mutex event_mutex;
    std::vector<DataMessage> events;
    auto event_stream = target_client.StartMessages(
        {.subscriber_id = "target-artifact-listener", .topics = {"swarmkit.artifact.received"}},
        [&](const DataMessage& message) {
            std::lock_guard<std::mutex> lock(event_mutex);
            events.push_back(message);
        });
    REQUIRE(event_stream.has_value());

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

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(event_mutex);
            return std::ranges::any_of(events, [&](const DataMessage& event) {
                const auto artifact = event.labels.find("artifact_id");
                return event.topic == "swarmkit.artifact.received" &&
                       artifact != event.labels.end() &&
                       artifact->second == sent.descriptor.artifact_id;
            });
        },
        kWaitTimeout));
    {
        std::lock_guard<std::mutex> lock(event_mutex);
        const auto iter = std::ranges::find_if(events, [&](const DataMessage& event) {
            const auto artifact = event.labels.find("artifact_id");
            return artifact != event.labels.end() &&
                   artifact->second == sent.descriptor.artifact_id;
        });
        REQUIRE(iter != events.end());
        CHECK(iter->source_id == "drone-2");
        CHECK(iter->labels.at("source_id") == "test-client");
        CHECK(iter->labels.at("target_id") == "drone-2");
        CHECK(iter->labels.at("content_type") == "image/jpeg");
        CHECK(iter->labels.at("artifact.label.frame_id") == "123");
    }

    const ArtifactListResult listed = target_client.ListArtifacts({
        .source_id = "test-client",
        .target_id = "drone-2",
    });
    REQUIRE(listed.ok);
    CHECK(std::ranges::any_of(listed.artifacts, [&](const ArtifactDescriptor& descriptor) {
        return descriptor.artifact_id == sent.descriptor.artifact_id;
    }));

    const ArtifactTransferResult downloaded =
        target_client.DownloadArtifact(sent.descriptor.artifact_id, output_path.string());
    REQUIRE(downloaded.ok);
    CHECK(downloaded.descriptor.sha256_hex == sent.descriptor.sha256_hex);

    std::ifstream input(output_path, std::ios::binary);
    const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                         std::istreambuf_iterator<char>()};
    CHECK(downloaded_payload == payload);

    event_stream->Stop();
    std::filesystem::remove(input_path, cleanup_error);
    std::filesystem::remove(output_path, cleanup_error);
    std::filesystem::remove_all(target_store, cleanup_error);
}

TEST_CASE("Client starts and observes agent-side artifact transfer",
          "[client][integration][data]") {
    const testsupport::DevMtlsPaths paths = testsupport::MakeDevMtlsPaths();
    swarmkit::agent::AgentConfig config;
    config.agent_id = "transfer-agent";
    config.data.artifact_dir =
        (std::filesystem::temp_directory_path() / "swarmkit-agent-side-transfer").string();
    config.safety.allow_unsafe_bench_commands = true;
    config.security.root_ca_cert_path = paths.root_ca_cert_path;
    config.security.cert_chain_path = paths.server_cert_chain_path;
    config.security.private_key_path = paths.server_private_key_path;
    std::error_code cleanup_error;
    std::filesystem::remove_all(config.data.artifact_dir, cleanup_error);
    testsupport::AgentServerHarness harness(config);
    Client client = MakeClient(harness.Address());

    const std::filesystem::path input_path =
        std::filesystem::temp_directory_path() / "swarmkit-agent-transfer-input.jpg";
    const std::filesystem::path output_path =
        std::filesystem::temp_directory_path() / "swarmkit-agent-transfer-output.jpg";
    const std::string payload = "agent-side-camera-frame";
    {
        std::ofstream output(input_path, std::ios::binary | std::ios::trunc);
        output << payload;
    }

    ArtifactUpload upload;
    upload.file_path = input_path.string();
    upload.chunk_bytes = 5;
    upload.descriptor.content_type = "image/jpeg";
    upload.descriptor.labels["camera_id"] = "front";

    const ArtifactTransferStatusResult started = client.StartArtifactTransfer(upload, false);
    REQUIRE(started.ok);
    REQUIRE_FALSE(started.transfer.transfer_id.empty());

    ArtifactTransferStatusResult status;
    REQUIRE(testsupport::WaitUntil(
        [&] {
            status = client.GetArtifactTransfer(started.transfer.transfer_id);
            return status.ok && status.transfer.state == ArtifactTransferState::kCompleted;
        },
        kWaitTimeout));
    CHECK(std::cmp_equal(status.transfer.bytes_transferred, payload.size()));
    REQUIRE_FALSE(status.transfer.descriptor.artifact_id.empty());

    const ArtifactTransferResult downloaded =
        client.DownloadArtifact(status.transfer.descriptor.artifact_id, output_path.string());
    REQUIRE(downloaded.ok);
    std::ifstream input(output_path, std::ios::binary);
    const std::string downloaded_payload{std::istreambuf_iterator<char>(input),
                                         std::istreambuf_iterator<char>()};
    CHECK(downloaded_payload == payload);

    std::filesystem::remove(input_path, cleanup_error);
    std::filesystem::remove(output_path, cleanup_error);
    std::filesystem::remove_all(config.data.artifact_dir, cleanup_error);
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
    CHECK(kRelease.error.code == core::ErrorCode::kOk);

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
    CHECK(result.error.code != core::ErrorCode::kOk);
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
    std::optional<TelemetryDelivery> received_delivery;
    auto telemetry_stream = client.StartTelemetry(
        {.drone_id = "drone-1", .rate_hertz = 5}, [&](const TelemetryDelivery& delivery) {
            const auto& frame = delivery.frame;
            if (frame.drone_id == "drone-1") {
                std::lock_guard<std::mutex> lock(received_mutex);
                received_delivery = delivery;
                frame_count.fetch_add(1, std::memory_order_relaxed);
            }
        });
    REQUIRE(telemetry_stream.has_value());

    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    core::TelemetryFrame frame;
    frame.drone_id = "drone-1";
    frame.lat_deg = 40.0;
    frame.lon_deg = 44.0;
    frame.rel_alt_m = 10.0F;
    frame.battery_percent = 80.0F;
    frame.mode = "guided";
    frame.validity.position = true;
    frame.validity.relative_altitude = true;
    frame.validity.battery = true;
    frame.validity.mode = true;
    frame.provenance.position.updated = true;
    frame.provenance.position.source_time = {
        .timestamp_ms = 100,
        .clock_domain = core::ClockDomain::kUnixEpoch,
    };
    frame.position_frame = core::CoordinateFrame::kWgs84;
    frame.velocity_frame = core::CoordinateFrame::kLocalNed;
    frame.gps_quality = core::GpsQuality::kFix3D;
    frame.validity.gps = true;
    frame.accuracy.horizontal_position = core::UncertaintyEstimate{
        .value = 1.5F,
        .descriptor = {.semantics = core::UncertaintySemantics::kBackendSpecific},
    };
    frame.estimator_state = core::EstimatorState::kHealthy;
    frame.validity.estimator = true;

    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", frame);
            return frame_count.load(std::memory_order_relaxed) >= 1;
        },
        kWaitTimeout, std::chrono::milliseconds{50}));
    {
        std::lock_guard<std::mutex> lock(received_mutex);
        REQUIRE(received_delivery.has_value());
        const auto& delivery = *received_delivery;
        const auto& frame = delivery.frame;
        CHECK(frame.HasPosition());
        REQUIRE(frame.provenance.position.source_time.timestamp_ms.has_value());
        CHECK(*frame.provenance.position.source_time.timestamp_ms == 100);
        CHECK(frame.position_frame == core::CoordinateFrame::kWgs84);
        CHECK(frame.gps_quality == core::GpsQuality::kFix3D);
        REQUIRE(frame.accuracy.horizontal_position.has_value());
        CHECK(frame.accuracy.horizontal_position->value == 1.5F);
        CHECK(frame.estimator_state == core::EstimatorState::kHealthy);
        CHECK_FALSE(delivery.transport_stream_id.empty());
        CHECK(delivery.sdk_receive_unix_time_ms > 0);
    }

    telemetry_stream->Stop();
    REQUIRE(testsupport::WaitUntil([&] { return !harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));
}

TEST_CASE("Agent session identity and receive time use injectable providers",
          "[agent][identity][integration]") {
    auto next_id = std::make_shared<std::atomic<std::uint64_t>>(1);
    auto providers = [next_id] {
        return swarmkit::agent::internal::RuntimeProviders{
            .wall_time_ms = [] { return 1'700'000'123'456LL; },
            .monotonic_time_ns = [] { return 77'000'000LL; },
            .new_id =
                [next_id](std::string_view prefix) {
                    return std::string(prefix) + "-" +
                           std::to_string(next_id->fetch_add(1, std::memory_order_relaxed));
                },
        };
    };

    testsupport::AgentServerHarness first(providers());
    Client first_client = MakeClient(first.Address());
    const PingResult first_ping = first_client.Ping();
    const HealthStatus first_health = first_client.GetHealth();
    const CapabilitiesResult first_capabilities = first_client.GetCapabilities();

    REQUIRE(first_ping.ok);
    CHECK(first_ping.unix_time_ms == 1'700'000'123'456LL);
    CHECK(first_ping.agent_session_id == "agent-session-1");
    CHECK(first_health.agent_session_id == first_ping.agent_session_id);
    CHECK(first_capabilities.agent_session_id == first_ping.agent_session_id);

    testsupport::AgentServerHarness second(providers());
    Client second_client = MakeClient(second.Address());
    const PingResult second_ping = second_client.Ping();
    REQUIRE(second_ping.ok);
    CHECK(second_ping.agent_session_id == "agent-session-2");
    CHECK(second_ping.agent_session_id != first_ping.agent_session_id);
}

TEST_CASE("Normalized telemetry producer identity survives subscribers and reconnect",
          "[client][integration][telemetry][identity]") {
    testsupport::AgentServerHarness harness;
    Client first = MakeClient(harness.Address());
    Client second = MakeClient(harness.Address());

    std::mutex frames_mutex;
    std::vector<TelemetryDelivery> first_frames;
    std::vector<TelemetryDelivery> second_frames;
    auto first_stream = first.StartTelemetry({.drone_id = "drone-1", .rate_hertz = 5},
                                             [&](const TelemetryDelivery& delivery) {
                                                 std::lock_guard<std::mutex> lock(frames_mutex);
                                                 first_frames.push_back(delivery);
                                             });
    auto second_stream = second.StartTelemetry({.drone_id = "drone-1", .rate_hertz = 5},
                                               [&](const TelemetryDelivery& delivery) {
                                                   std::lock_guard<std::mutex> lock(frames_mutex);
                                                   second_frames.push_back(delivery);
                                               });
    REQUIRE(first_stream.has_value());
    REQUIRE(second_stream.has_value());
    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().TelemetryRate("drone-1") == 5; },
                                   kWaitTimeout));

    core::TelemetryFrame backend_frame;
    backend_frame.validity.position = true;
    backend_frame.lat_deg = 40.0;
    backend_frame.lon_deg = 44.0;
    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", backend_frame);
            std::lock_guard<std::mutex> lock(frames_mutex);
            return !first_frames.empty() && !second_frames.empty();
        },
        kWaitTimeout));

    TelemetryDelivery first_seen;
    TelemetryDelivery second_seen;
    {
        std::lock_guard<std::mutex> lock(frames_mutex);
        first_seen = first_frames.back();
        second_seen = second_frames.back();
    }
    CHECK_FALSE(first_seen.frame.agent_session_id.empty());
    CHECK(first_seen.frame.agent_session_id == second_seen.frame.agent_session_id);
    CHECK(first_seen.frame.telemetry_sequence == second_seen.frame.telemetry_sequence);
    CHECK(first_seen.frame.telemetry_sequence == 1);
    CHECK_FALSE(first_seen.transport_stream_id.empty());
    CHECK_FALSE(second_seen.transport_stream_id.empty());
    CHECK(first_seen.transport_stream_id != second_seen.transport_stream_id);

    first_stream->Stop();
    second_stream->Stop();
    REQUIRE(testsupport::WaitUntil([&] { return !harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    std::optional<TelemetryDelivery> reconnected_frame;
    auto reconnected = first.StartTelemetry({.drone_id = "drone-1", .rate_hertz = 5},
                                            [&](const TelemetryDelivery& delivery) {
                                                std::lock_guard<std::mutex> lock(frames_mutex);
                                                reconnected_frame = delivery;
                                            });
    REQUIRE(reconnected.has_value());
    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));
    harness.Backend().EmitTelemetry("drone-1", backend_frame);
    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(frames_mutex);
            return reconnected_frame.has_value();
        },
        kWaitTimeout));
    {
        std::lock_guard<std::mutex> lock(frames_mutex);
        REQUIRE(reconnected_frame.has_value());
        CHECK(reconnected_frame->frame.agent_session_id == first_seen.frame.agent_session_id);
        CHECK(reconnected_frame->frame.telemetry_sequence > first_seen.frame.telemetry_sequence);
        CHECK(reconnected_frame->transport_stream_id != first_seen.transport_stream_id);
    }
    reconnected->Stop();
}

TEST_CASE("Telemetry preserves per-measurement source and receive-time freshness",
          "[client][integration][telemetry][provenance]") {
    auto wall_time = std::make_shared<std::atomic<std::int64_t>>(10'000);
    auto monotonic_time = std::make_shared<std::atomic<std::int64_t>>(1'000'000);
    swarmkit::agent::internal::RuntimeProviders providers{
        .wall_time_ms = [wall_time] { return wall_time->fetch_add(10, std::memory_order_relaxed); },
        .monotonic_time_ns =
            [monotonic_time] { return monotonic_time->fetch_add(100, std::memory_order_relaxed); },
        .new_id = [](std::string_view prefix) { return std::string(prefix) + "-fixed"; },
    };
    testsupport::AgentServerHarness harness(std::move(providers));
    Client client = MakeClient(harness.Address());

    std::mutex frames_mutex;
    std::vector<TelemetryDelivery> frames;
    auto stream = client.StartTelemetry({.drone_id = "drone-1", .rate_hertz = 100},
                                        [&](const TelemetryDelivery& delivery) {
                                            std::lock_guard<std::mutex> lock(frames_mutex);
                                            frames.push_back(delivery);
                                        });
    REQUIRE(stream.has_value());
    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    core::TelemetryFrame position;
    position.validity.position = true;
    position.lat_deg = 40.0;
    position.lon_deg = 44.0;
    position.provenance.position.updated = true;
    position.provenance.position.source = "script.position";
    position.provenance.position.source_time = {
        .timestamp_ms = 1234,
        .clock_domain = core::ClockDomain::kVehicleBoot,
        .synchronization = core::ClockSynchronization::kUnsynchronized,
    };
    position.accuracy.horizontal_position = core::UncertaintyEstimate{
        .value = 0.7F,
        .descriptor =
            {
                .semantics = core::UncertaintySemantics::kConfidenceBound,
                .confidence_level = 0.95,
                .calibration_profile_id = "lab-rtk",
                .calibration_version = "2026-08-11",
                .source = "script.horizontal_accuracy",
            },
    };
    position.accuracy.vertical_position = core::UncertaintyEstimate{
        .value = 1.1F,
        .descriptor =
            {
                .semantics = core::UncertaintySemantics::kDeterministicHardBound,
                .source = "script.test_only_vertical_bound",
            },
    };
    position.accuracy.speed = core::UncertaintyEstimate{
        .value = 0.4F,
        .descriptor =
            {
                .semantics = core::UncertaintySemantics::kBackendSpecific,
                .source = "script.speed_accuracy",
            },
    };
    position.accuracy.horizontal_velocity = core::UncertaintyEstimate{
        .value = 0.2F,
        .descriptor =
            {
                .semantics = core::UncertaintySemantics::kEmpiricallyCalibratedBound,
                .confidence_level = 0.99,
                .calibration_profile_id = "lab-velocity",
                .calibration_version = "v2",
                .source = "script.horizontal_velocity_accuracy",
            },
    };
    position.accuracy.vertical_velocity = core::UncertaintyEstimate{
        .value = 0.3F,
        .descriptor =
            {
                .semantics = core::UncertaintySemantics::kStandardDeviation,
                .source = "script.vertical_velocity_accuracy",
            },
    };
    position.provenance.accuracy.updated = true;
    position.provenance.accuracy.source = "script.accuracy";
    position.provenance.accuracy.source_time = position.provenance.position.source_time;
    harness.Backend().EmitTelemetry("drone-1", position);
    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(frames_mutex);
            return !frames.empty();
        },
        kWaitTimeout));

    TelemetryDelivery first_delivery;
    {
        std::lock_guard<std::mutex> lock(frames_mutex);
        first_delivery = frames.back();
    }
    const auto& first = first_delivery.frame;
    CHECK(first.agent_receive_unix_time_ms > 0);
    CHECK(first.agent_receive_monotonic_time_ns > 0);
    CHECK(first_delivery.sdk_receive_unix_time_ms > 0);
    CHECK(first.provenance.position.updated);
    CHECK(first.provenance.position.generation == 1);
    CHECK(first.provenance.position.source == "script.position");
    REQUIRE(first.provenance.position.source_time.timestamp_ms.has_value());
    CHECK(*first.provenance.position.source_time.timestamp_ms == 1234);
    CHECK(first.provenance.position.source_time.clock_domain == core::ClockDomain::kVehicleBoot);
    CHECK_FALSE(first.provenance.position.source_time.clock_uncertainty_ms.has_value());
    CHECK(first.provenance.position.agent_receive_unix_time_ms == first.agent_receive_unix_time_ms);
    REQUIRE(first.accuracy.horizontal_position.has_value());
    CHECK(first.accuracy.horizontal_position->descriptor.semantics ==
          core::UncertaintySemantics::kConfidenceBound);
    REQUIRE(first.accuracy.horizontal_position->descriptor.confidence_level.has_value());
    CHECK(*first.accuracy.horizontal_position->descriptor.confidence_level == 0.95);
    CHECK(first.accuracy.horizontal_position->descriptor.calibration_profile_id == "lab-rtk");
    CHECK(first.accuracy.horizontal_position->descriptor.calibration_version == "2026-08-11");
    CHECK(first.accuracy.horizontal_position->descriptor.measurement_generation ==
          first.provenance.accuracy.generation);
    REQUIRE(first.accuracy.horizontal_velocity.has_value());
    CHECK(first.accuracy.horizontal_velocity->descriptor.semantics ==
          core::UncertaintySemantics::kEmpiricallyCalibratedBound);
    REQUIRE(first.accuracy.horizontal_velocity->descriptor.confidence_level.has_value());
    CHECK(*first.accuracy.horizontal_velocity->descriptor.confidence_level == 0.99);
    REQUIRE(first.accuracy.vertical_velocity.has_value());
    CHECK(first.accuracy.vertical_velocity->descriptor.semantics ==
          core::UncertaintySemantics::kStandardDeviation);
    REQUIRE(first.accuracy.vertical_position.has_value());
    CHECK(first.accuracy.vertical_position->descriptor.semantics ==
          core::UncertaintySemantics::kDeterministicHardBound);
    REQUIRE(first.accuracy.speed.has_value());
    CHECK(first.accuracy.speed->descriptor.semantics ==
          core::UncertaintySemantics::kBackendSpecific);
    CHECK(core::UncertaintyDescriptor{}.semantics == core::UncertaintySemantics::kUnknown);

    core::TelemetryFrame heartbeat = position;
    heartbeat.failsafe = false;
    heartbeat.validity.failsafe = true;
    heartbeat.provenance.position.updated = false;
    heartbeat.provenance.vehicle_state.updated = true;
    heartbeat.provenance.vehicle_state.source = "script.heartbeat";
    const std::uint64_t first_sequence = first.telemetry_sequence;
    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", heartbeat);
            std::lock_guard<std::mutex> lock(frames_mutex);
            return !frames.empty() && frames.back().frame.telemetry_sequence > first_sequence;
        },
        kWaitTimeout, std::chrono::milliseconds{20}));

    core::TelemetryFrame repeated;
    {
        std::lock_guard<std::mutex> lock(frames_mutex);
        repeated = frames.back().frame;
    }
    CHECK_FALSE(repeated.provenance.position.updated);
    CHECK(repeated.provenance.position.generation == first.provenance.position.generation);
    CHECK(repeated.provenance.position.agent_receive_unix_time_ms ==
          first.provenance.position.agent_receive_unix_time_ms);
    CHECK(repeated.provenance.position.source_time == first.provenance.position.source_time);
    CHECK(repeated.provenance.vehicle_state.updated);
    CHECK(repeated.provenance.vehicle_state.generation >= 1);
    CHECK(repeated.provenance.vehicle_state.agent_receive_unix_time_ms !=
          first.provenance.position.agent_receive_unix_time_ms);
    stream->Stop();
}

TEST_CASE("Client telemetry subscription handle contains callback exceptions",
          "[client][integration][telemetry]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    auto invalid_subscription = client.StartTelemetry({.drone_id = "drone-1", .rate_hertz = 0},
                                                      [](const TelemetryDelivery&) {});
    REQUIRE_FALSE(invalid_subscription.has_value());
    CHECK(invalid_subscription.error().code == core::ErrorCode::kInvalidArgument);

    std::atomic<bool> callback_error_seen{false};
    std::atomic<bool> connected_seen{false};

    SubscriptionOptions options;
    options.backpressure.max_pending_callbacks = 4;
    options.backpressure.policy = StreamBackpressurePolicy::kDropOldest;

    auto subscription = client.StartTelemetry(
        {.drone_id = "drone-1", .rate_hertz = 5},
        [](const TelemetryDelivery&) { throw std::runtime_error("boom"); },
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

    const GoalResult result = client.SetActiveGoal({.goal = goal});
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

    const ActiveGoalResult status = client.GetActiveGoal("drone-1");
    REQUIRE(status.ok);
    REQUIRE(status.active_goal.has_value());
    CHECK(status.active_goal->status == GoalStatus::kReached);
    CHECK(status.active_goal->goal.labels.at("from_node") == "node-a");
    CHECK(status.active_goal->goal.labels.at("to_node") == "node-b");
    CHECK(status.active_goal->goal.labels.at("edge_id") == "edge-42");

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

    const GoalResult set_result = client.SetActiveGoal({.goal = goal});
    REQUIRE(set_result.ok);
    REQUIRE(testsupport::WaitUntil([&] { return harness.Backend().HasTelemetryStream("drone-1"); },
                                   kWaitTimeout));

    REQUIRE(set_result.execution_handle.has_value());
    const CancelGoalResult cancel_result = client.CancelGoal(*set_result.execution_handle);
    REQUIRE(cancel_result.ok);
    CHECK(cancel_result.cancelled_execution == set_result.execution_handle);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [](const AgentReport& report) {
                return report.goal.has_value() && report.goal->status == GoalStatus::kCancelled &&
                       report.goal->goal_id == "goal-cancel";
            });
        },
        kWaitTimeout));

    const ActiveGoalResult status = client.GetActiveGoal("drone-1");
    REQUIRE(status.ok);
    CHECK_FALSE(status.active_goal.has_value());

    reports_stream->Stop();
}

TEST_CASE("Goal retries receive distinct physical attempts and guarded cancellation",
          "[client][integration][goal][identity]") {
    auto next_id = std::make_shared<std::atomic<std::uint64_t>>(1);
    swarmkit::agent::internal::RuntimeProviders providers{
        .wall_time_ms = [] { return 1'700'000'000'000LL; },
        .monotonic_time_ns = [] { return 42'000'000LL; },
        .new_id =
            [next_id](std::string_view prefix) {
                return std::string(prefix) + "-" +
                       std::to_string(next_id->fetch_add(1, std::memory_order_relaxed));
            },
    };
    testsupport::AgentServerHarness harness(std::move(providers));
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto report_stream =
        client.StartReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
            std::lock_guard<std::mutex> lock(reports_mutex);
            reports.push_back(report);
        });
    REQUIRE(report_stream.has_value());

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-retry";
    goal.revision = 4;
    goal.target = {.lat_deg = 41.0, .lon_deg = 45.0, .alt_m = 20.0};
    goal.acceptance_radius_m = 2.0F;
    goal.deviation_radius_m = 10.0F;
    goal.timeout_ms = 5000;
    const core::ExecutionContext execution_context{
        .mission_id = "mission-9",
        .mission_revision = 2,
        .model_hash = "sha256:model",
        .operation_id = "operation-17",
        .operation_attempt_revision = 6,
    };

    const ActiveGoalRequest request{.goal = goal, .execution_context = execution_context};
    const GoalResult first = client.SetActiveGoal(request);
    REQUIRE(first.ok);
    REQUIRE(first.execution_handle.has_value());
    CHECK(first.execution_handle->agent_session_id == "agent-session-1");
    CHECK(first.execution_handle->physical_attempt_id == "physical-attempt-2");
    CHECK(first.execution_handle->physical_attempt_revision == 1);
    CHECK(first.execution_handle->goal_id == goal.goal_id);
    CHECK(first.execution_handle->goal_revision == goal.revision);
    REQUIRE(first.execution_handle->context.has_value());
    CHECK(first.execution_handle->context->mission_id == "mission-9");
    CHECK(first.execution_handle->client_id == "test-client");
    CHECK(first.execution_handle->correlation_id == first.correlation_id);

    const GoalResult retry = client.SetActiveGoal(request);
    REQUIRE(retry.ok);
    REQUIRE(retry.execution_handle.has_value());
    CHECK(retry.execution_handle->physical_attempt_id == "physical-attempt-3");
    CHECK(retry.execution_handle->physical_attempt_revision == 2);
    CHECK(*retry.execution_handle != *first.execution_handle);

    const ActiveGoalResult active = client.GetActiveGoal("drone-1");
    REQUIRE(active.ok);
    REQUIRE(active.active_goal.has_value());
    CHECK(active.active_goal->execution_handle == *retry.execution_handle);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [&](const AgentReport& report) {
                const auto* handle = std::get_if<core::ExecutionHandle>(&report.execution_binding);
                return report.goal.has_value() && report.goal->status == GoalStatus::kSuperseded &&
                       handle != nullptr && *handle == *first.execution_handle;
            });
        },
        kWaitTimeout));

    std::mutex telemetry_mutex;
    std::optional<core::TelemetryFrame> bound_frame;
    auto telemetry_stream = client.StartTelemetry(
        {.drone_id = "drone-1", .rate_hertz = 5}, [&](const TelemetryDelivery& delivery) {
            std::lock_guard<std::mutex> lock(telemetry_mutex);
            bound_frame = delivery.frame;
        });
    REQUIRE(telemetry_stream.has_value());
    core::TelemetryFrame backend_frame;
    backend_frame.validity.position = true;
    backend_frame.validity.relative_altitude = true;
    backend_frame.lat_deg = 40.0;
    backend_frame.lon_deg = 44.0;
    backend_frame.rel_alt_m = 0.0F;
    REQUIRE(testsupport::WaitUntil(
        [&] {
            harness.Backend().EmitTelemetry("drone-1", backend_frame);
            std::lock_guard<std::mutex> lock(telemetry_mutex);
            return bound_frame.has_value();
        },
        kWaitTimeout, std::chrono::milliseconds{50}));
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        REQUIRE(bound_frame.has_value());
        REQUIRE(bound_frame->execution_handle.has_value());
        CHECK(*bound_frame->execution_handle == *retry.execution_handle);
        CHECK(bound_frame->agent_session_id == retry.execution_handle->agent_session_id);
    }
    telemetry_stream->Stop();

    const CancelGoalResult stale_cancel = client.CancelGoal(*first.execution_handle);
    CHECK_FALSE(stale_cancel.ok);
    const ActiveGoalResult still_active = client.GetActiveGoal("drone-1");
    REQUIRE(still_active.active_goal.has_value());
    CHECK(still_active.active_goal->execution_handle == *retry.execution_handle);

    const CancelGoalResult current_cancel = client.CancelGoal(*retry.execution_handle);
    REQUIRE(current_cancel.ok);
    CHECK_FALSE(client.GetActiveGoal("drone-1").active_goal.has_value());
    report_stream->Stop();
}

TEST_CASE("Backend-dispatched goal failure retains its physical attempt identity",
          "[client][integration][goal][identity]") {
    testsupport::AgentServerHarness harness;
    harness.Backend().SetExecuteHandler([](const commands::CommandEnvelope&) {
        return core::Result::Failed("dispatch failed after acceptance");
    });
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto report_stream =
        client.StartReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
            std::lock_guard<std::mutex> lock(reports_mutex);
            reports.push_back(report);
        });
    REQUIRE(report_stream.has_value());

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-failed-attempt";
    goal.revision = 1;
    goal.target = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    goal.acceptance_radius_m = 2.0F;
    goal.deviation_radius_m = 8.0F;

    const GoalResult result = client.SetActiveGoal({.goal = goal});
    CHECK_FALSE(result.ok);
    REQUIRE(result.execution_handle.has_value());
    CHECK(result.execution_handle->IsComplete());
    CHECK(result.execution_handle->goal_id == goal.goal_id);
    CHECK_FALSE(client.GetActiveGoal("drone-1").active_goal.has_value());
    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [&](const AgentReport& report) {
                const auto* handle = std::get_if<core::ExecutionHandle>(&report.execution_binding);
                return report.goal.has_value() && report.goal->status == GoalStatus::kFailed &&
                       handle != nullptr && *handle == *result.execution_handle;
            });
        },
        kWaitTimeout));
    report_stream->Stop();
}

TEST_CASE("A new goal revision supersedes the exact previous attempt",
          "[client][integration][goal][identity]") {
    testsupport::AgentServerHarness harness;
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto report_stream =
        client.StartReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
            std::lock_guard<std::mutex> lock(reports_mutex);
            reports.push_back(report);
        });
    REQUIRE(report_stream.has_value());

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-revision";
    goal.revision = 1;
    goal.target = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    goal.acceptance_radius_m = 2.0F;
    goal.deviation_radius_m = 8.0F;
    const GoalResult first = client.SetActiveGoal({.goal = goal});
    REQUIRE(first.ok);
    REQUIRE(first.execution_handle.has_value());

    goal.revision = 2;
    const GoalResult second = client.SetActiveGoal({.goal = goal});
    REQUIRE(second.ok);
    REQUIRE(second.execution_handle.has_value());
    CHECK(second.execution_handle->goal_id == first.execution_handle->goal_id);
    CHECK(second.execution_handle->goal_revision == 2);
    CHECK(first.execution_handle->goal_revision == 1);
    CHECK(second.execution_handle->physical_attempt_revision >
          first.execution_handle->physical_attempt_revision);
    CHECK(second.execution_handle->physical_attempt_id !=
          first.execution_handle->physical_attempt_id);

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [&](const AgentReport& report) {
                const auto* handle = std::get_if<core::ExecutionHandle>(&report.execution_binding);
                return report.goal.has_value() && report.goal->status == GoalStatus::kSuperseded &&
                       handle != nullptr && *handle == *first.execution_handle;
            });
        },
        kWaitTimeout));
    REQUIRE(client.CancelGoal(*second.execution_handle).ok);
    report_stream->Stop();
}

TEST_CASE("Timed-out goal reports retain the exact physical attempt handle",
          "[client][integration][goal][identity][timeout]") {
    auto wall_time = std::make_shared<std::atomic<std::int64_t>>(1'000);
    auto next_id = std::make_shared<std::atomic<std::uint64_t>>(1);
    swarmkit::agent::internal::RuntimeProviders providers{
        .wall_time_ms =
            [wall_time] { return wall_time->fetch_add(1'000, std::memory_order_relaxed); },
        .monotonic_time_ns = [] { return 1'000'000LL; },
        .new_id =
            [next_id](std::string_view prefix) {
                return std::string(prefix) + "-" +
                       std::to_string(next_id->fetch_add(1, std::memory_order_relaxed));
            },
    };
    testsupport::AgentServerHarness harness(std::move(providers));
    Client client = MakeClient(harness.Address());

    std::mutex reports_mutex;
    std::vector<AgentReport> reports;
    auto report_stream =
        client.StartReports({.drone_id = "drone-1"}, [&](const AgentReport& report) {
            std::lock_guard<std::mutex> lock(reports_mutex);
            reports.push_back(report);
        });
    REQUIRE(report_stream.has_value());

    ActiveGoal goal;
    goal.drone_id = "drone-1";
    goal.goal_id = "goal-timeout";
    goal.revision = 3;
    goal.target = {.lat_deg = 40.0, .lon_deg = 44.0, .alt_m = 10.0};
    goal.acceptance_radius_m = 1.0F;
    goal.deviation_radius_m = 5.0F;
    goal.timeout_ms = 10;
    const GoalResult result = client.SetActiveGoal({.goal = goal});
    REQUIRE(result.ok);
    REQUIRE(result.execution_handle.has_value());

    REQUIRE(testsupport::WaitUntil(
        [&] {
            std::lock_guard<std::mutex> lock(reports_mutex);
            return std::ranges::any_of(reports, [&](const AgentReport& report) {
                const auto* handle = std::get_if<core::ExecutionHandle>(&report.execution_binding);
                return report.goal.has_value() && report.goal->status == GoalStatus::kTimeout &&
                       handle != nullptr && *handle == *result.execution_handle;
            });
        },
        kWaitTimeout));
    const ActiveGoalResult status = client.GetActiveGoal("drone-1");
    REQUIRE(status.ok);
    REQUIRE(status.active_goal.has_value());
    CHECK(status.active_goal->status == GoalStatus::kTimeout);
    CHECK(status.active_goal->execution_handle == *result.execution_handle);
    report_stream->Stop();
}

TEST_CASE("Client reports backend command failure and telemetry counters",
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
    CHECK(kCommand.error.code == core::ErrorCode::kBackendFailure);
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
