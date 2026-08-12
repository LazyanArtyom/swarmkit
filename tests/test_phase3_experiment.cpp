// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "../src/agent/execution_recorder.h"
#include "swarmkit/client/telemetry_codec.h"
#include "swarmkit/evidence/execution_log.h"
#include "swarmkit/experiment/fault_injection.h"
#include "swarmkit/experiment/manual_runtime.h"
#include "swarmkit/experiment/scripted_backend.h"
#include "swarmkit/experiment/telemetry_replay.h"

namespace swarmkit::experiment {
namespace {

namespace fs = std::filesystem;

[[nodiscard]] fs::path UniqueEvidencePath(std::string_view name) {
    return fs::temp_directory_path() /
           (std::string(name) + "-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".swkev");
}

[[nodiscard]] core::TelemetryFrame InputFrame(std::string drone_id, std::int64_t source_time_ms) {
    core::TelemetryFrame frame;
    frame.drone_id = std::move(drone_id);
    frame.lat_deg = 40.0;
    frame.lon_deg = 44.0;
    frame.rel_alt_m = 10.0F;
    frame.abs_alt_m = 1010.0F;
    frame.validity.position = true;
    frame.validity.relative_altitude = true;
    frame.validity.absolute_altitude = true;
    frame.validity.velocity = true;
    frame.validity.estimator = true;
    frame.estimator_state = core::EstimatorState::kHealthy;
    frame.estimator_position_ok = true;
    frame.estimator_velocity_ok = true;
    frame.accuracy.horizontal_position = core::UncertaintyEstimate{
        .value = 0.5F,
        .descriptor = {.semantics = core::UncertaintySemantics::kBackendSpecific,
                       .source = "scripted.raw"},
    };
    const core::TimestampEvidence source_time{
        .timestamp_ms = source_time_ms,
        .clock_domain = core::ClockDomain::kUnixEpoch,
        .synchronization = core::ClockSynchronization::kSynchronized,
    };
    frame.provenance.position = {
        .updated = true, .source_time = source_time, .source = "scripted.position"};
    frame.provenance.velocity = {
        .updated = true, .source_time = source_time, .source = "scripted.velocity"};
    frame.provenance.accuracy = {
        .updated = true, .source_time = source_time, .source = "scripted.accuracy"};
    frame.provenance.estimator = {
        .updated = true, .source_time = source_time, .source = "scripted.estimator"};
    return frame;
}

[[nodiscard]] commands::CommandEnvelope TestCommand(std::string correlation_id = "command-1") {
    return {
        .context = {.drone_id = "drone-1",
                    .client_id = "experiment",
                    .correlation_id = std::move(correlation_id)},
        .command = commands::FlightCmd{commands::CmdArm{}},
    };
}

TEST_CASE("ManualRuntime advances clocks and identifiers without wall-clock sleeps",
          "[phase3][runtime]") {
    ManualRuntime runtime(1000, 2000);
    CHECK(runtime.WallTimeMs() == 1000);
    CHECK(runtime.MonotonicTimeNs() == 2000);
    CHECK(runtime.NewId("session") == "session-1");
    CHECK(runtime.NewId("session") == "session-2");
    runtime.AdvanceMilliseconds(25);
    CHECK(runtime.WallTimeMs() == 1025);
    CHECK(runtime.MonotonicTimeNs() == 25'002'000);
}

TEST_CASE("ScriptedBackend consumes exact command outcomes and emits telemetry synchronously",
          "[phase3][scripted]") {
    auto scripted = MakeScriptedBackend();
    core::BackendCommandOutcome accepted{
        .result = core::Result::Success("ACKed"),
        .dispatch_state = core::BackendDispatchState::kAccepted,
        .protocol_responses = {{.protocol = "test-protocol",
                                .command_name = "ARM",
                                .response_expected = true,
                                .response_received = true,
                                .result_code = 0,
                                .result_name = "ACCEPTED"}},
    };
    scripted.control->QueueCommandStep(
        {.expected_correlation_id = "command-1", .outcome = accepted});

    const core::BackendCommandOutcome result = scripted.backend->Execute(TestCommand());
    REQUIRE(result.IsOk());
    REQUIRE(result.protocol_responses.size() == 1);
    CHECK(result.protocol_responses.front().response_received);
    CHECK(scripted.control->PendingCommandSteps() == 0);
    REQUIRE(scripted.control->Commands().size() == 1);

    std::vector<core::TelemetryFrame> frames;
    REQUIRE(scripted.backend
                ->StartTelemetry(
                    "drone-1", 50,
                    [&frames](const core::TelemetryFrame& frame) { frames.push_back(frame); })
                .IsOk());
    core::TelemetryFrame exact = InputFrame("backend-id-is-normalized-later", 1234);
    exact.telemetry_sequence = 77;
    REQUIRE(scripted.control->EmitTelemetry("drone-1", exact).IsOk());
    REQUIRE(frames.size() == 1);
    CHECK(frames.front() == exact);
    CHECK(scripted.backend->StopTelemetry("drone-1").IsOk());
    CHECK_FALSE(scripted.control->EmitTelemetry("drone-1", exact).IsOk());
}

struct FaultRun {
    std::vector<core::TelemetryFrame> frames;
    std::vector<FaultDecision> decisions;
    std::vector<agent::BackendEvidenceEvent> evidence;
};

[[nodiscard]] FaultRun RunFaultScenario(std::uint64_t seed) {
    auto scripted = MakeScriptedBackend();
    auto fault = MakeFaultInjectingBackend(std::move(scripted.backend),
                                           {.seed = seed,
                                            .telemetry_loss_probability = 0.2,
                                            .telemetry_duplication_probability = 0.3,
                                            .telemetry_reorder_probability = 0.25,
                                            .position_spike_probability = 0.1,
                                            .estimator_degradation_probability = 0.15,
                                            .invalid_accuracy_probability = 0.2,
                                            .telemetry_delay_frames = 2,
                                            .horizontal_bias_north_m = 1.0,
                                            .uniform_horizontal_noise_m = 0.5,
                                            .source_clock_offset_ms = 40,
                                            .source_clock_uncertainty_ms = 5.0});
    REQUIRE(fault.has_value());
    FaultRun run;
    fault->backend->SetEvidenceCallback(
        [&run](const agent::BackendEvidenceEvent& event) { run.evidence.push_back(event); });
    REQUIRE(fault->backend
                ->StartTelemetry(
                    "drone-1", 10,
                    [&run](const core::TelemetryFrame& frame) { run.frames.push_back(frame); })
                .IsOk());
    for (int index = 0; index < 24; ++index) {
        REQUIRE(scripted.control
                    ->EmitTelemetry("drone-1",
                                    InputFrame("drone-1", 1000 + static_cast<std::int64_t>(index)))
                    .IsOk());
    }
    fault->control->Flush();
    run.decisions = fault->control->Decisions();
    REQUIRE(fault->backend->StopTelemetry("drone-1").IsOk());
    return run;
}

TEST_CASE("Fault injection is deterministic for one seed and records realized decisions",
          "[phase3][faults]") {
    const FaultRun first = RunFaultScenario(0x12345678ULL);
    const FaultRun second = RunFaultScenario(0x12345678ULL);
    const FaultRun changed = RunFaultScenario(0x87654321ULL);

    CHECK(first.frames == second.frames);
    CHECK(first.decisions == second.decisions);
    CHECK(first.decisions != changed.decisions);
    REQUIRE_FALSE(first.evidence.empty());
    CHECK(first.evidence.front().source == "deterministic-fault-injector");
    CHECK(first.evidence.front().kind == "configuration");
    REQUIRE(first.evidence.front().random_seed.has_value());
    CHECK(*first.evidence.front().random_seed == 0x12345678ULL);
    CHECK(std::ranges::all_of(first.frames, [](const core::TelemetryFrame& frame) {
        return frame.provenance.position.source_time.timestamp_ms.has_value() &&
               frame.provenance.position.source_time.clock_uncertainty_ms == 5.0;
    }));
}

TEST_CASE("Fault injector can ACK without physical dispatch", "[phase3][faults][commands]") {
    auto scripted = MakeScriptedBackend();
    auto scripted_control = scripted.control;
    auto fault = MakeFaultInjectingBackend(std::move(scripted.backend),
                                           {.seed = 9, .ack_without_motion_probability = 1.0});
    REQUIRE(fault.has_value());
    const core::BackendCommandOutcome outcome = fault->backend->Execute(TestCommand());
    REQUIRE(outcome.IsOk());
    REQUIRE(outcome.protocol_responses.size() == 1);
    CHECK(outcome.protocol_responses.front().response_received);
    CHECK(scripted_control->Commands().empty());
}

[[nodiscard]] swarmkit::v1::TelemetryFrame RecordedFrame(std::uint64_t sequence) {
    swarmkit::v1::TelemetryFrame frame;
    frame.set_drone_id("drone-1");
    frame.set_agent_session_id("agent-session-log");
    frame.set_telemetry_sequence(sequence);
    frame.set_agent_receive_unix_time_ms(5000 + static_cast<std::int64_t>(sequence));
    frame.set_agent_receive_monotonic_time_ns(6000 + static_cast<std::int64_t>(sequence));
    frame.set_lat_deg(40.0 + static_cast<double>(sequence) / 1000.0);
    frame.mutable_validity()->set_position(true);
    frame.mutable_provenance()->mutable_position()->set_updated(true);
    frame.mutable_provenance()->mutable_position()->mutable_source_time()->set_timestamp_ms(
        4000 + static_cast<std::int64_t>(sequence));
    return frame;
}

TEST_CASE("Execution evidence reader validates checksums and replays normalized ordering exactly",
          "[phase3][evidence][replay]") {
    const fs::path path = UniqueEvidencePath("swarmkit-phase3-reader");
    ManualRuntime runtime(10'000, 20'000);
    agent::internal::ExecutionRecorderOptions options;
    options.config.file_path = path.string();
    options.config.overwrite_existing = true;
    options.providers.wall_time_ms = [&runtime] { return runtime.WallTimeMs(); };
    options.providers.monotonic_time_ns = [&runtime] { return runtime.MonotonicTimeNs(); };
    options.providers.new_id = [&runtime](std::string_view prefix) {
        return runtime.NewId(prefix);
    };
    options.agent_session_id = "agent-session-log";
    options.metadata.set_run_id("phase3-run");
    options.metadata.set_scenario_id("ordering-scenario");
    options.metadata.set_random_seed(17);

    {
        agent::internal::ExecutionRecorder recorder(std::move(options));
        REQUIRE(recorder.Valid());
        for (const std::uint64_t sequence : {10ULL, 11ULL, 13ULL, 12ULL, 13ULL}) {
            recorder.RecordTelemetry(RecordedFrame(sequence));
            runtime.AdvanceMilliseconds(1);
        }
        recorder.RecordBackendEvidence({.source = "scenario",
                                        .kind = "supersession",
                                        .source_sequence = 1,
                                        .random_seed = 17,
                                        .attributes = {{"old_attempt", "attempt-1"}}});
        recorder.Close();
        REQUIRE(recorder.Valid());
    }

    auto log = evidence::ReadExecutionLog(path.string());
    REQUIRE(log.has_value());
    CHECK(log->has_session_start);
    CHECK(log->has_clean_completion);
    CHECK(log->agent_session_id == "agent-session-log");
    REQUIRE(log->events.front().event_sequence() == 1);
    for (std::size_t index = 1; index < log->events.size(); ++index) {
        CHECK(log->events[index].event_sequence() == log->events[index - 1].event_sequence() + 1);
    }

    std::vector<client::TelemetryFrameObservation> replayed;
    const core::Result replay_result = ReplayNormalizedTelemetry(
        *log, [&replayed](const client::TelemetryObservation& observation) {
            if (const auto* frame = std::get_if<client::TelemetryFrameObservation>(&observation)) {
                replayed.push_back(*frame);
            }
        });
    REQUIRE(replay_result.IsOk());
    REQUIRE(replayed.size() == 5);
    CHECK(replayed[0].sequence_relation == client::TelemetrySequenceRelation::kFirst);
    CHECK(replayed[1].sequence_relation == client::TelemetrySequenceRelation::kNext);
    CHECK(replayed[2].sequence_relation == client::TelemetrySequenceRelation::kGap);
    CHECK(replayed[2].missing_first_sequence == 12);
    CHECK(replayed[3].sequence_relation == client::TelemetrySequenceRelation::kReordered);
    CHECK(replayed[4].sequence_relation == client::TelemetrySequenceRelation::kDuplicate);
    CHECK(replayed[0].delivery.frame.agent_receive_unix_time_ms == 5010);
    CHECK_FALSE(replayed[0].delivery.sdk_receive_unix_time_ms.has_value());

    const auto original_size = fs::file_size(path);
    REQUIRE(original_size > 1);
    fs::resize_file(path, original_size - 1);
    const auto truncated = evidence::ReadExecutionLog(path.string());
    REQUIRE_FALSE(truncated.has_value());
    CHECK(truncated.error().code == evidence::ExecutionLogErrorCode::kTruncatedRecord);
    std::error_code remove_error;
    fs::remove(path, remove_error);
}

TEST_CASE("Execution evidence reader reconstructs bounded rotation segments oldest first",
          "[phase3][evidence][rotation]") {
    const fs::path path = UniqueEvidencePath("swarmkit-phase3-rotation");
    agent::internal::ExecutionRecorderOptions options;
    options.config.file_path = path.string();
    options.config.max_segment_bytes = 2000;
    options.config.max_segments = 3;
    options.config.loss_policy = agent::EvidenceLossPolicy::kRotateOldest;
    options.config.overwrite_existing = true;
    options.providers.wall_time_ms = [] { return 100; };
    options.providers.monotonic_time_ns = [] { return 200; };
    options.agent_session_id = "rotated-session";
    options.metadata.set_run_id("rotated-run");

    {
        agent::internal::ExecutionRecorder recorder(std::move(options));
        auto frame = RecordedFrame(1);
        frame.set_mode(std::string(1600, 'x'));
        recorder.RecordTelemetry(frame);
        recorder.Close();
        REQUIRE(recorder.Valid());
    }
    REQUIRE(fs::exists(path.string() + ".1"));
    auto log = evidence::ReadExecutionLog(path.string());
    REQUIRE(log.has_value());
    REQUIRE(log->events.size() >= 3);
    CHECK(log->events.front().has_session());
    CHECK(log->events.back().has_session());
    CHECK(log->events.back().session().state() == swarmkit::v1::EXECUTION_SESSION_COMPLETED);

    std::error_code error;
    fs::remove(path, error);
    fs::remove(path.string() + ".1", error);
    fs::remove(path.string() + ".2", error);
}

TEST_CASE("TelemetrySequenceTracker distinguishes session changes from sequence gaps",
          "[phase3][telemetry][sequence]") {
    client::TelemetrySequenceTracker tracker;
    const auto observe = [&tracker](std::string session, std::uint64_t sequence) {
        client::TelemetryDelivery delivery;
        delivery.frame.agent_session_id = std::move(session);
        delivery.frame.telemetry_sequence = sequence;
        return tracker.Observe(std::move(delivery));
    };
    CHECK(observe("session-a", 5).sequence_relation == client::TelemetrySequenceRelation::kFirst);
    const auto gap = observe("session-a", 8);
    CHECK(gap.sequence_relation == client::TelemetrySequenceRelation::kGap);
    CHECK(gap.missing_first_sequence == 6);
    CHECK(gap.missing_last_sequence == 7);
    CHECK(observe("session-a", 8).sequence_relation ==
          client::TelemetrySequenceRelation::kDuplicate);
    CHECK(observe("session-a", 7).sequence_relation ==
          client::TelemetrySequenceRelation::kReordered);
    CHECK(observe("session-b", 1).sequence_relation ==
          client::TelemetrySequenceRelation::kNewSession);
}

}  // namespace
}  // namespace swarmkit::experiment
