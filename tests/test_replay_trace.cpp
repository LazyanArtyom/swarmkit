// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <filesystem>

#include "swarmkit/core/replay_trace.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_acceptance_verifier.h"

using namespace swarmkit::core;
using Catch::Matchers::WithinAbs;

namespace {

EvidenceRecord MakeSampleRecord(const std::string& agent_id, std::uint64_t seq, std::int64_t src_ms, std::int64_t rx_ms) {
    EvidenceRecord rec;
    rec.identity.agent_id = agent_id;
    rec.identity.agent_session_id = "sess-" + agent_id;
    rec.identity.field_id = EvidenceFieldId::kPosition;
    rec.identity.sequence = seq;
    rec.identity.coordinate_frame = CoordinateFrame::kWgs84;
    rec.identity.source_component = "sim_pos";
    rec.source_time.timestamp_ms = src_ms;
    rec.source_time.clock_domain = ClockDomain::kUnixEpoch;
    rec.source_time.synchronization = ClockSynchronization::kSynchronized;
    rec.source_time.clock_uncertainty_ms = 2.0;
    rec.receive_time_ms = rx_ms;
    rec.quality.estimator_healthy = true;
    rec.quality.estimator_position_ok = true;
    rec.quality.uncertainty = UncertaintyEstimate{
        .value = 0.2F,
        .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
    };
    rec.value = std::array<double, 3>{37.7749, -122.4194, 10.0};
    return rec;
}

}  // namespace

TEST_CASE("ReplayTrace JSON-lines serialization roundtrip", "[replay]") {
    ReplayTrace trace;
    trace.trace_id = "trace-test-001";

    // Add session transition
    trace.events.push_back(SessionTransitionEvent{
        .timestamp_ms = 1000,
        .agent_id = "uav-1",
        .new_session_id = "sess-uav-1",
    });

    // Add clock model update
    trace.events.push_back(ClockModelUpdateEvent{
        .timestamp_ms = 1005,
        .agent_id = "uav-1",
        .clock_state = ClockQualityState{
            .offset_estimate_ms = 0.0,
            .uncertainty_radius_ms = 2.0,
            .max_drift_rate_ppm = 10.0,
            .source_domain = ClockDomain::kUnixEpoch,
            .synchronization = ClockSynchronization::kSynchronized,
            .last_update_ms = 1000,
            .deterministic_bound = true,
            .agent_incarnation_id = "sess-uav-1",
            .clock_model_version = "clock-v1",
        },
    });

    // Add evidence received
    trace.events.push_back(EvidenceReceivedEvent{
        .receive_time_ms = 1020,
        .agent_id = "uav-1",
        .record = MakeSampleRecord("uav-1", 1, 1000, 1020),
    });

    // Add snapshot request
    trace.events.push_back(SnapshotRequestEvent{
        .request_id = "req-001",
        .evaluation_time_ms = 1100.0,
        .evidence_freeze_ms = 1100,
        .contract_hash = "fake-hash",
        .participants = {
            .agent_ids = {"uav-1"},
            .membership_revision = 1,
        },
    });

    std::string json_lines = trace.ToJsonLines();
    REQUIRE_FALSE(json_lines.empty());

    auto loaded = ReplayTrace::FromJsonLines(json_lines);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->trace_id == "trace-test-001");
    REQUIRE(loaded->events.size() == 4);
}

TEST_CASE("ReplayTrace file persist, reload, and independent verification (P0.9)", "[replay]") {
    const std::string trace_path = (std::filesystem::temp_directory_path() / "test_replay_trace.jsonl").string();

    // 1. Run live state acceptance
    StateAcceptanceEngine engine;
    EvidenceStore store;
    store.SetCurrentSession("uav-1", "sess-uav-1");

    auto rec1 = MakeSampleRecord("uav-1", 1, 1000, 1020);
    store.Insert("uav-1", rec1);

    StateQualityContract contract{
        .contract_id = "sqc-replay-test",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 5.0,
        .require_estimator_position_ok = true,
        .required_position_frame = CoordinateFrame::kWgs84,
        .require_current_epoch = true,
        .required_agents = {"uav-1"},
        .completeness = CompletenessRule::kAllRequired,
        .require_deterministic_bounds = true,
        .max_horizontal_speed_mps = 10.0F,
    };

    std::unordered_map<std::string, ClockQualityState> clock_states;
    clock_states["uav-1"] = ClockQualityState{
        .offset_estimate_ms = 0.0,
        .uncertainty_radius_ms = 2.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 1000,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-uav-1",
    };

    SnapshotRequestContext req_ctx{
        .evaluation_time_ms = 1100.0,
        .evidence_freeze_ms = 1100,
        .participants = {
            .agent_ids = {"uav-1"},
            .membership_revision = 1,
        },
    };

    auto res = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(res));
    const auto& snapshot = std::get<AcceptedSnapshot>(res);
    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    // 2. Persist trace to file
    ReplayTrace trace;
    trace.trace_id = "persisted-trace-1";
    trace.events.push_back(SessionTransitionEvent{
        .timestamp_ms = 1000,
        .agent_id = "uav-1",
        .new_session_id = "sess-uav-1",
    });
    trace.events.push_back(EvidenceReceivedEvent{
        .receive_time_ms = 1020,
        .agent_id = "uav-1",
        .record = rec1,
    });
    // Add a post-r* packet that should NOT influence replay verification
    auto rec_future = MakeSampleRecord("uav-1", 2, 1150, 1160);
    trace.events.push_back(EvidenceReceivedEvent{
        .receive_time_ms = 1160,
        .agent_id = "uav-1",
        .record = rec_future,
    });

    REQUIRE(trace.SaveToFile(trace_path));

    // 3. Destroy all live state and reload from file
    auto loaded_trace = ReplayTrace::LoadFromFile(trace_path);
    REQUIRE(loaded_trace.has_value());

    EvidenceStore replayed_store;
    for (const auto& ev : loaded_trace->events) {
        if (std::holds_alternative<SessionTransitionEvent>(ev)) {
            const auto& st = std::get<SessionTransitionEvent>(ev);
            replayed_store.SetCurrentSession(st.agent_id, st.new_session_id);
        } else if (std::holds_alternative<EvidenceReceivedEvent>(ev)) {
            const auto& er = std::get<EvidenceReceivedEvent>(ev);
            replayed_store.Insert(er.agent_id, er.record);
        }
    }

    // 4. Verify loaded certificate against replayed store
    StateAcceptanceVerifier verifier;
    auto ver_res = verifier.Verify(cert, replayed_store, contract, req_ctx, clock_states);
    REQUIRE(std::holds_alternative<VerifiedAcceptance>(ver_res));

    // Clean up
    std::filesystem::remove(trace_path);
}
