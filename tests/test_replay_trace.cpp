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

TEST_CASE("test_replay_roundtrip_every_event_field", "[replay][roundtrip]") {
    auto record = MakeSampleRecord("uav-1", 41, 1000, 1020);
    record.identity.estimator_id = "estimator-A";
    record.identity.mission_id = "mission-A";
    record.identity.mission_revision = 9;
    record.identity.uncertainty_kind = UncertaintySemantics::kDeterministicHardBound;
    record.quality.estimator_velocity_ok = true;
    record.quality.uncertainty = UncertaintyEstimate{
        .value = 0.25F,
        .descriptor = {
            .semantics = UncertaintySemantics::kDeterministicHardBound,
            .confidence_level = 0.99,
            .calibration_profile_id = "profile-A",
            .calibration_version = "cal-v2",
            .source = "sim-calibration",
            .measurement_generation = 12,
        },
    };
    const ClockQualityState clock{
        .offset_estimate_ms = 3.25,
        .uncertainty_radius_ms = 1.75,
        .max_drift_rate_ppm = 22.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kEstimated,
        .last_update_ms = 900,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-uav-1",
        .clock_model_version = "clock-model-v7",
    };
    StateQualityContract contract{
        .contract_id = "roundtrip-contract",
        .required_fields = {EvidenceFieldId::kPosition},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 20.0,
        .required_position_frame = CoordinateFrame::kWgs84,
        .required_agents = {"uav-1"},
        .require_deterministic_bounds = true,
    };
    EvidenceStore store;
    store.SetCurrentSession("uav-1", "sess-uav-1");
    store.Insert("uav-1", record);
    const SnapshotRequestContext request{
        .evaluation_time_ms = 1100.5,
        .evidence_freeze_ms = 1101,
        .participants = {.agent_ids = {"uav-1"}, .membership_revision = 23},
    };
    const std::unordered_map<std::string, ClockQualityState> clocks{{"uav-1", clock}};
    const auto decision = StateAcceptanceEngine{}.RequestSnapshot(contract, request, store, clocks);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(decision));
    const auto certificate = BuildCertificate(
        std::get<AcceptedSnapshot>(decision), contract, request);

    ReplayTrace original{
        .trace_id = "complete-roundtrip",
        .version = "2.0",
        .events = {
            MembershipChangeEvent{.timestamp_ms = 890, .participants = request.participants},
            SessionTransitionEvent{.timestamp_ms = 891, .agent_id = "uav-1",
                                   .new_session_id = "sess-uav-1"},
            ClockModelUpdateEvent{.timestamp_ms = 900, .agent_id = "uav-1",
                                  .clock_state = clock},
            EvidenceReceivedEvent{.receive_time_ms = 1020, .agent_id = "uav-1",
                                  .record = record},
            SnapshotRequestEvent{.request_id = "request-complete",
                                 .evaluation_time_ms = request.evaluation_time_ms,
                                 .evidence_freeze_ms = request.evidence_freeze_ms,
                                 .contract_hash = ComputeContractHash(contract),
                                 .participants = request.participants,
                                 .certificate = certificate},
        },
    };
    const auto loaded = ReplayTrace::FromJsonLines(original.ToJsonLines());
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->trace_id == original.trace_id);
    REQUIRE(loaded->version == original.version);
    REQUIRE(loaded->events == original.events);
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

TEST_CASE("Critical r* evidence freeze frontier exclusion test", "[replay][frontier]") {
    // Generate trace:
    // Packet A: s = 90, rx = 95
    // Packet B: s = 95, rx = 105
    // Query: t* = 100, r* = 100
    // Packet B satisfies causality (s=95 <= t*=100), but rx=105 > r*=100.
    // Packet A satisfies causality AND rx <= r*.
    // Verify that Packet B is excluded SOLELY because rx > r*, and Packet A is selected.

    StateAcceptanceEngine engine;
    EvidenceStore store;
    store.SetCurrentSession("uav-1", "sess-uav-1");

    auto rec_a = MakeSampleRecord("uav-1", 1, 90, 95);
    rec_a.value = std::array<double, 3>{10.0, 20.0, 30.0};
    auto rec_b = MakeSampleRecord("uav-1", 2, 95, 105);
    rec_b.value = std::array<double, 3>{50.0, 60.0, 70.0};

    store.Insert("uav-1", rec_a);
    store.Insert("uav-1", rec_b);

    StateQualityContract contract{
        .contract_id = "sqc-rstar-test",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 100.0,
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
        .uncertainty_radius_ms = 1.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 50,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-uav-1",
    };

    SnapshotRequestContext req_ctx{
        .evaluation_time_ms = 100.0,
        .evidence_freeze_ms = 100,  // r* = 100 excludes rec_b (rx = 105)
        .participants = {
            .agent_ids = {"uav-1"},
            .membership_revision = 1,
        },
    };

    auto res = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(res));
    const auto& snapshot = std::get<AcceptedSnapshot>(res);

    // Selected evidence MUST be Packet A (sequence 1), NOT Packet B (sequence 2)
    const auto& agent_map = snapshot.agent_states.at("uav-1");
    const auto& pos_state = agent_map.at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    REQUIRE(pos_state.evidence.identity.sequence == 1);
    REQUIRE(pos_state.evidence.receive_time_ms == 95);

    // Build and verify certificate independently
    auto cert = BuildCertificate(snapshot, contract, req_ctx);
    StateAcceptanceVerifier verifier;
    auto ver_res = verifier.Verify(cert, store, contract, req_ctx, clock_states);
    REQUIRE(std::holds_alternative<VerifiedAcceptance>(ver_res));
}

TEST_CASE("test_persisted_replay_post_rstar_pre_tstar_evidence_is_excluded",
          "[replay][frontier][persisted]") {
    const std::string trace_path =
        (std::filesystem::temp_directory_path() /
         "test_persisted_rstar_frontier.jsonl")
            .string();
    StateQualityContract contract{
        .contract_id = "sqc-persisted-rstar",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 100.0,
        .require_estimator_position_ok = true,
        .required_position_frame = CoordinateFrame::kWgs84,
        .require_current_epoch = true,
        .required_agents = {"uav-1"},
        .completeness = CompletenessRule::kAllRequired,
        .require_deterministic_bounds = true,
        .max_horizontal_speed_mps = 10.0F,
    };

    {
        EvidenceStore live_store;
        live_store.SetCurrentSession("uav-1", "sess-uav-1");
        auto evidence_a = MakeSampleRecord("uav-1", 1, 900, 1050);
        auto evidence_b = MakeSampleRecord("uav-1", 2, 950, 1200);
        live_store.Insert("uav-1", evidence_a);
        live_store.Insert("uav-1", evidence_b);

        std::unordered_map<std::string, ClockQualityState> live_clocks;
        live_clocks["uav-1"] = ClockQualityState{
            .offset_estimate_ms = 0.0,
            .uncertainty_radius_ms = 1.0,
            .max_drift_rate_ppm = 20.0,
            .source_domain = ClockDomain::kUnixEpoch,
            .synchronization = ClockSynchronization::kSynchronized,
            .last_update_ms = 800,
            .deterministic_bound = true,
            .agent_incarnation_id = "sess-uav-1",
            .clock_model_version = "clock-v1",
        };
        SnapshotRequestContext live_request{
            .evaluation_time_ms = 1000.0,
            .evidence_freeze_ms = 1100,
            .participants = {
                .agent_ids = {"uav-1"},
                .membership_revision = 17,
            },
        };
        const auto result = StateAcceptanceEngine{}.RequestSnapshot(
            contract, live_request, live_store, live_clocks);
        REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
        const auto& live_snapshot = std::get<AcceptedSnapshot>(result);
        REQUIRE(live_snapshot.agent_states.at("uav-1")
                    .at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition))
                    .evidence.identity.sequence == 1);
        const auto live_certificate =
            BuildCertificate(live_snapshot, contract, live_request);

        ReplayTrace trace;
        trace.trace_id = "persisted-rstar-frontier";
        trace.events.push_back(MembershipChangeEvent{
            .timestamp_ms = 800,
            .participants = live_request.participants,
        });
        trace.events.push_back(SessionTransitionEvent{
            .timestamp_ms = 800,
            .agent_id = "uav-1",
            .new_session_id = "sess-uav-1",
        });
        trace.events.push_back(ClockModelUpdateEvent{
            .timestamp_ms = 800,
            .agent_id = "uav-1",
            .clock_state = live_clocks.at("uav-1"),
        });
        trace.events.push_back(EvidenceReceivedEvent{
            .receive_time_ms = evidence_a.receive_time_ms,
            .agent_id = "uav-1",
            .record = evidence_a,
        });
        trace.events.push_back(EvidenceReceivedEvent{
            .receive_time_ms = evidence_b.receive_time_ms,
            .agent_id = "uav-1",
            .record = evidence_b,
        });
        trace.events.push_back(SnapshotRequestEvent{
            .request_id = "request-rstar",
            .evaluation_time_ms = live_request.evaluation_time_ms,
            .evidence_freeze_ms = live_request.evidence_freeze_ms,
            .contract_hash = ComputeContractHash(contract),
            .participants = live_request.participants,
            .certificate = live_certificate,
        });
        REQUIRE(trace.SaveToFile(trace_path));
    }  // Destroy the live store, sessions, clocks, request, snapshot, and certificate.

    const auto loaded = ReplayTrace::LoadFromFile(trace_path);
    REQUIRE(loaded.has_value());
    EvidenceStore replay_store;
    std::unordered_map<std::string, ClockQualityState> replay_clocks;
    std::optional<ParticipantSnapshot> replay_membership;
    std::optional<SnapshotRequestEvent> replay_request;
    for (const auto& event : loaded->events) {
        std::visit([&](const auto& item) {
            using T = std::decay_t<decltype(item)>;
            if constexpr (std::is_same_v<T, MembershipChangeEvent>) {
                replay_membership = item.participants;
            } else if constexpr (std::is_same_v<T, SessionTransitionEvent>) {
                replay_store.SetCurrentSession(item.agent_id, item.new_session_id);
            } else if constexpr (std::is_same_v<T, ClockModelUpdateEvent>) {
                replay_clocks[item.agent_id] = item.clock_state;
            } else if constexpr (std::is_same_v<T, EvidenceReceivedEvent>) {
                replay_store.Insert(item.agent_id, item.record);
            } else if constexpr (std::is_same_v<T, SnapshotRequestEvent>) {
                replay_request = item;
            }
        }, event);
    }

    REQUIRE(replay_membership.has_value());
    REQUIRE(replay_request.has_value());
    REQUIRE(replay_request->certificate.has_value());
    REQUIRE(VerifySnapshotRequestContractBinding(*replay_request, contract));
    REQUIRE(replay_request->participants == *replay_membership);
    REQUIRE(replay_request->certificate->evidence_entries.at(0).sequence == 1);

    const SnapshotRequestContext reconstructed_context{
        .evaluation_time_ms = replay_request->evaluation_time_ms,
        .evidence_freeze_ms = replay_request->evidence_freeze_ms,
        .participants = replay_request->participants,
    };
    const auto verified = StateAcceptanceVerifier{}.Verify(
        *replay_request->certificate, replay_store, contract,
        reconstructed_context, replay_clocks);
    REQUIRE(std::holds_alternative<VerifiedAcceptance>(verified));
    REQUIRE(std::get<VerifiedAcceptance>(verified)
                .reconstructed_snapshot.agent_states.at("uav-1")
                .at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition))
                .evidence.identity.sequence == 1);
    std::filesystem::remove(trace_path);
}

TEST_CASE("test_replay_contract_hash_is_canonical_hash", "[replay][contract]") {
    StateQualityContract contract{
        .contract_id = "fixed-contract",
        .required_fields = {EvidenceFieldId::kPosition},
        .max_position_uncertainty_m = 3.0,
        .required_agents = {"uav-1"},
    };
    SnapshotRequestEvent event{
        .request_id = "request-1",
        .evaluation_time_ms = 1000.0,
        .evidence_freeze_ms = 1000,
        .contract_hash = ComputeContractHash(contract),
        .participants = {.agent_ids = {"uav-1"}, .membership_revision = 1},
    };
    REQUIRE(VerifySnapshotRequestContractBinding(event, contract));
    auto changed_contract = contract;
    changed_contract.max_position_uncertainty_m = 3.001;
    REQUIRE_FALSE(VerifySnapshotRequestContractBinding(event, changed_contract));
}
