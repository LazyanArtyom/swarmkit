// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_acceptance_certificate.h"

using namespace swarmkit::core;
using Catch::Matchers::WithinAbs;

namespace {

TelemetryFrame CreateTestFrame(
    const std::string& drone_id,
    const std::string& session_id,
    std::uint64_t seq,
    std::int64_t source_time_ms,
    std::int64_t receive_time_ms = 1000) {

    TelemetryFrame frame;
    frame.drone_id = drone_id;
    frame.agent_session_id = session_id;
    frame.telemetry_sequence = seq;
    frame.agent_receive_unix_time_ms = receive_time_ms;

    // Position
    frame.lat_deg = 37.7749;
    frame.lon_deg = -122.4194;
    frame.rel_alt_m = 10.0F;
    frame.position_frame = CoordinateFrame::kWgs84;
    frame.validity.position = true;
    frame.provenance.position.source_time.timestamp_ms = source_time_ms;
    frame.provenance.position.source_time.clock_domain = ClockDomain::kUnixEpoch;
    frame.provenance.position.source_time.synchronization = ClockSynchronization::kSynchronized;
    frame.provenance.position.source_time.clock_uncertainty_ms = 2.0;
    frame.provenance.position.source = "ekf";
    frame.accuracy.horizontal_position = UncertaintyEstimate{
        .value = 0.2F,
        .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
    };

    // Velocity
    frame.vx_mps = 1.0F;
    frame.vy_mps = 0.0F;
    frame.vz_mps = 0.0F;
    frame.velocity_frame = CoordinateFrame::kLocalNed;
    frame.validity.velocity = true;
    frame.provenance.velocity.source_time.timestamp_ms = source_time_ms;
    frame.provenance.velocity.source_time.clock_domain = ClockDomain::kUnixEpoch;
    frame.provenance.velocity.source_time.synchronization = ClockSynchronization::kSynchronized;
    frame.provenance.velocity.source_time.clock_uncertainty_ms = 2.0;
    frame.provenance.velocity.source = "ekf";
    frame.accuracy.horizontal_velocity = UncertaintyEstimate{
        .value = 0.1F,
        .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
    };

    // Estimator
    frame.estimator_state = EstimatorState::kHealthy;
    frame.estimator_position_ok = true;
    frame.estimator_velocity_ok = true;
    frame.validity.estimator = true;
    frame.provenance.estimator.source_time.timestamp_ms = source_time_ms;

    return frame;
}

StateQualityContract CreateStandardContract(const std::unordered_set<std::string>& agents = {"uav-1"}) {
    return StateQualityContract{
        .contract_id = "sqc-standard",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition, EvidenceFieldId::kVelocity},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 5.0,
        .max_velocity_uncertainty_mps = 2.0,
        .require_estimator_position_ok = true,
        .require_estimator_velocity_ok = true,
        .require_estimator_healthy = true,
        .required_position_frame = CoordinateFrame::kWgs84,
        .required_velocity_frame = CoordinateFrame::kLocalNed,
        .require_current_epoch = true,
        .required_agents = agents,
        .completeness = CompletenessRule::kAllRequired,
        .require_deterministic_bounds = true,
        .max_horizontal_speed_mps = 10.0F,
        .max_vertical_speed_mps = 0.0F,
    };
}

std::unordered_map<std::string, ClockQualityState> CreateStandardClockStates(
    const std::unordered_set<std::string>& agents = {"uav-1"},
    const std::string& session_id = "sess-1") {
    std::unordered_map<std::string, ClockQualityState> map;
    for (const auto& a : agents) {
        map[a] = ClockQualityState{
            .offset_estimate_ms = 0.0,
            .uncertainty_radius_ms = 2.0,
            .source_domain = ClockDomain::kUnixEpoch,
            .synchronization = ClockSynchronization::kSynchronized,
            .last_update_ms = 500,
            .deterministic_bound = true,
            .agent_incarnation_id = session_id,
        };
    }
    return map;
}

SnapshotRequestContext MakeReqCtx(double t_star, std::int64_t r_star = 0, std::vector<std::string> agents = {"uav-1"}) {
    if (r_star == 0) r_star = static_cast<std::int64_t>(t_star) + 1000;
    return SnapshotRequestContext{
        .evaluation_time_ms = t_star,
        .evidence_freeze_ms = r_star,
        .participants = {
            .agent_ids = std::move(agents),
            .membership_revision = 1,
        },
    };
}

EvidenceRecord MakeGpsRecord(EvidenceValue value, std::uint64_t sequence = 1) {
    return EvidenceRecord{
        .value = std::move(value),
        .source_time = {
            .timestamp_ms = 1000,
            .clock_domain = ClockDomain::kUnixEpoch,
            .synchronization = ClockSynchronization::kSynchronized,
        },
        .receive_time_ms = 1020,
        .quality = {
            .estimator_healthy = true,
            .estimator_position_ok = true,
            .estimator_velocity_ok = true,
        },
        .identity = {
            .agent_id = "uav-1",
            .agent_session_id = "sess-1",
            .field_id = EvidenceFieldId::kGpsQuality,
            .sequence = sequence,
            .source_component = "gps",
        },
    };
}

}  // namespace

TEST_CASE("StateAcceptanceEngine single-UAV acceptance", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    // Insert frame for uav-1 with source_time = 1000 ms, receive_time = 1020 ms
    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    store.InsertFrame(frame);

    // Evaluation time t* = 1100 ms, r* = 1100 ms
    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& snapshot = std::get<AcceptedSnapshot>(result);

    REQUIRE(snapshot.contract_id == "sqc-standard");
    REQUIRE_THAT(snapshot.evaluation_time_ms, WithinAbs(1100.0, 1e-9));
    REQUIRE(snapshot.evidence_freeze_ms == 1100);
    REQUIRE(snapshot.accepted_agents.size() == 1);
    REQUIRE(snapshot.accepted_agents[0] == "uav-1");

    auto agent_it = snapshot.agent_states.find("uav-1");
    REQUIRE(agent_it != snapshot.agent_states.end());

    auto pos_it = agent_it->second.find(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    REQUIRE(pos_it != agent_it->second.end());
    REQUIRE_THAT(pos_it->second.conservative_elapsed_ms, WithinAbs(102.0, 1e-6));
    REQUIRE_THAT(pos_it->second.propagated_uncertainty, WithinAbs(1.22, 1e-6));
    REQUIRE_THAT(pos_it->second.observation_uncertainty, WithinAbs(0.2, 1e-6));
    REQUIRE_THAT(pos_it->second.generation_interval.lower_ms, WithinAbs(998.0, 1e-6));
    REQUIRE_THAT(pos_it->second.generation_interval.upper_ms, WithinAbs(1002.0, 1e-6));
}

TEST_CASE("StateAcceptanceEngine multi-UAV common-time acceptance", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 10, 950, 980));
    store.InsertFrame(CreateTestFrame("uav-2", "sess-2", 20, 980, 1000));
    store.InsertFrame(CreateTestFrame("uav-3", "sess-3", 30, 920, 960));

    auto contract = CreateStandardContract({"uav-1", "uav-2", "uav-3"});
    std::unordered_map<std::string, ClockQualityState> clock_states;
    clock_states["uav-1"] = ClockQualityState{.uncertainty_radius_ms = 2.0, .source_domain = ClockDomain::kUnixEpoch, .synchronization = ClockSynchronization::kSynchronized, .last_update_ms = 500, .deterministic_bound = true, .agent_incarnation_id = "sess-1"};
    clock_states["uav-2"] = ClockQualityState{.uncertainty_radius_ms = 2.0, .source_domain = ClockDomain::kUnixEpoch, .synchronization = ClockSynchronization::kSynchronized, .last_update_ms = 500, .deterministic_bound = true, .agent_incarnation_id = "sess-2"};
    clock_states["uav-3"] = ClockQualityState{.uncertainty_radius_ms = 2.0, .source_domain = ClockDomain::kUnixEpoch, .synchronization = ClockSynchronization::kSynchronized, .last_update_ms = 500, .deterministic_bound = true, .agent_incarnation_id = "sess-3"};

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1050.0, 1050, {"uav-1", "uav-2", "uav-3"}), store, clock_states);

    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& snapshot = std::get<AcceptedSnapshot>(result);
    REQUIRE(snapshot.accepted_agents.size() == 3);
}

TEST_CASE("StateAcceptanceEngine rejection on missing evidence", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1000.0, 1000), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    REQUIRE_FALSE(rej.failures.empty());
    REQUIRE(rej.failures[0].reason == RejectionReason::kMissingRequiredEvidence);
}

TEST_CASE("StateAcceptanceEngine rejection on non-causal evidence (g^+ > t*)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    // Evidence generated at source_time = 1200 ms (g^+ = 1202 ms)
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1200, 1220));

    // Request snapshot at evaluation time t* = 1100 ms
    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1300), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    REQUIRE_FALSE(rej.failures.empty());
    REQUIRE(rej.failures[0].reason == RejectionReason::kCausalSampleUnavailable);
}

TEST_CASE("StateAcceptanceEngine receive frontier r* enforcement (P0.1)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    // Evidence generated at source_time = 1000 ms, but received at receive_time = 1200 ms
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1200));

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    // Request with evidence-freeze cutoff r* = 1100 ms (before frame was received)
    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    REQUIRE_FALSE(rej.failures.empty());
    REQUIRE(rej.failures[0].reason == RejectionReason::kCausalSampleUnavailable);

    // If we request with r* = 1250 ms, it should be accepted
    auto result_ok = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1250), store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result_ok));
}

TEST_CASE("StateAcceptanceEngine deterministic selector tie-breaking (P0.5)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store(EvidenceStoreConfig{.max_records_per_field = 10});

    // Insert two records with equal generation intervals (same source time, same clock)
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 10, 1000, 1010));
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 20, 1000, 1020));

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& snap = std::get<AcceptedSnapshot>(result);

    // Sequence 20 must win tie-break over sequence 10
    auto pos_it = snap.agent_states.at("uav-1").at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    REQUIRE(pos_it.evidence.identity.sequence == 20);
}

TEST_CASE("StateAcceptanceEngine context-invalid pre-filtering (P0.6)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store(EvidenceStoreConfig{.max_records_per_field = 10});

    // Old frame from obsolete session with higher sequence and newer source time
    store.InsertFrame(CreateTestFrame("uav-1", "sess-old", 50, 1050, 1060));

    // Valid frame from current session with lower sequence and slightly older source time
    store.InsertFrame(CreateTestFrame("uav-1", "sess-new", 10, 1000, 1020));
    store.SetCurrentSession("uav-1", "sess-new");

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-new");

    // The old-session record should be filtered BEFORE ranking, so it does not displace
    // or block the valid new-session record.
    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& snap = std::get<AcceptedSnapshot>(result);

    auto pos_it = snap.agent_states.at("uav-1").at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    REQUIRE(pos_it.evidence.identity.agent_session_id == "sess-new");
    REQUIRE(pos_it.evidence.identity.sequence == 10);
}

TEST_CASE("StateAcceptanceEngine clock drift expands effective rho (P0.4)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    frame.provenance.position.source_time.clock_uncertainty_ms = std::nullopt;
    frame.provenance.velocity.source_time.clock_uncertainty_ms = std::nullopt;
    store.InsertFrame(frame);

    auto contract = CreateStandardContract({"uav-1"});

    // Clock state with 100 ppm drift rate, last updated at 900 ms
    std::unordered_map<std::string, ClockQualityState> clock_states;
    clock_states["uav-1"] = ClockQualityState{
        .offset_estimate_ms = 0.0,
        .uncertainty_radius_ms = 2.0,
        .max_drift_rate_ppm = 100.0, // 100 ppm
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 900,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-1",
    };

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& snap = std::get<AcceptedSnapshot>(result);

    auto pos_it = snap.agent_states.at("uav-1").at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    // Effective rho = 2.0 + 100 * 1e-6 * (1000 - 900) = 2.0 + 0.01 = 2.01 ms
    REQUIRE_THAT(pos_it.clock_evaluation.effective_rho_ms, WithinAbs(2.01, 1e-6));
}

TEST_CASE("StateAcceptanceEngine rejection on evidence age exceeded", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1600.0, 1600), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    auto it = std::find_if(rej.failures.begin(), rej.failures.end(), [](const PredicateFailure& f) {
        return f.reason == RejectionReason::kAgeExceeded;
    });
    REQUIRE(it != rej.failures.end());
}

TEST_CASE("StateAcceptanceEngine rejection on clock uncertainty exceeded", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    store.InsertFrame(frame);

    auto contract = CreateStandardContract({"uav-1"});
    // Clock uncertainty is 15 ms > contract limit (10 ms)
    std::unordered_map<std::string, ClockQualityState> clock_states;
    clock_states["uav-1"] = ClockQualityState{
        .offset_estimate_ms = 0.0,
        .uncertainty_radius_ms = 15.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 500,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-1",
    };

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    auto it = std::find_if(rej.failures.begin(), rej.failures.end(), [](const PredicateFailure& f) {
        return f.reason == RejectionReason::kClockUncertaintyExceeded;
    });
    REQUIRE(it != rej.failures.end());
}

TEST_CASE("StateAcceptanceEngine rejection on propagated uncertainty exceeded", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    frame.accuracy.horizontal_position = UncertaintyEstimate{
        .value = 4.0F,
        .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
    };
    store.InsertFrame(frame);

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1200.0, 1200), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    auto it = std::find_if(rej.failures.begin(), rej.failures.end(), [](const PredicateFailure& f) {
        return f.reason == RejectionReason::kStateUncertaintyExceeded;
    });
    REQUIRE(it != rej.failures.end());
}

TEST_CASE("StateAcceptanceEngine rejection on unhealthy estimator", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    frame.estimator_state = EstimatorState::kDegraded;
    frame.estimator_position_ok = false;
    store.InsertFrame(frame);

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    auto it = std::find_if(rej.failures.begin(), rej.failures.end(), [](const PredicateFailure& f) {
        return f.reason == RejectionReason::kEstimatorUnhealthy;
    });
    REQUIRE(it != rej.failures.end());
}

TEST_CASE("StateAcceptanceEngine rejection on frame mismatch", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    frame.position_frame = CoordinateFrame::kLocalNed;
    store.InsertFrame(frame);

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rej = std::get<StructuredRejection>(result);
    REQUIRE_FALSE(rej.failures.empty());
}

TEST_CASE("StateAcceptanceEngine rejection on stale agent epoch (E_msg != E_cur)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame_old = CreateTestFrame("uav-1", "session-old", 1, 1000, 1020);
    store.InsertFrame(frame_old);

    TelemetryFrame frame_new;
    frame_new.drone_id = "uav-1";
    frame_new.agent_session_id = "session-new";
    frame_new.telemetry_sequence = 2;
    frame_new.agent_receive_unix_time_ms = 1050;
    frame_new.validity.battery = true;
    frame_new.battery_percent = 95.0F;
    store.InsertFrame(frame_new);
    store.SetCurrentSession("uav-1", "session-new");

    REQUIRE(store.CurrentSessionId("uav-1") == "session-new");

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "session-new");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
}

TEST_CASE("StateAcceptanceEngine rejection on non-deterministic uncertainty semantics", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    frame.accuracy.horizontal_position = UncertaintyEstimate{
        .value = 0.2F,
        .descriptor = {.semantics = UncertaintySemantics::kStandardDeviation},
    };
    store.InsertFrame(frame);

    auto contract = CreateStandardContract({"uav-1"});
    contract.require_deterministic_bounds = true;
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100), store, clock_states);

    REQUIRE(std::holds_alternative<StructuredRejection>(result));
}

TEST_CASE("StateAcceptanceEngine completeness rules", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    store.InsertFrame(CreateTestFrame("uav-2", "sess-2", 1, 1000, 1020));

    std::unordered_map<std::string, ClockQualityState> clock_states;
    clock_states["uav-1"] = ClockQualityState{.uncertainty_radius_ms = 2.0, .source_domain = ClockDomain::kUnixEpoch, .synchronization = ClockSynchronization::kSynchronized, .last_update_ms = 500, .deterministic_bound = true, .agent_incarnation_id = "sess-1"};
    clock_states["uav-2"] = ClockQualityState{.uncertainty_radius_ms = 2.0, .source_domain = ClockDomain::kUnixEpoch, .synchronization = ClockSynchronization::kSynchronized, .last_update_ms = 500, .deterministic_bound = true, .agent_incarnation_id = "sess-2"};
    clock_states["uav-3"] = ClockQualityState{.uncertainty_radius_ms = 2.0, .source_domain = ClockDomain::kUnixEpoch, .synchronization = ClockSynchronization::kSynchronized, .last_update_ms = 500, .deterministic_bound = true, .agent_incarnation_id = "sess-3"};

    SECTION("kAllRequired fails when 1 of 3 missing") {
        auto contract = CreateStandardContract({"uav-1", "uav-2", "uav-3"});
        contract.completeness = CompletenessRule::kAllRequired;

        auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100, {"uav-1", "uav-2", "uav-3"}), store, clock_states);
        REQUIRE(std::holds_alternative<StructuredRejection>(result));
    }

    SECTION("kMinimumCount succeeds when min threshold met") {
        auto contract = CreateStandardContract({"uav-1", "uav-2", "uav-3"});
        contract.completeness = CompletenessRule::kMinimumCount;
        contract.min_required_agents = 2;

        auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1100, {"uav-1", "uav-2", "uav-3"}), store, clock_states);
        REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
        const auto& snap = std::get<AcceptedSnapshot>(result);
        REQUIRE(snap.accepted_agents.size() == 2);
    }
}

TEST_CASE("StateAcceptanceEngine causal search beyond 16 records and reordering (P0.4)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store(EvidenceStoreConfig{.max_records_per_field = 50});

    // Insert 1 causal record at source_time = 1000 ms (seq 1)
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1010));

    // Insert 20 non-causal records at source_time = 1200..1220 ms (g^+ > t* = 1100 ms)
    for (int i = 2; i <= 21; ++i) {
        store.InsertFrame(CreateTestFrame("uav-1", "sess-1", i, 1200 + i, 1300 + i));
    }

    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto result = engine.RequestSnapshot(contract, MakeReqCtx(1100.0, 1400), store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& snap = std::get<AcceptedSnapshot>(result);
    REQUIRE(snap.accepted_agents.size() == 1);
    auto pos_it = snap.agent_states.at("uav-1").at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    REQUIRE(pos_it.evidence.identity.sequence == 1);
}

TEST_CASE("StateAcceptanceEngine statelessness and deterministic repeatability (P0.7)", "[acceptance_engine]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    auto contract = CreateStandardContract({"uav-1"});
    auto clock_states = CreateStandardClockStates({"uav-1"}, "sess-1");

    auto req_ctx = MakeReqCtx(1100.0, 1100);
    auto res1 = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
    auto res2 = engine.RequestSnapshot(contract, req_ctx, store, clock_states);

    REQUIRE(std::holds_alternative<AcceptedSnapshot>(res1));
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(res2));

    const auto& snap1 = std::get<AcceptedSnapshot>(res1);
    const auto& snap2 = std::get<AcceptedSnapshot>(res2);

    REQUIRE(snap1.snapshot_id == snap2.snapshot_id);
    REQUIRE(snap1.contract_hash == snap2.contract_hash);
    REQUIRE(snap1.evaluation_time_ms == snap2.evaluation_time_ms);
}

TEST_CASE("test_engine_min_gps_quality_rejects_below_threshold", "[acceptance_engine][gps]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    store.Insert("uav-1", MakeGpsRecord(GpsQuality::kFix2D));

    auto contract = CreateStandardContract();
    contract.min_gps_quality = GpsQuality::kFix3D;
    const auto result = engine.RequestSnapshot(
        contract, MakeReqCtx(1100.0, 1100), store,
        CreateStandardClockStates());
    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rejection = std::get<StructuredRejection>(result);
    REQUIRE(std::any_of(rejection.failures.begin(), rejection.failures.end(), [](const auto& failure) {
        return failure.reason == RejectionReason::kGpsQualityInsufficient;
    }));
}

TEST_CASE("test_engine_min_gps_quality_accepts_at_threshold", "[acceptance_engine][gps]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    store.Insert("uav-1", MakeGpsRecord(GpsQuality::kFix3D));

    auto contract = CreateStandardContract();
    contract.min_gps_quality = GpsQuality::kFix3D;
    const auto result = engine.RequestSnapshot(
        contract, MakeReqCtx(1100.0, 1100), store,
        CreateStandardClockStates());
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    REQUIRE(std::get<AcceptedSnapshot>(result)
                .agent_states.at("uav-1")
                .contains(static_cast<std::uint8_t>(EvidenceFieldId::kGpsQuality)));
}

TEST_CASE("test_gps_wrong_variant_rejects", "[acceptance_engine][gps]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    store.Insert("uav-1", MakeGpsRecord(true));

    auto contract = CreateStandardContract();
    contract.min_gps_quality = GpsQuality::kFix3D;
    const auto result = engine.RequestSnapshot(
        contract, MakeReqCtx(1100.0, 1100), store,
        CreateStandardClockStates());
    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rejection = std::get<StructuredRejection>(result);
    REQUIRE(std::any_of(rejection.failures.begin(), rejection.failures.end(), [](const auto& failure) {
        return failure.reason == RejectionReason::kGpsQualityInsufficient;
    }));
}

TEST_CASE("test_deterministic_missing_clock_model_rejects", "[acceptance_engine][clock]") {
    EvidenceStore store;
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    const auto result = StateAcceptanceEngine{}.RequestSnapshot(
        CreateStandardContract(), MakeReqCtx(1100.0, 1100), store, {});
    REQUIRE(std::holds_alternative<StructuredRejection>(result));
    const auto& rejection = std::get<StructuredRejection>(result);
    REQUIRE(std::any_of(rejection.failures.begin(), rejection.failures.end(), [](const auto& failure) {
        return failure.reason == RejectionReason::kMissingClockQuality;
    }));
}

TEST_CASE("test_per_sample_uncertainty_does_not_assume_theta_zero",
          "[acceptance_engine][clock]") {
    EvidenceStore store;
    auto frame = CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020);
    frame.provenance.position.source_time.clock_uncertainty_ms = 0.1;
    frame.provenance.velocity.source_time.clock_uncertainty_ms = 0.1;
    store.InsertFrame(frame);
    auto contract = CreateStandardContract();
    contract.require_deterministic_bounds = true;
    const auto result = StateAcceptanceEngine{}.RequestSnapshot(
        contract, MakeReqCtx(1100.0, 1100), store, {});
    REQUIRE(std::holds_alternative<StructuredRejection>(result));
}

TEST_CASE("test_restart_requires_new_incarnation_clock", "[acceptance_engine][clock][restart]") {
    EvidenceStore store;
    store.InsertFrame(CreateTestFrame("uav-1", "E2", 2, 1000, 1020));
    store.SetCurrentSession("uav-1", "E2");
    auto old_clock = CreateStandardClockStates({"uav-1"}, "E1");
    REQUIRE(std::holds_alternative<StructuredRejection>(StateAcceptanceEngine{}.RequestSnapshot(
        CreateStandardContract(), MakeReqCtx(1100.0, 1100), store, old_clock)));

    auto new_clock = CreateStandardClockStates({"uav-1"}, "E2");
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(StateAcceptanceEngine{}.RequestSnapshot(
        CreateStandardContract(), MakeReqCtx(1100.0, 1100), store, new_clock)));
}

TEST_CASE("test_equal_gminus_equal_sequence_evidence_id_tiebreak_and_insertion_order",
          "[acceptance_engine][selector]") {
    auto frame_a = CreateTestFrame("uav-1", "sess-1", 7, 1000, 1020);
    auto frame_b = frame_a;
    frame_a.lat_deg = 1.0;
    frame_b.lat_deg = 2.0;
    auto records_a = DecomposeToEvidence(frame_a);
    auto records_b = DecomposeToEvidence(frame_b);
    const auto position = [](const auto& records) {
        return *std::find_if(records.begin(), records.end(), [](const auto& record) {
            return record.identity.field_id == EvidenceFieldId::kPosition;
        });
    };
    const auto rec_a = position(records_a);
    const auto rec_b = position(records_b);
    REQUIRE(ComputeCanonicalEvidenceId(rec_a) != ComputeCanonicalEvidenceId(rec_b));
    const auto expected = std::max(ComputeCanonicalEvidenceId(rec_a),
                                   ComputeCanonicalEvidenceId(rec_b));

    auto contract = CreateStandardContract();
    contract.required_fields = {EvidenceFieldId::kPosition};
    contract.require_estimator_velocity_ok = false;
    contract.max_velocity_uncertainty_mps.reset();
    const auto clocks = CreateStandardClockStates();
    const auto context = MakeReqCtx(1100.0, 1100);
    for (const bool reverse : {false, true}) {
        EvidenceStore store;
        store.SetCurrentSession("uav-1", "sess-1");
        store.Insert("uav-1", reverse ? rec_b : rec_a);
        store.Insert("uav-1", reverse ? rec_a : rec_b);
        const auto result = StateAcceptanceEngine{}.RequestSnapshot(contract, context, store, clocks);
        REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
        const auto& selected = std::get<AcceptedSnapshot>(result).agent_states.at("uav-1")
            .at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition)).evidence;
        REQUIRE(ComputeCanonicalEvidenceId(selected) == expected);
    }
}

TEST_CASE("test_3d_speed_bound", "[acceptance_engine][propagation]") {
    EvidenceStore store;
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 1, 1000, 1020));
    auto contract = CreateStandardContract();
    contract.required_fields = {EvidenceFieldId::kPosition};
    contract.require_estimator_velocity_ok = false;
    contract.max_velocity_uncertainty_mps.reset();
    contract.max_horizontal_speed_mps = 4.0F;
    contract.max_vertical_speed_mps = 3.0F;
    const auto result = StateAcceptanceEngine{}.RequestSnapshot(
        contract, MakeReqCtx(1100.0, 1100), store, CreateStandardClockStates());
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(result));
    const auto& selected = std::get<AcceptedSnapshot>(result).agent_states.at("uav-1")
        .at(static_cast<std::uint8_t>(EvidenceFieldId::kPosition));
    REQUIRE_THAT(selected.propagated_uncertainty, WithinAbs(0.2 + 5.0 * 0.102, 1e-6));
}
