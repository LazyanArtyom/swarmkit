// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"
#include "swarmkit/core/state_acceptance_verifier.h"

using namespace swarmkit::core;

namespace {

TelemetryFrame CreateTestFrame(
    const std::string& drone_id,
    const std::string& session_id,
    std::uint64_t seq,
    std::int64_t source_time_ms) {

    TelemetryFrame frame;
    frame.drone_id = drone_id;
    frame.agent_session_id = session_id;
    frame.telemetry_sequence = seq;
    frame.agent_receive_unix_time_ms = source_time_ms + 20;

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

    frame.estimator_state = EstimatorState::kHealthy;
    frame.estimator_position_ok = true;
    frame.validity.estimator = true;
    frame.provenance.estimator.source_time.timestamp_ms = source_time_ms;

    return frame;
}

StateQualityContract CreateTestContract() {
    return StateQualityContract{
        .contract_id = "sqc-verifier-test",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 5.0,
        .require_estimator_position_ok = true,
        .required_position_frame = CoordinateFrame::kWgs84,
        .require_current_epoch = true,
        .required_agents = {"uav-1", "uav-2"},
        .completeness = CompletenessRule::kAllRequired,
        .require_deterministic_bounds = true,
        .max_horizontal_speed_mps = 10.0F,
    };
}

}  // namespace

TEST_CASE("StateAcceptanceVerifier independent replay equivalence", "[verifier]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    // Populate trace with observations
    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 10, 1000));
    store.InsertFrame(CreateTestFrame("uav-2", "sess-2", 20, 1005));

    auto contract = CreateTestContract();
    std::unordered_map<std::string, ClockQualityState> clock_states;

    const double t_star = 1100.0;
    auto engine_result = engine.RequestSnapshot(contract, t_star, store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(engine_result));

    const auto& snapshot = std::get<AcceptedSnapshot>(engine_result);
    auto cert = BuildCertificate(snapshot, contract);

    StateAcceptanceVerifier verifier;
    auto verify_result = verifier.Verify(cert, store, contract, clock_states);

    SECTION("Verifier accepts valid certificate and reconstructed snapshot matches") {
        REQUIRE(std::holds_alternative<VerifiedAcceptance>(verify_result));
        const auto& accepted = std::get<VerifiedAcceptance>(verify_result);
        REQUIRE(accepted.certificate_id == cert.certificate_id);
        REQUIRE(accepted.reconstructed_snapshot.accepted_agents == snapshot.accepted_agents);
        REQUIRE(accepted.reconstructed_snapshot.contract_hash == snapshot.contract_hash);
    }
}

TEST_CASE("StateAcceptanceVerifier rejection on certificate tampering", "[verifier]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 10, 1000));
    store.InsertFrame(CreateTestFrame("uav-2", "sess-2", 20, 1005));

    auto contract = CreateTestContract();
    std::unordered_map<std::string, ClockQualityState> clock_states;

    auto engine_result = engine.RequestSnapshot(contract, 1100.0, store, clock_states);
    const auto& snapshot = std::get<AcceptedSnapshot>(engine_result);
    auto cert = BuildCertificate(snapshot, contract);

    StateAcceptanceVerifier verifier;

    SECTION("Tampered certificate hash") {
        auto tampered_cert = cert;
        tampered_cert.certificate_hash = "0000000000000000000000000000000000000000000000000000000000000000";

        auto result = verifier.Verify(tampered_cert, store, contract, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(result));
        const auto& rej = std::get<VerificationRejection>(result);
        REQUIRE(rej.failures[0].reason == VerificationFailureReason::kCertificateHashMismatch);
    }

    SECTION("Contract mismatch (different contract passed to verifier)") {
        auto mutated_contract = contract;
        mutated_contract.max_evidence_age_ms = 200.0;  // changes h_C

        auto result = verifier.Verify(cert, store, mutated_contract, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(result));
        const auto& rej = std::get<VerificationRejection>(result);
        REQUIRE(rej.failures[0].reason == VerificationFailureReason::kContractHashMismatch);
    }

    SECTION("Missing evidence in trace") {
        EvidenceStore empty_store;

        auto result = verifier.Verify(cert, empty_store, contract, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(result));
        const auto& rej = std::get<VerificationRejection>(result);
        REQUIRE(rej.failures[0].reason == VerificationFailureReason::kDecisionReconstructionFailed);
    }

    SECTION("Evidence sequence mismatch (evidence changed in trace)") {
        EvidenceStore altered_store;
        // Insert frame with different sequence number (seq 11 instead of 10)
        altered_store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 11, 1000));
        altered_store.InsertFrame(CreateTestFrame("uav-2", "sess-2", 20, 1005));

        auto result = verifier.Verify(cert, altered_store, contract, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(result));
        const auto& rej = std::get<VerificationRejection>(result);
        auto it = std::find_if(rej.failures.begin(), rej.failures.end(), [](const VerificationFailure& f) {
            return f.reason == VerificationFailureReason::kEvidenceSequenceMismatch;
        });
        REQUIRE(it != rej.failures.end());
    }
}
