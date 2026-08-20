// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>

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

std::unordered_map<std::string, ClockQualityState> CreateStandardClockStates() {
    std::unordered_map<std::string, ClockQualityState> map;
    map["uav-1"] = ClockQualityState{
        .offset_estimate_ms = 0.0,
        .uncertainty_radius_ms = 2.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 500,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-1",
    };
    map["uav-2"] = ClockQualityState{
        .offset_estimate_ms = 0.0,
        .uncertainty_radius_ms = 2.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 500,
        .deterministic_bound = true,
        .agent_incarnation_id = "sess-2",
    };
    return map;
}

SnapshotRequestContext CreateTestReqCtx(double t_star = 1100.0, std::int64_t r_star = 1100) {
    return SnapshotRequestContext{
        .evaluation_time_ms = t_star,
        .evidence_freeze_ms = r_star,
        .participants = {
            .agent_ids = {"uav-1", "uav-2"},
            .membership_revision = 1,
        },
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
    auto clock_states = CreateStandardClockStates();

    const auto req_ctx = CreateTestReqCtx(1100.0, 1100);
    auto engine_result = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(engine_result));

    const auto& snapshot = std::get<AcceptedSnapshot>(engine_result);
    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    StateAcceptanceVerifier verifier;
    auto verify_result = verifier.Verify(cert, store, contract, req_ctx, clock_states);

    SECTION("Verifier accepts valid certificate and reconstructed snapshot matches") {
        REQUIRE(std::holds_alternative<VerifiedAcceptance>(verify_result));
        const auto& accepted = std::get<VerifiedAcceptance>(verify_result);
        REQUIRE(accepted.certificate_id == cert.certificate_id);
        REQUIRE(accepted.reconstructed_snapshot.accepted_agents == snapshot.accepted_agents);
        REQUIRE(accepted.reconstructed_snapshot.contract_hash == snapshot.contract_hash);
    }
}

TEST_CASE("StateAcceptanceVerifier rejection on certificate tampering (Expanded Matrix, P1)", "[verifier]") {
    StateAcceptanceEngine engine;
    EvidenceStore store;

    store.InsertFrame(CreateTestFrame("uav-1", "sess-1", 10, 1000));
    store.InsertFrame(CreateTestFrame("uav-2", "sess-2", 20, 1005));

    auto contract = CreateTestContract();
    auto clock_states = CreateStandardClockStates();

    const auto req_ctx = CreateTestReqCtx(1100.0, 1100);
    auto engine_result = engine.RequestSnapshot(contract, req_ctx, store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(engine_result));
    const auto& snapshot = std::get<AcceptedSnapshot>(engine_result);
    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    StateAcceptanceVerifier verifier;

    SECTION("1. Tampered certificate hash") {
        auto tampered = cert;
        tampered.certificate_hash = "0000000000000000000000000000000000000000000000000000000000000000";
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("2. Tampered evaluation time") {
        auto tampered = cert;
        tampered.evaluation_time_ms += 50.0;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("3. Tampered contract hash") {
        auto tampered = cert;
        tampered.contract_hash = "1111222233334444555566667777888899990000aaaabbbbccccddddeeeeffff";
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("4. Tampered contract version") {
        auto tampered = cert;
        tampered.contract_content_version += 1;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("5. Tampered semantic version") {
        auto tampered = cert;
        tampered.acceptance_semantics_version = "3.0";
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("6. Tampered evidence sequence") {
        auto tampered = cert;
        tampered.evidence_entries[0].sequence += 99;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("7. Tampered evidence content hash") {
        auto tampered = cert;
        tampered.evidence_entries[0].evidence_hash = "badhash000000000000000000000000000000000000000000000000000000000";
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("8. Tampered agent session") {
        auto tampered = cert;
        tampered.evidence_entries[0].agent_session_id = "stale-session";
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("9. Tampered propagated bound") {
        auto tampered = cert;
        tampered.evidence_entries[0].propagated_uncertainty = 0.001;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("10. Tampered clock bound rho") {
        auto tampered = cert;
        tampered.evidence_entries[0].base_rho_ms = 0.001;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("11. Tampered accepted agents list") {
        auto tampered = cert;
        tampered.accepted_agents.push_back("uav-rogue");
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("12. Tampered source timestamp") {
        auto tampered = cert;
        tampered.evidence_entries[0].source_time_ms =
            tampered.evidence_entries[0].source_time_ms.value_or(0) + 1000;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("13. Tampered source component provenance") {
        auto tampered = cert;
        tampered.evidence_entries[0].source_component = "compromised-sensor";
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("14. Tampered coordinate frame") {
        auto tampered = cert;
        tampered.evidence_entries[0].coordinate_frame = CoordinateFrame::kLocalNed;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    SECTION("15. Tampered observation uncertainty") {
        auto tampered = cert;
        tampered.evidence_entries[0].observation_uncertainty = 0.001;
        tampered.certificate_hash = ComputeCertificateHash(tampered);
        auto res = verifier.Verify(tampered, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(res));
    }

    const auto require_semantic_rejection = [&](StateAcceptanceCertificate mutated) {
        mutated.certificate_hash = ComputeCertificateHash(mutated);
        const auto result = verifier.Verify(mutated, store, contract, req_ctx, clock_states);
        REQUIRE(std::holds_alternative<VerificationRejection>(result));
    };

    SECTION("16. Rehashed r-star mutation") {
        auto mutated = cert; ++mutated.evidence_freeze_ms; require_semantic_rejection(mutated);
    }
    SECTION("17. Rehashed participant-set mutation") {
        auto mutated = cert; mutated.participants.agent_ids.push_back("uav-3");
        require_semantic_rejection(mutated);
    }
    SECTION("18. Rehashed membership-revision mutation") {
        auto mutated = cert; ++mutated.participants.membership_revision;
        require_semantic_rejection(mutated);
    }
    SECTION("19. Rehashed theta-hat mutation") {
        auto mutated = cert; mutated.evidence_entries[0].theta_hat_ms += 0.5;
        require_semantic_rejection(mutated);
    }
    SECTION("20. Rehashed effective-rho mutation") {
        auto mutated = cert; mutated.evidence_entries[0].effective_rho_ms += 0.5;
        require_semantic_rejection(mutated);
    }
    SECTION("21. Rehashed drift-rate mutation") {
        auto mutated = cert; mutated.evidence_entries[0].max_drift_rate_ppm += 1.0;
        require_semantic_rejection(mutated);
    }
    SECTION("22. Rehashed clock-update-time mutation") {
        auto mutated = cert; ++mutated.evidence_entries[0].clock_model_last_update_reference_ms;
        require_semantic_rejection(mutated);
    }
    SECTION("23. Rehashed clock-model-version mutation") {
        auto mutated = cert; mutated.evidence_entries[0].clock_model_version = "clock-v999";
        require_semantic_rejection(mutated);
    }
    SECTION("24. Rehashed clock-incarnation mutation") {
        auto mutated = cert; mutated.evidence_entries[0].agent_incarnation_id = "wrong-incarnation";
        require_semantic_rejection(mutated);
    }
    SECTION("25. Rehashed g-minus mutation") {
        auto mutated = cert; mutated.evidence_entries[0].generation_interval.lower_ms -= 0.5;
        require_semantic_rejection(mutated);
    }
    SECTION("26. Rehashed g-plus mutation") {
        auto mutated = cert; mutated.evidence_entries[0].generation_interval.upper_ms += 0.5;
        require_semantic_rejection(mutated);
    }
    SECTION("27. Rehashed Delta-plus mutation") {
        auto mutated = cert; mutated.evidence_entries[0].conservative_elapsed_ms += 0.5;
        require_semantic_rejection(mutated);
    }
    SECTION("28. Rehashed propagation-model-id mutation") {
        auto mutated = cert; mutated.propagation_model_id = "wrong-model";
        require_semantic_rejection(mutated);
    }
    SECTION("29. Rehashed propagation-model-version mutation") {
        auto mutated = cert; mutated.propagation_model_version = "999";
        require_semantic_rejection(mutated);
    }
    SECTION("30. Rehashed horizontal-speed mutation") {
        auto mutated = cert; mutated.max_horizontal_speed_mps += 0.5F;
        require_semantic_rejection(mutated);
    }
    SECTION("31. Rehashed vertical-speed mutation") {
        auto mutated = cert; mutated.max_vertical_speed_mps += 0.5F;
        require_semantic_rejection(mutated);
    }
    SECTION("32. Rehashed certificate-schema mutation") {
        auto mutated = cert; mutated.certificate_schema_version = "CERT_V999";
        require_semantic_rejection(mutated);
    }
    SECTION("33. Rehashed health-predicate mutation") {
        auto mutated = cert; mutated.evidence_entries[0].estimator_position_ok = false;
        require_semantic_rejection(mutated);
    }
    SECTION("34. Rehashed mission-revision mutation") {
        auto mutated = cert; ++mutated.evidence_entries[0].mission_revision;
        require_semantic_rejection(mutated);
    }
    SECTION("35. Rehashed GPS-claim mutation on non-GPS evidence") {
        auto mutated = cert; mutated.evidence_entries[0].gps_quality = GpsQuality::kRtkFixed;
        require_semantic_rejection(mutated);
    }
}

TEST_CASE("test_verifier_gps_predicate", "[verifier][gps]") {
    auto frame = CreateTestFrame("uav-1", "sess-1", 10, 1000);
    frame.validity.gps = true;
    frame.gps_quality = GpsQuality::kRtkFixed;
    frame.provenance.vehicle_state.source_time = frame.provenance.position.source_time;
    frame.provenance.vehicle_state.source = "gps-receiver";
    EvidenceStore store;
    store.InsertFrame(frame);
    auto contract = CreateTestContract();
    contract.required_agents = {"uav-1"};
    contract.min_gps_quality = GpsQuality::kRtkFloat;
    auto clocks = CreateStandardClockStates();
    clocks.erase("uav-2");
    auto context = CreateTestReqCtx();
    context.participants.agent_ids = {"uav-1"};
    const auto decision = StateAcceptanceEngine{}.RequestSnapshot(contract, context, store, clocks);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(decision));
    const auto certificate = BuildCertificate(std::get<AcceptedSnapshot>(decision), contract, context);
    REQUIRE(std::holds_alternative<VerifiedAcceptance>(
        StateAcceptanceVerifier{}.Verify(certificate, store, contract, context, clocks)));

    SECTION("rehashed GPS claim is rejected independently") {
        auto mutated = certificate;
        const auto gps_entry = std::find_if(mutated.evidence_entries.begin(),
            mutated.evidence_entries.end(), [](const auto& entry) {
                return entry.field == EvidenceFieldId::kGpsQuality;
            });
        REQUIRE(gps_entry != mutated.evidence_entries.end());
        gps_entry->gps_quality = GpsQuality::kNoFix;
        mutated.certificate_hash = ComputeCertificateHash(mutated);
        REQUIRE(std::holds_alternative<VerificationRejection>(
            StateAcceptanceVerifier{}.Verify(mutated, store, contract, context, clocks)));
    }

    SECTION("below-minimum GPS evidence cannot produce a certificate") {
        auto below = frame;
        below.gps_quality = GpsQuality::kFix3D;
        EvidenceStore below_store;
        below_store.InsertFrame(below);
        REQUIRE(std::holds_alternative<StructuredRejection>(
            StateAcceptanceEngine{}.RequestSnapshot(contract, context, below_store, clocks)));
    }
}

TEST_CASE("StateAcceptanceVerifier offline deserialization and fresh-store verification", "[verifier]") {
    StateAcceptanceEngine engine;
    EvidenceStore live_store;

    auto frame1 = CreateTestFrame("uav-1", "sess-1", 10, 1000);
    auto frame2 = CreateTestFrame("uav-2", "sess-2", 20, 1005);
    live_store.InsertFrame(frame1);
    live_store.InsertFrame(frame2);

    auto contract = CreateTestContract();
    auto clock_states = CreateStandardClockStates();

    const auto req_ctx = CreateTestReqCtx(1100.0, 1100);
    auto engine_result = engine.RequestSnapshot(contract, req_ctx, live_store, clock_states);
    REQUIRE(std::holds_alternative<AcceptedSnapshot>(engine_result));
    const auto& snapshot = std::get<AcceptedSnapshot>(engine_result);
    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    // Save certificate to wire string
    std::string wire_bytes = SerializeCertificate(cert);
    auto loaded_cert = DeserializeCertificate(wire_bytes);
    REQUIRE(loaded_cert.has_value());

    // Construct a completely fresh, separate EvidenceStore for offline replay
    EvidenceStore fresh_offline_store;
    fresh_offline_store.SetCurrentSession("uav-1", "sess-1");
    fresh_offline_store.SetCurrentSession("uav-2", "sess-2");
    fresh_offline_store.InsertFrame(frame1);
    fresh_offline_store.InsertFrame(frame2);

    auto fresh_clock_states = CreateStandardClockStates();

    StateAcceptanceVerifier offline_verifier;
    auto verify_result = offline_verifier.Verify(*loaded_cert, fresh_offline_store, contract, req_ctx, fresh_clock_states);
    REQUIRE(std::holds_alternative<VerifiedAcceptance>(verify_result));

    // If an observation is missing in fresh offline store, replay MUST fail
    EvidenceStore empty_offline_store;
    auto empty_result = offline_verifier.Verify(*loaded_cert, empty_offline_store, contract, req_ctx, fresh_clock_states);
    REQUIRE(std::holds_alternative<VerificationRejection>(empty_result));
}
