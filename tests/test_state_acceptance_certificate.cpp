// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "swarmkit/core/state_acceptance_certificate.h"
#include "swarmkit/core/state_acceptance_engine.h"

using namespace swarmkit::core;
using Catch::Matchers::WithinAbs;

namespace {

AcceptedSnapshot CreateMockSnapshot() {
    AcceptedSnapshot snap;
    snap.snapshot_id = "snap-test-001";
    snap.evaluation_time_ms = 1000.0;
    snap.evidence_freeze_ms = 1000;
    snap.contract_id = "sqc-cert-test";
    snap.contract_version = 1;
    snap.contract_hash = "abc123contracthash";
    snap.model_id = "ball-enclosure-v1";
    snap.model_version = "1.0";
    snap.accepted_agents = {"uav-1", "uav-2"};
    snap.produced_at_ms = 1050;
    snap.participants = {
        .agent_ids = {"uav-1", "uav-2"},
        .membership_revision = 1,
    };

    // uav-1 position
    snap.agent_states["uav-1"][static_cast<std::uint8_t>(EvidenceFieldId::kPosition)] = {
        .evidence = EvidenceRecord{
            .value = std::array<double, 3>{37.0, -122.0, 10.0},
            .source_time = {.timestamp_ms = 900},
            .receive_time_ms = 920,
            .quality = {},
            .identity = {
                .agent_id = "uav-1",
                .agent_session_id = "sess-1",
                .field_id = EvidenceFieldId::kPosition,
                .sequence = 10,
                .source_component = "ekf",
            },
        },
        .generation_interval = {.lower_ms = 898.0, .upper_ms = 902.0},
        .conservative_elapsed_ms = 102.0,
        .observation_uncertainty = 0.2,
        .propagated_uncertainty = 1.22,
        .clock_uncertainty_ms = 2.0,
    };

    // uav-2 position
    snap.agent_states["uav-2"][static_cast<std::uint8_t>(EvidenceFieldId::kPosition)] = {
        .evidence = EvidenceRecord{
            .value = std::array<double, 3>{37.1, -122.1, 12.0},
            .source_time = {.timestamp_ms = 910},
            .receive_time_ms = 930,
            .quality = {},
            .identity = {
                .agent_id = "uav-2",
                .agent_session_id = "sess-2",
                .field_id = EvidenceFieldId::kPosition,
                .sequence = 20,
                .source_component = "ekf",
            },
        },
        .generation_interval = {.lower_ms = 907.0, .upper_ms = 913.0},
        .conservative_elapsed_ms = 93.0,
        .observation_uncertainty = 0.3,
        .propagated_uncertainty = 1.23,
        .clock_uncertainty_ms = 3.0,
    };

    return snap;
}

StateQualityContract CreateMockContract() {
    return StateQualityContract{
        .contract_id = "sqc-cert-test",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition},
        .required_agents = {"uav-1", "uav-2"},
        .propagation_model_id = "ball-enclosure-v1",
        .propagation_model_version = "1.0",
        .max_horizontal_speed_mps = 10.0F,
        .max_vertical_speed_mps = 5.0F,
    };
}

SnapshotRequestContext CreateMockReqCtx() {
    return SnapshotRequestContext{
        .evaluation_time_ms = 1000.0,
        .evidence_freeze_ms = 1000,
        .participants = {
            .agent_ids = {"uav-1", "uav-2"},
            .membership_revision = 1,
        },
    };
}

}  // namespace

TEST_CASE("StateAcceptanceCertificate construction and hash integrity", "[certificate]") {
    auto snapshot = CreateMockSnapshot();
    auto contract = CreateMockContract();
    auto req_ctx = CreateMockReqCtx();

    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    SECTION("Certificate fields populated accurately") {
        REQUIRE(cert.certificate_id == "cert-snap-test-001");
        REQUIRE(cert.certificate_schema_version == "CERT_V3");
        REQUIRE(cert.contract_id == "sqc-cert-test");
        REQUIRE(cert.contract_schema_version == 1);
        REQUIRE(cert.contract_content_version == 1);
        REQUIRE(cert.contract_hash == "abc123contracthash");
        REQUIRE_THAT(cert.evaluation_time_ms, WithinAbs(1000.0, 1e-9));
        REQUIRE(cert.evidence_freeze_ms == 1000);

        REQUIRE(cert.evidence_entries.size() == 2);
        // Sorted deterministically: uav-1 first, uav-2 second
        REQUIRE(cert.evidence_entries[0].agent_id == "uav-1");
        REQUIRE(cert.evidence_entries[1].agent_id == "uav-2");
        REQUIRE(cert.evidence_entries[0].receive_time_ms == 920);
        REQUIRE(cert.evidence_entries[1].receive_time_ms == 930);

        REQUIRE_THAT(cert.max_clock_uncertainty_ms, WithinAbs(3.0, 1e-9));
        REQUIRE_THAT(cert.max_conservative_elapsed_ms, WithinAbs(102.0, 1e-9));
        REQUIRE_THAT(cert.max_propagated_position_uncertainty_m, WithinAbs(1.23, 1e-9));

        REQUIRE_FALSE(cert.certificate_hash.empty());
        REQUIRE(cert.certificate_hash.size() == 64);
    }

    SECTION("Integrity verification succeeds on untampered certificate") {
        REQUIRE(VerifyCertificateIntegrity(cert));
    }
}

TEST_CASE("StateAcceptanceCertificate tamper detection", "[certificate]") {
    auto snapshot = CreateMockSnapshot();
    auto contract = CreateMockContract();
    auto req_ctx = CreateMockReqCtx();
    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    REQUIRE(VerifyCertificateIntegrity(cert));

    SECTION("Tamper evaluation time") {
        auto mutated = cert;
        mutated.evaluation_time_ms += 1.0;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper evidence_freeze_ms r*") {
        auto mutated = cert;
        mutated.evidence_freeze_ms += 100;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper contract hash") {
        auto mutated = cert;
        mutated.contract_hash = "tampered_hash";
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper evidence sequence") {
        auto mutated = cert;
        mutated.evidence_entries[0].sequence += 1;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper evidence receive_time_ms") {
        auto mutated = cert;
        mutated.evidence_entries[0].receive_time_ms += 50;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper evidence uncertainty") {
        auto mutated = cert;
        mutated.evidence_entries[0].propagated_uncertainty += 0.5;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper generation interval") {
        auto mutated = cert;
        mutated.evidence_entries[0].generation_interval.lower_ms -= 10.0;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper speed bound") {
        auto mutated = cert;
        mutated.max_horizontal_speed_mps = 20.0F;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper accepted agents list") {
        auto mutated = cert;
        mutated.accepted_agents.push_back("uav-rogue");
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper participants snapshot") {
        auto mutated = cert;
        mutated.participants.membership_revision += 1;
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }

    SECTION("Tamper certificate hash itself") {
        auto mutated = cert;
        mutated.certificate_hash[0] = (mutated.certificate_hash[0] == 'a') ? 'b' : 'a';
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }
}

TEST_CASE("StateAcceptanceCertificate canonical serialization roundtrip (P1.2)", "[certificate]") {
    auto snapshot = CreateMockSnapshot();
    auto contract = CreateMockContract();
    auto req_ctx = CreateMockReqCtx();
    auto cert = BuildCertificate(snapshot, contract, req_ctx);

    const std::string serialized = SerializeCertificate(cert);
    REQUIRE_FALSE(serialized.empty());
    REQUIRE(serialized.size() > 100);

    auto deserialized = DeserializeCertificate(serialized);
    REQUIRE(deserialized.has_value());
    REQUIRE(deserialized->certificate_id == cert.certificate_id);
    REQUIRE(deserialized->contract_hash == cert.contract_hash);
    REQUIRE(deserialized->certificate_hash == cert.certificate_hash);
    REQUIRE(deserialized->evidence_entries.size() == cert.evidence_entries.size());
    REQUIRE(deserialized->evidence_freeze_ms == cert.evidence_freeze_ms);
    REQUIRE(deserialized->participants.membership_revision == cert.participants.membership_revision);
    REQUIRE(VerifyCertificateIntegrity(*deserialized));

    // Byte-identical serialization roundtrip
    const std::string reserialized = SerializeCertificate(*deserialized);
    REQUIRE(reserialized == serialized);
}

TEST_CASE("StateAcceptanceCertificate precision with large timestamps and small deltas", "[certificate]") {
    auto snapshot = CreateMockSnapshot();
    snapshot.evaluation_time_ms = 1'700'000'000'000.12345;
    auto contract = CreateMockContract();
    auto req_ctx1 = CreateMockReqCtx();
    req_ctx1.evaluation_time_ms = snapshot.evaluation_time_ms;
    auto cert1 = BuildCertificate(snapshot, contract, req_ctx1);

    snapshot.evaluation_time_ms = 1'700'000'000'001.12345;  // 1 ms delta at large timestamp
    auto req_ctx2 = CreateMockReqCtx();
    req_ctx2.evaluation_time_ms = snapshot.evaluation_time_ms;
    auto cert2 = BuildCertificate(snapshot, contract, req_ctx2);

    REQUIRE(cert1.certificate_hash != cert2.certificate_hash);

    const std::string ser1 = SerializeCertificate(cert1);
    const std::string ser2 = SerializeCertificate(cert2);
    REQUIRE(ser1 != ser2);

    auto deser1 = DeserializeCertificate(ser1);
    auto deser2 = DeserializeCertificate(ser2);
    REQUIRE(deser1.has_value());
    REQUIRE(deser2.has_value());
    REQUIRE_THAT(deser1->evaluation_time_ms, WithinAbs(1'700'000'000'000.12345, 1e-6));
    REQUIRE_THAT(deser2->evaluation_time_ms, WithinAbs(1'700'000'000'001.12345, 1e-6));
}

TEST_CASE("ComputeEvidenceHash is sensitive to every decision-relevant field", "[certificate][evidence_hash]") {
    EvidenceRecord base{
        .value = std::array<double, 3>{37.7749, -122.4194, 10.0},
        .source_time = {
            .timestamp_ms = 1'700'000'000'000LL,
            .clock_domain = ClockDomain::kUnixEpoch,
            .synchronization = ClockSynchronization::kSynchronized,
            .clock_uncertainty_ms = 2.0,
        },
        .receive_time_ms = 1'700'000'000'020LL,
        .quality = {
            .uncertainty = UncertaintyEstimate{
                .value = 0.25F,
                .descriptor = {.semantics = UncertaintySemantics::kDeterministicHardBound},
            },
            .estimator_healthy = true,
            .estimator_position_ok = true,
            .estimator_velocity_ok = true,
        },
        .identity = {
            .agent_id = "uav-1",
            .agent_session_id = "sess-1",
            .field_id = EvidenceFieldId::kPosition,
            .sequence = 100,
            .coordinate_frame = CoordinateFrame::kWgs84,
            .source_component = "ekf",
            .estimator_id = "ekf3",
            .uncertainty_kind = UncertaintySemantics::kDeterministicHardBound,
            .mission_id = "mission-alpha",
            .mission_revision = 1,
        },
    };

    const std::string base_hash = ComputeEvidenceHash(base);
    REQUIRE_FALSE(base_hash.empty());

    // Agent ID mutation
    auto m_agent = base;
    m_agent.identity.agent_id = "uav-2";
    REQUIRE(ComputeEvidenceHash(m_agent) != base_hash);

    // Session ID mutation
    auto m_sess = base;
    m_sess.identity.agent_session_id = "sess-2";
    REQUIRE(ComputeEvidenceHash(m_sess) != base_hash);

    // Sequence mutation
    auto m_seq = base;
    m_seq.identity.sequence = 101;
    REQUIRE(ComputeEvidenceHash(m_seq) != base_hash);

    // Timestamp mutation
    auto m_ts = base;
    *m_ts.source_time.timestamp_ms += 1;
    REQUIRE(ComputeEvidenceHash(m_ts) != base_hash);

    // Timestamp absent mutation
    auto m_ts_abs = base;
    m_ts_abs.source_time.timestamp_ms = std::nullopt;
    REQUIRE(ComputeEvidenceHash(m_ts_abs) != base_hash);

    // Clock uncertainty mutation
    auto m_clk_unc = base;
    *m_clk_unc.source_time.clock_uncertainty_ms = 5.0;
    REQUIRE(ComputeEvidenceHash(m_clk_unc) != base_hash);

    // Receive time mutation
    auto m_rx = base;
    m_rx.receive_time_ms += 50;
    REQUIRE(ComputeEvidenceHash(m_rx) != base_hash);

    // Coordinate frame mutation
    auto m_frame = base;
    m_frame.identity.coordinate_frame = CoordinateFrame::kLocalNed;
    REQUIRE(ComputeEvidenceHash(m_frame) != base_hash);

    // Estimator health mutation
    auto m_health = base;
    m_health.quality.estimator_healthy = false;
    REQUIRE(ComputeEvidenceHash(m_health) != base_hash);

    // Observation uncertainty value mutation
    auto m_unc_val = base;
    m_unc_val.quality.uncertainty->value = 0.5F;
    REQUIRE(ComputeEvidenceHash(m_unc_val) != base_hash);

    // Observation uncertainty semantics mutation
    auto m_unc_sem = base;
    m_unc_sem.quality.uncertainty->descriptor.semantics = UncertaintySemantics::kStandardDeviation;
    REQUIRE(ComputeEvidenceHash(m_unc_sem) != base_hash);

    // Value mutation
    auto m_val = base;
    m_val.value = std::array<double, 3>{37.7750, -122.4194, 10.0};
    REQUIRE(ComputeEvidenceHash(m_val) != base_hash);
}
