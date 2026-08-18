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
    snap.contract_id = "sqc-cert-test";
    snap.contract_version = 1;
    snap.contract_hash = "abc123contracthash";
    snap.model_id = "ball-enclosure-v1";
    snap.model_version = "1.0";
    snap.accepted_agents = {"uav-1", "uav-2"};
    snap.produced_at_ms = 1050;

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

}  // namespace

TEST_CASE("StateAcceptanceCertificate construction and hash integrity", "[certificate]") {
    auto snapshot = CreateMockSnapshot();
    auto contract = CreateMockContract();

    auto cert = BuildCertificate(snapshot, contract);

    SECTION("Certificate fields populated accurately") {
        REQUIRE(cert.certificate_id == "cert-snap-test-001");
        REQUIRE(cert.contract_id == "sqc-cert-test");
        REQUIRE(cert.contract_schema_version == 1);
        REQUIRE(cert.contract_content_version == 1);
        REQUIRE(cert.contract_hash == "abc123contracthash");
        REQUIRE_THAT(cert.evaluation_time_ms, WithinAbs(1000.0, 1e-9));

        REQUIRE(cert.evidence_entries.size() == 2);
        // Sorted deterministically: uav-1 first, uav-2 second
        REQUIRE(cert.evidence_entries[0].agent_id == "uav-1");
        REQUIRE(cert.evidence_entries[1].agent_id == "uav-2");

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
    auto cert = BuildCertificate(snapshot, contract);

    REQUIRE(VerifyCertificateIntegrity(cert));

    SECTION("Tamper evaluation time") {
        auto mutated = cert;
        mutated.evaluation_time_ms += 1.0;
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

    SECTION("Tamper certificate hash itself") {
        auto mutated = cert;
        mutated.certificate_hash[0] = (mutated.certificate_hash[0] == 'a') ? 'b' : 'a';
        REQUIRE_FALSE(VerifyCertificateIntegrity(mutated));
    }
}
