// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include "swarmkit/core/state_quality_contract.h"

using namespace swarmkit::core;

TEST_CASE("StateQualityContract hash computation is deterministic and sensitive", "[contract]") {
    StateQualityContract c1{
        .contract_id = "sqc-formation-flight",
        .schema_version = 1,
        .content_version = 1,
        .required_fields = {EvidenceFieldId::kPosition, EvidenceFieldId::kVelocity},
        .max_evidence_age_ms = 250.0,
        .max_clock_uncertainty_ms = 5.0,
        .max_position_uncertainty_m = 1.0,
        .max_velocity_uncertainty_mps = 0.5,
        .require_estimator_position_ok = true,
        .require_estimator_velocity_ok = true,
        .require_estimator_healthy = true,
        .min_gps_quality = GpsQuality::kRtkFixed,
        .required_position_frame = CoordinateFrame::kWgs84,
        .required_velocity_frame = CoordinateFrame::kLocalNed,
        .require_current_epoch = true,
        .require_current_mission = true,
        .required_mission_id = "mission-alpha",
        .required_mission_revision = 42,
        .required_agents = {"uav-1", "uav-2", "uav-3"},
        .completeness = CompletenessRule::kAllRequired,
        .min_required_agents = 3,
        .require_deterministic_bounds = true,
        .propagation_model_id = "ball-enclosure-v1",
        .propagation_model_version = "1.0",
        .max_horizontal_speed_mps = 12.0F,
        .max_vertical_speed_mps = 4.0F,
    };

    SECTION("Determinism across calls") {
        const std::string h1 = ComputeContractHash(c1);
        const std::string h2 = ComputeContractHash(c1);
        REQUIRE(h1 == h2);
        REQUIRE(h1.size() == 64);  // SHA-256 hex string length
    }

    SECTION("Required agents sorting determinism") {
        StateQualityContract c2 = c1;
        // Permute required_agents insertion order in the unordered_set
        c2.required_agents = {"uav-3", "uav-1", "uav-2"};
        REQUIRE(ComputeContractHash(c1) == ComputeContractHash(c2));
    }

    SECTION("Sensitivity to contract_id") {
        StateQualityContract c2 = c1;
        c2.contract_id = "sqc-formation-flight-v2";
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to version") {
        StateQualityContract c2 = c1;
        c2.content_version = 2;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to required fields") {
        StateQualityContract c2 = c1;
        c2.required_fields.push_back(EvidenceFieldId::kBattery);
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to age limit") {
        StateQualityContract c2 = c1;
        c2.max_evidence_age_ms = 300.0;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to clock uncertainty threshold") {
        StateQualityContract c2 = c1;
        c2.max_clock_uncertainty_ms = 10.0;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to position uncertainty threshold") {
        StateQualityContract c2 = c1;
        c2.max_position_uncertainty_m = 2.0;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to health flags") {
        StateQualityContract c2 = c1;
        c2.require_estimator_healthy = false;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to GPS quality requirement") {
        StateQualityContract c2 = c1;
        c2.min_gps_quality = GpsQuality::kFix3D;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to coordinate frames") {
        StateQualityContract c2 = c1;
        c2.required_position_frame = CoordinateFrame::kLocalNed;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to mission requirements") {
        StateQualityContract c2 = c1;
        c2.required_mission_revision = 43;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to required agents") {
        StateQualityContract c2 = c1;
        c2.required_agents = {"uav-1", "uav-2"};
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to completeness rule") {
        StateQualityContract c2 = c1;
        c2.completeness = CompletenessRule::kMinimumCount;
        c2.min_required_agents = 2;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to speed limit bounds") {
        StateQualityContract c2 = c1;
        c2.max_horizontal_speed_mps = 15.0F;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }

    SECTION("Sensitivity to deterministic bounds requirement") {
        StateQualityContract c2 = c1;
        c2.require_deterministic_bounds = false;
        REQUIRE(ComputeContractHash(c1) != ComputeContractHash(c2));
    }
}

TEST_CASE("ValidateStateQualityContract rejects malformed contract parameters", "[contract]") {
    StateQualityContract valid{
        .contract_id = "sqc-valid",
        .required_fields = {EvidenceFieldId::kPosition},
        .max_evidence_age_ms = 500.0,
        .max_clock_uncertainty_ms = 10.0,
        .max_position_uncertainty_m = 2.0,
        .required_agents = {"uav-1", "uav-2"},
        .completeness = CompletenessRule::kAllRequired,
        .propagation_model_id = "ball-enclosure-v1",
        .max_horizontal_speed_mps = 10.0F,
        .max_vertical_speed_mps = 5.0F,
    };

    REQUIRE(ValidateStateQualityContract(valid).IsOk());

    SECTION("Empty contract_id") {
        auto invalid = valid;
        invalid.contract_id = "";
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());
    }

    SECTION("Empty required_fields") {
        auto invalid = valid;
        invalid.required_fields.clear();
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());
    }

    SECTION("Empty required_agents") {
        auto invalid = valid;
        invalid.required_agents.clear();
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());
    }

    SECTION("Invalid min_required_agents") {
        auto invalid = valid;
        invalid.completeness = CompletenessRule::kMinimumCount;
        invalid.min_required_agents = 0;
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());

        invalid.min_required_agents = 5;  // Greater than required_agents.size() == 2
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());

        invalid.min_required_agents = 2;  // Valid
        REQUIRE(ValidateStateQualityContract(invalid).IsOk());
    }

    SECTION("Negative or NaN thresholds") {
        auto invalid = valid;
        invalid.max_evidence_age_ms = -10.0;
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());

        invalid = valid;
        invalid.max_evidence_age_ms = std::numeric_limits<double>::quiet_NaN();
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());

        invalid = valid;
        invalid.max_clock_uncertainty_ms = -1.0;
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());

        invalid = valid;
        invalid.max_position_uncertainty_m = -0.5;
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());

        invalid = valid;
        invalid.max_horizontal_speed_mps = -1.0F;
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());
    }

    SECTION("Empty propagation_model_id") {
        auto invalid = valid;
        invalid.propagation_model_id = "";
        REQUIRE_FALSE(ValidateStateQualityContract(invalid).IsOk());
    }
}
