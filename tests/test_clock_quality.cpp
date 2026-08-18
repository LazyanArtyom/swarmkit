// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "swarmkit/core/clock_quality.h"

using namespace swarmkit::core;
using Catch::Matchers::WithinAbs;

TEST_CASE("GenerationTimeInterval arithmetic and causality", "[clock_quality]") {
    // Interval [95.0, 105.0] (center 100, radius 5)
    GenerationTimeInterval interval{
        .lower_ms = 95.0,
        .upper_ms = 105.0,
    };

    SECTION("Width calculation") {
        REQUIRE_THAT(interval.Width(), WithinAbs(10.0, 1e-9));
    }

    SECTION("Causality check against evaluation time t*") {
        // At t* = 110.0, g^+ = 105.0 <= 110.0 -> causal
        REQUIRE(interval.IsCausal(110.0));
        // At t* = 105.0, g^+ = 105.0 <= 105.0 -> causal (boundary)
        REQUIRE(interval.IsCausal(105.0));
        // At t* = 104.9, g^+ = 105.0 > 104.9 -> non-causal
        REQUIRE_FALSE(interval.IsCausal(104.9));
        // At t* = 100.0, g^+ = 105.0 > 100.0 -> non-causal
        REQUIRE_FALSE(interval.IsCausal(100.0));
    }

    SECTION("Conservative elapsed time Delta^+ = t* - g^-") {
        // At t* = 120.0, Delta^+ = 120.0 - 95.0 = 25.0 ms
        REQUIRE_THAT(interval.ConservativeElapsed(120.0), WithinAbs(25.0, 1e-9));
        // At t* = 105.0, Delta^+ = 105.0 - 95.0 = 10.0 ms
        REQUIRE_THAT(interval.ConservativeElapsed(105.0), WithinAbs(10.0, 1e-9));
    }
}

TEST_CASE("ClockQualityState interval derivation", "[clock_quality]") {
    ClockQualityState clock_state{
        .offset_estimate_ms = 20.0,          // source is 20ms ahead of reference
        .uncertainty_radius_ms = 5.0,         // bound rho = 5ms
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kEstimated,
        .last_update_ms = 1000,
        .deterministic_bound = true,
    };

    REQUIRE(clock_state.IsValid());

    SECTION("Derives [s - theta - rho, s - theta + rho]") {
        // Source timestamp s = 1000.0 ms
        // ref_time = 1000.0 - 20.0 = 980.0 ms
        // lower = 980.0 - 5.0 = 975.0 ms
        // upper = 980.0 + 5.0 = 985.0 ms
        auto interval = clock_state.ComputeGenerationInterval(1000.0);
        REQUIRE_THAT(interval.lower_ms, WithinAbs(975.0, 1e-9));
        REQUIRE_THAT(interval.upper_ms, WithinAbs(985.0, 1e-9));
        REQUIRE_THAT(interval.Width(), WithinAbs(10.0, 1e-9));
    }

    SECTION("Negative offset (source is behind reference)") {
        ClockQualityState behind_state = clock_state;
        behind_state.offset_estimate_ms = -15.0;  // source is 15ms behind

        // ref_time = 1000.0 - (-15.0) = 1015.0 ms
        // lower = 1015.0 - 5.0 = 1010.0 ms
        // upper = 1015.0 + 5.0 = 1020.0 ms
        auto interval = behind_state.ComputeGenerationInterval(1000.0);
        REQUIRE_THAT(interval.lower_ms, WithinAbs(1010.0, 1e-9));
        REQUIRE_THAT(interval.upper_ms, WithinAbs(1020.0, 1e-9));
    }

    SECTION("Validation rejects uninitialized or invalid states") {
        ClockQualityState invalid_state = clock_state;
        invalid_state.synchronization = ClockSynchronization::kUnknown;
        REQUIRE_FALSE(invalid_state.IsValid());

        invalid_state = clock_state;
        invalid_state.source_domain = ClockDomain::kUnknown;
        REQUIRE_FALSE(invalid_state.IsValid());

        invalid_state = clock_state;
        invalid_state.uncertainty_radius_ms = -1.0;
        REQUIRE_FALSE(invalid_state.IsValid());

        invalid_state = clock_state;
        invalid_state.last_update_ms = 0;
        REQUIRE_FALSE(invalid_state.IsValid());
    }
}

TEST_CASE("ComputeGenerationInterval helper functions", "[clock_quality]") {
    ClockQualityState clock_state{
        .offset_estimate_ms = 10.0,
        .uncertainty_radius_ms = 2.0,
        .source_domain = ClockDomain::kUnixEpoch,
        .synchronization = ClockSynchronization::kSynchronized,
        .last_update_ms = 500,
        .deterministic_bound = true,
    };

    SECTION("From TimestampEvidence with valid clock state") {
        TimestampEvidence te{
            .timestamp_ms = 2000,
            .clock_domain = ClockDomain::kUnixEpoch,
            .synchronization = ClockSynchronization::kSynchronized,
        };

        auto opt_interval = ComputeGenerationInterval(te, clock_state);
        REQUIRE(opt_interval.has_value());
        // ref = 2000 - 10 = 1990
        // lower = 1990 - 2 = 1988, upper = 1990 + 2 = 1992
        REQUIRE_THAT(opt_interval->lower_ms, WithinAbs(1988.0, 1e-9));
        REQUIRE_THAT(opt_interval->upper_ms, WithinAbs(1992.0, 1e-9));
    }

    SECTION("From TimestampEvidence missing timestamp_ms returns nullopt") {
        TimestampEvidence te{
            .timestamp_ms = std::nullopt,
        };
        auto opt_interval = ComputeGenerationInterval(te, clock_state);
        REQUIRE_FALSE(opt_interval.has_value());
    }

    SECTION("From per-sample evidence (fallback)") {
        TimestampEvidence te{
            .timestamp_ms = 5000,
            .clock_uncertainty_ms = 4.0,
        };

        auto opt_interval = ComputeGenerationIntervalFromSample(te);
        REQUIRE(opt_interval.has_value());
        REQUIRE_THAT(opt_interval->lower_ms, WithinAbs(4996.0, 1e-9));
        REQUIRE_THAT(opt_interval->upper_ms, WithinAbs(5004.0, 1e-9));
    }

    SECTION("From per-sample evidence without uncertainty") {
        TimestampEvidence te{
            .timestamp_ms = 5000,
            .clock_uncertainty_ms = std::nullopt,
        };

        auto opt_interval = ComputeGenerationIntervalFromSample(te);
        REQUIRE(opt_interval.has_value());
        REQUIRE_THAT(opt_interval->lower_ms, WithinAbs(5000.0, 1e-9));
        REQUIRE_THAT(opt_interval->upper_ms, WithinAbs(5000.0, 1e-9));
    }
}

TEST_CASE("ComputePropagatedUncertainty formula epsilon = e_p + V_max * Delta^+", "[clock_quality]") {
    const double e_p = 0.5;                // observation uncertainty = 0.5 m
    const double v_max = 10.0;             // max speed = 10.0 m/s
    const double elapsed_ms = 200.0;       // conservative elapsed = 200 ms = 0.2 s

    // epsilon = 0.5 + 10.0 * 0.2 = 0.5 + 2.0 = 2.5 m
    double propagated = ComputePropagatedUncertainty(e_p, v_max, elapsed_ms);
    REQUIRE_THAT(propagated, WithinAbs(2.5, 1e-9));

    // Zero elapsed time
    REQUIRE_THAT(ComputePropagatedUncertainty(e_p, v_max, 0.0), WithinAbs(0.5, 1e-9));

    // Zero velocity
    REQUIRE_THAT(ComputePropagatedUncertainty(e_p, 0.0, elapsed_ms), WithinAbs(0.5, 1e-9));
}
