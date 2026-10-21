// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>

#include "vecr_truth_oracle.h"

using swarmkit::agent::SimulationTruthFrame;
using swarmkit::testsupport::VecrTruthOracle;

TEST_CASE("VECR private raster oracle detects a narrow unseen gap", "[vecr][oracle]") {
    VecrTruthOracle oracle(20, 0.25, 4);
    SimulationTruthFrame truth;
    truth.drone_id = "drone-1";
    truth.up_m = 10;
    truth.simulation_time_ms = 100;
    oracle.ObservePose(truth);
    for (int x = 0; x <= 20; ++x) {
        truth.east_m = x;
        truth.simulation_time_ms += 100;
        oracle.ObservePose(truth);
        oracle.ObserveActualCapture(truth, x != 10, 0.5, 10);
    }
    CHECK_FALSE(oracle.Complete());
    CHECK(oracle.CoveredCells() == 76);
    CHECK(oracle.LongestMissingM() == 1.0);
    CHECK(oracle.DistanceM() == 20.0);
    oracle.ObserveActualCapture(truth, false, 20, 10);
    CHECK_FALSE(oracle.Complete());
    truth.east_m = 10;
    oracle.ObserveActualCapture(truth, true, 0.5, 3.9);
    CHECK_FALSE(oracle.Complete());
    oracle.ObserveActualCapture(truth, true, 0.5, 10);
    CHECK(oracle.Complete());
}
