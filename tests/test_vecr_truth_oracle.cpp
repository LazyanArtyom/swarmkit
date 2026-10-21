// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

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

TEST_CASE("VECR private oracle refuses partial-width and failed acquisitions",
          "[vecr][oracle]") {
    VecrTruthOracle oracle(4, 0.25, 4);
    SimulationTruthFrame truth;
    truth.drone_id = "drone-1";
    truth.up_m = 10;
    truth.east_m = 2;
    truth.north_m = 2;
    truth.simulation_time_ms = 100;
    oracle.ObservePose(truth);
    oracle.ObserveActualCapture(truth, true, 2, 5.9);
    CHECK(oracle.CoveredCells() == 0);
    truth.failed = true;
    oracle.ObserveActualCapture(truth, true, 2, 6);
    CHECK(oracle.CoveredCells() == 0);
    truth.failed = false;
    oracle.ObserveActualCapture(truth, false, 2, 6);
    CHECK(oracle.CoveredCells() == 0);
    oracle.ObserveActualCapture(truth, true, 2, 6);
    CHECK(oracle.Complete());
}

TEST_CASE("VECR private oracle checks true stop and rejects invalid pose order",
          "[vecr][oracle]") {
    VecrTruthOracle oracle(20, 0.25, 4);
    SimulationTruthFrame truth;
    truth.drone_id = "drone-1";
    truth.east_m = 20;
    truth.up_m = 10;
    truth.velocity_east_mps = 1;
    truth.simulation_time_ms = 100;
    oracle.ObservePose(truth);
    CHECK_FALSE(oracle.AtB(2, 0.5, 10));
    truth.simulation_time_ms = 200;
    truth.velocity_east_mps = 0;
    oracle.ObservePose(truth);
    CHECK(oracle.AtB(2, 0.5, 10));
    CHECK_THROWS_AS(oracle.ObservePose(truth), std::invalid_argument);
    truth.simulation_time_ms = 300;
    truth.drone_id = "foreign-drone";
    CHECK_THROWS_AS(oracle.ObservePose(truth), std::invalid_argument);
    truth.drone_id = "drone-1";
    truth.east_m = 15;
    oracle.ObservePose(truth);
    CHECK_FALSE(oracle.AtB(2, 0.5, 10));
}
