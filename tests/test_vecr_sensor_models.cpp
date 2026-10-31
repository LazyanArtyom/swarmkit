// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>
#include <random>

#include "vecr_sensor_models.h"

TEST_CASE("Independent full-strip truth preserves subcell holes and rotated footprints",
          "[vecr][sensor-truth]") {
    swarmkit::testsupport::sensing::ExactStripOracle oracle(20);
    oracle.Add(std::pair{0.0, 10.01});
    oracle.Add(std::pair{10.02, 20.0});
    CHECK_FALSE(oracle.Complete());
    CHECK(oracle.RasterFraction(0.25, true) == 1.0);
    CHECK(oracle.RasterFraction(0.25, false) < 1.0);
    CHECK_FALSE(oracle.Footprint(10, 0, 2, 7, 45));
    REQUIRE(oracle.Footprint(10, 5, 2, 14, 0));
    CHECK_FALSE(oracle.Footprint(10, 5, 2, 7, 0));
}

TEST_CASE("Bounded camera intervals are contained in independent actual footprints",
          "[vecr][sensor-truth]") {
    using namespace swarmkit;
    std::mt19937_64 random(20261024);
    std::uniform_real_distribution<double> uniform(0, 1);
    const edge::Route route{{{0, 0}, {40, 0}}};
    testsupport::sensing::ExactStripOracle oracle(40);
    int proved = 0;
    for (int i = 0; i < 3000; ++i) {
        const double x = uniform(random) * 40, y = (uniform(random) - 0.5) * 16;
        const double altitude = 8 + 4 * uniform(random), bound = uniform(random);
        const double angle = uniform(random) * 2 * std::numbers::pi;
        const double along = 1.2 + uniform(random) * 3, cross = 5 + uniform(random) * 10;
        const double ve = 0.1 * uniform(random);
        const auto geo =
            edge::LocalToWgs84(40, 44, {x + bound * std::sin(angle), y + bound * std::cos(angle)});
        core::TelemetryFrame frame;
        frame.lat_deg = geo.lat_deg;
        frame.lon_deg = geo.lon_deg;
        frame.rel_alt_m = static_cast<float>(altitude + ve);
        frame.validity.position = true;
        frame.validity.relative_altitude = true;
        frame.validity.estimator = true;
        frame.estimator_state = core::EstimatorState::kHealthy;
        frame.position_frame = core::CoordinateFrame::kWgs84;
        core::UncertaintyDescriptor descriptor{
            .semantics = core::UncertaintySemantics::kDeterministicHardBound};
        frame.accuracy.horizontal_position = core::UncertaintyEstimate{
            .value = static_cast<float>(bound + 1e-5), .descriptor = descriptor};
        frame.accuracy.vertical_position =
            core::UncertaintyEstimate{.value = 0.10001F, .descriptor = descriptor};
        testsupport::sensing::CameraEstimator estimator(40, 44, along, cross);
        auto input = estimator.Estimate(frame, 1000, true, true, "cal");
        REQUIRE(input);
        input->mission_id = "m";
        input->mission_revision = 5;
        input->edge_id = "e";
        input->attempt_id = "a";
        input->capture_id = "c";
        const auto evidence = edge::BuildNadirStripEvidence(route, 4, *input);
        if (evidence) {
            ++proved;
            const auto actual =
                oracle.Footprint(x, y, along * altitude / 10, cross * altitude / 10, 0);
            REQUIRE(actual);
            CHECK(evidence->guaranteed_full_strip_interval.begin_m >= actual->first - 1e-6);
            CHECK(evidence->guaranteed_full_strip_interval.end_m <= actual->second + 1e-6);
        }
        frame.accuracy.horizontal_position->descriptor.semantics =
            core::UncertaintySemantics::kStandardDeviation;
        CHECK_FALSE(estimator.Estimate(frame, 1000, true, true, "cal"));
        frame.accuracy.horizontal_position.reset();
        CHECK_FALSE(estimator.Estimate(frame, 1000, true, true, "cal"));
    }
    CHECK(proved > 500);
}
