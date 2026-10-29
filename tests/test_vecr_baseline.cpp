#include <catch2/catch_test_macros.hpp>

#include "swarmkit/edge/vecr.h"

TEST_CASE("Research baseline returns to A only for an in-flight deviation", "[vecr][baseline]") {
    using namespace swarmkit::edge;
    EdgeRequest request;
    request.mission_id = "m";
    request.mission_revision = 1;
    request.edge_id = "e";
    request.attempt_id = "a";
    request.drone_id = "d";
    request.route.points = {{0, 0}, {100, 0}};
    request.corridor_half_width_m = 4;
    request.arrival_radius_m = 1;
    request.arrival_max_speed_mps = .5;
    request.max_recovery_attempts = 2;
    request.timeout_ms = 10000;
    request.maximum_capture_age_ms = 1500;
    request.endpoint_evidence_wait_ms = 500;
    EdgeExecutionSupervisor supervisor(request);
    REQUIRE(supervisor.Valid());
    PositionObservation observation{.position = {50, 0},
                                    .monotonic_ms = 1000,
                                    .valid = true,
                                    .safe_to_navigate = true,
                                    .speed_mps = 0,
                                    .horizontal_error_hard_bound_m = 0,
                                    .speed_error_hard_bound_mps = 0,
                                    .altitude_m = 0,
                                    .vertical_error_hard_bound_m = 0,
                                    .battery_percent = 100};
    (void)supervisor.ObservePosition(observation);
    CaptureEvidence capture{.mission_id = "m",
                            .mission_revision = 1,
                            .edge_id = "e",
                            .attempt_id = "a",
                            .capture_id = "c",
                            .calibration_id = "cal",
                            .monotonic_ms = 1000,
                            .guaranteed_full_strip_interval = {0, 40},
                            .capture_succeeded = true,
                            .full_strip_quality_established = true};
    (void)supervisor.ObserveCapture(capture);
    observation.position.north_m = 6;
    observation.monotonic_ms = 1100;
    CHECK_FALSE(supervisor.ObservePosition(observation).pending_reentry);
    observation.monotonic_ms = 1200;
    CHECK_FALSE(supervisor.ObservePosition(observation).pending_reentry);
    observation.monotonic_ms = 1300;
    const auto status = supervisor.ObservePosition(observation);
    REQUIRE(status.pending_reentry);
    CHECK(status.pending_reentry->east_m == 0);
    CHECK(status.survey_covered_m == 40);
    CHECK(status.survey_missing.front().begin_m == 40);
    CHECK(status.phase == Phase::kRecovering);
    CHECK_FALSE(status.at_b_verified);
}
