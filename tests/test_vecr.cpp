// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include <catch2/catch_test_macros.hpp>

#include "swarmkit/edge/vecr.h"

namespace {
using namespace swarmkit::edge;  // NOLINT(google-build-using-namespace)

EdgeRequest Request(Obligation obligation = Obligation::kSurvey) {
    EdgeRequest request;
    request.mission_id = "mission";
    request.mission_revision = 2;
    request.edge_id = "A-B";
    request.attempt_id = "attempt-2";
    request.drone_id = "uav-1";
    request.route.points = {{0, 0}, {100, 0}};
    request.obligation = obligation;
    request.corridor_half_width_m = 4;
    request.arrival_radius_m = 1;
    request.arrival_max_speed_mps = 0.5;
    request.maximum_speed_mps = 5;
    request.recovery_overlap_m = 2;
    request.max_recovery_attempts = 2;
    request.sustained_deviation_samples = 3;
    request.recovery_cooldown_ms = 1000;
    request.timeout_ms = 1000000;
    return request;
}

PositionObservation Position(double x, std::int64_t time_ms, double y = 0, bool stopped = false) {
    return {.position = {x, y}, .monotonic_ms = time_ms, .valid = true,
            .safe_to_navigate = true, .speed_mps = stopped ? 0.0 : 1.0,
            .horizontal_error_hard_bound_m = 0.0};
}

CaptureEvidence Capture(double a, double b, std::int64_t time_ms, std::string id) {
    return {.attempt_id = "attempt-2", .capture_id = std::move(id),
            .calibration_id = "calibration", .monotonic_ms = time_ms,
            .guaranteed_full_strip_interval = {a, b}, .capture_succeeded = true,
            .full_strip_quality_established = true};
}
}  // namespace

TEST_CASE("VECR interval union retains holes and later progress", "[vecr]") {
    IntervalSet set(100);
    REQUIRE(set.Add({50, 80}));
    REQUIRE(set.Add({0, 20}));
    REQUIRE(set.Add({15, 30}));
    CHECK(set.Intervals().size() == 2);
    CHECK(set.PrefixMeters() == 30);
    CHECK(set.CoveredMeters() == 60);
    REQUIRE(set.Missing().size() == 2);
    CHECK(set.Missing().front().begin_m == 30);
    CHECK(set.Missing().front().end_m == 50);
    CHECK(set.Missing().back().begin_m == 80);
}

TEST_CASE("VECR arrival does not imply surveyed terrain", "[vecr]") {
    EdgeExecutionSupervisor supervisor(Request());
    REQUIRE(supervisor.Valid());
    static_cast<void>(supervisor.ObservePosition(Position(0, 1000)));
    static_cast<void>(supervisor.ObserveCapture(Capture(0, 40, 1000, "a")));
    static_cast<void>(supervisor.ObserveCapture(Capture(70, 100, 1000, "b")));
    const auto status = supervisor.ObservePosition(Position(100, 2000, 0, true));
    CHECK(status.at_b_verified);
    CHECK_FALSE(status.survey_verified);
    CHECK(status.phase != Phase::kFulfilled);
    REQUIRE(status.pending_reentry.has_value());
    CHECK(status.pending_reentry->east_m == 38);
    CHECK(status.survey_prefix_m == 40);
    REQUIRE(status.survey_missing.size() == 1);
    CHECK(status.survey_missing.front().begin_m == 40);
    CHECK(status.survey_missing.front().end_m == 70);
}

TEST_CASE("VECR retains accepted off-corridor survey and enforces attempt identity", "[vecr]") {
    EdgeExecutionSupervisor supervisor(Request());
    static_cast<void>(supervisor.ObservePosition(Position(0, 1000)));
    static_cast<void>(supervisor.ObservePosition(Position(50, 2000, 8)));
    auto stale = Capture(0, 100, 2000, "stale");
    stale.attempt_id = "previous-attempt";
    CHECK_FALSE(supervisor.ObserveCapture(stale).survey_verified);
    const auto accepted = supervisor.ObserveCapture(Capture(0, 100, 2000, "valid"));
    CHECK(accepted.survey_verified);
    CHECK_FALSE(accepted.at_b_verified);
    CHECK(accepted.phase != Phase::kFulfilled);
    const auto done = supervisor.ObservePosition(Position(100, 3000, 0, true));
    CHECK(done.phase == Phase::kFulfilled);
    CHECK_FALSE(done.traversal_verified);
}

TEST_CASE("VECR ignores duplicate captures and rejects absent camera evidence", "[vecr]") {
    EdgeExecutionSupervisor supervisor(Request());
    static_cast<void>(supervisor.ObservePosition(Position(0, 1000)));
    auto failed = Capture(0, 100, 1000, "failed");
    failed.capture_succeeded = false;
    CHECK_FALSE(supervisor.ObserveCapture(failed).survey_verified);
    auto uncertain = Capture(0, 100, 1000, "uncertain");
    uncertain.full_strip_quality_established = false;
    CHECK_FALSE(supervisor.ObserveCapture(uncertain).survey_verified);
    static_cast<void>(supervisor.ObserveCapture(Capture(0, 20, 1000, "one")));
    static_cast<void>(supervisor.ObserveCapture(Capture(20, 100, 1000, "one")));
    CHECK(supervisor.Snapshot().survey_covered_m == 20);
}

TEST_CASE("VECR sparse samples cannot prove whole interval traversal", "[vecr]") {
    EdgeExecutionSupervisor supervisor(Request(Obligation::kTraversal));
    static_cast<void>(supervisor.ObservePosition(Position(0, 1000)));
    const auto status = supervisor.ObservePosition(Position(100, 11000, 0, true));
    CHECK(status.at_b_verified);
    CHECK_FALSE(status.traversal_verified);
    CHECK(status.phase != Phase::kFulfilled);
}

TEST_CASE("VECR recovery is bounded and failsafe wins", "[vecr]") {
    auto req = Request();
    req.max_recovery_attempts = 1;
    EdgeExecutionSupervisor supervisor(req);
    static_cast<void>(supervisor.ObservePosition(Position(0, 1000)));
    auto proposed = supervisor.ObservePosition(Position(100, 2000, 0, true));
    REQUIRE(proposed.pending_reentry);
    static_cast<void>(supervisor.RecoveryDispatched(true, 2000));
    CHECK(supervisor.Snapshot().recovery_attempts == 1);
    proposed = supervisor.ObservePosition(Position(100, 4000, 0, true));
    CHECK_FALSE(proposed.pending_reentry);
    auto fail = Position(100, 5000, 0, true);
    fail.failsafe = true;
    CHECK(supervisor.ObservePosition(fail).phase == Phase::kInterrupted);
    CHECK(supervisor.ObserveCapture(Capture(0, 100, 5000, "late")).phase ==
          Phase::kInterrupted);
}

TEST_CASE("VECR rejects degenerate route and resolves polyline projection", "[vecr]") {
    auto req = Request();
    req.route.points = {{0, 0}, {0, 0}};
    CHECK_FALSE(EdgeExecutionSupervisor(req).Valid());
    const Route l_shape{{{0, 0}, {10, 0}, {10, 10}}};
    const auto projection = Project(l_shape, {11, 8});
    REQUIRE(projection);
    CHECK(projection->arclength_m == 18);
    CHECK(projection->cross_track_m == 1);
    const GeoPoint geo = LocalToWgs84(40.0, 44.0, {10, 20});
    const Point local = Wgs84ToLocal(40.0, 44.0, geo.lat_deg, geo.lon_deg);
    CHECK(std::abs(local.east_m - 10.0) < 1e-6);
    CHECK(std::abs(local.north_m - 20.0) < 1e-6);
}

TEST_CASE("VECR nadir adapter contracts footprint with hard bounds", "[vecr]") {
    const Route route{{{0, 0}, {100, 0}}};
    NadirCaptureInput input;
    input.attempt_id = "attempt-2";
    input.capture_id = "image-1";
    input.calibration_id = "calibration-1";
    input.monotonic_ms = 1000;
    input.camera_center = {50, 1};
    input.altitude_above_ground_m = 10;
    input.altitude_error_hard_bound_m = 1;
    input.horizontal_error_hard_bound_m = 0.5;
    input.along_fov_deg = 60;
    input.cross_fov_deg = 90;
    input.required_strip_half_width_m = 4;
    input.capture_succeeded = true;
    input.image_quality_established = true;
    input.nadir_route_alignment_established = true;
    input.flat_ground_established = true;
    const auto evidence = BuildNadirStripEvidence(route, input);
    REQUIRE(evidence);
    CHECK(evidence->guaranteed_full_strip_interval.begin_m > 45);
    CHECK(evidence->guaranteed_full_strip_interval.end_m < 55);
    input.image_quality_established = false;
    CHECK_FALSE(BuildNadirStripEvidence(route, input));
    input.image_quality_established = true;
    input.camera_center.north_m = 5;
    CHECK_FALSE(BuildNadirStripEvidence(route, input));
    input.camera_center.north_m = 1;
    input.cross_fov_deg = 0;
    CHECK_FALSE(BuildNadirStripEvidence(route, input));
    input.cross_fov_deg = 90;
    input.altitude_error_hard_bound_m = 11;
    CHECK_FALSE(BuildNadirStripEvidence(route, input));
    input.altitude_error_hard_bound_m = 1;
    CHECK_FALSE(BuildNadirStripEvidence(Route{{{0, 0}, {50, 0}, {100, 0}}}, input));
}
