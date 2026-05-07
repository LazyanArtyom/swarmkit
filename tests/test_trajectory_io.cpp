// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include <sstream>

#include "swarmkit/client/trajectory_io.h"

using swarmkit::client::LoadTrajectoryPlan;
using swarmkit::client::TrajectoryFileFormat;
using swarmkit::client::TrajectoryLoadOptions;

TEST_CASE("Trajectory IO loads planner JSONL streams", "[client][trajectory]") {
    std::istringstream input{
        "{\"execution_id\":\"jsonl-path-1\",\"drone_id\":\"drone-1\",\"frame\":\"global\","
        "\"min_battery_percent\":30,\"require_gps\":true}\n"
        "{\"time_offset_ms\":0,\"lat\":40.18,\"lon\":44.51,\"alt_m\":10,\"yaw_deg\":90}\n"
        "{\"time_offset_ms\":5000,\"lat\":40.1801,\"lon\":44.5101,\"alt_m\":12}\n"};

    const auto plan = LoadTrajectoryPlan(input, TrajectoryFileFormat::kJsonLines);

    REQUIRE(plan.has_value());
    CHECK(plan->execution_id == "jsonl-path-1");
    CHECK(plan->drone_id == "drone-1");
    CHECK(plan->validation.min_battery_percent == 30.0F);
    CHECK(plan->validation.require_gps);
    REQUIRE(plan->points.size() == 2);
    CHECK(plan->points[0].time_offset_ms == 0);
    CHECK(plan->points[0].position.lat_deg == 40.18);
    CHECK(plan->points[0].position.lon_deg == 44.51);
    CHECK(plan->points[0].position.alt_m == 10.0);
    CHECK(plan->points[0].has_yaw);
    CHECK(plan->points[0].yaw_deg == 90.0F);
    CHECK(plan->points[1].time_offset_ms == 5000);
}

TEST_CASE("Trajectory IO loads CSV rows with SDK defaults", "[client][trajectory]") {
    std::istringstream input{
        "time_offset_ms,lat,lon,alt_m,yaw_deg\n"
        "0,40.18,44.51,10,90\n"
        "5000,40.1801,44.5101,12,\n"};
    TrajectoryLoadOptions options;
    options.default_execution_id = "csv-path-1";
    options.fallback_drone_id = "drone-2";

    const auto plan = LoadTrajectoryPlan(input, TrajectoryFileFormat::kCsv, options);

    REQUIRE(plan.has_value());
    CHECK(plan->execution_id == "csv-path-1");
    CHECK(plan->drone_id == "drone-2");
    REQUIRE(plan->points.size() == 2);
    CHECK(plan->points[1].time_offset_ms == 5000);
    CHECK(plan->points[1].position.alt_m == 12.0);
    CHECK_FALSE(plan->points[1].has_yaw);
}
