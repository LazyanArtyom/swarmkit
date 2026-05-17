// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include "mavlink_common.h"

namespace swarmkit::agent::mavlink {
namespace {

[[nodiscard]] mavlink_heartbeat_t MakeHeartbeat(std::uint32_t custom_mode,
                                                std::uint8_t base_mode = 0U) {
    mavlink_heartbeat_t heartbeat{};
    heartbeat.custom_mode = custom_mode;
    heartbeat.base_mode = base_mode;
    return heartbeat;
}

[[nodiscard]] std::uint32_t MakePx4CustomMode(std::uint8_t main_mode, std::uint8_t sub_mode = 0U) {
    return (static_cast<std::uint32_t>(main_mode) << 16U) |
           (static_cast<std::uint32_t>(sub_mode) << 24U);
}

TEST_CASE("MAVLink mode strings decode ArduPilot Copter custom modes",
          "[agent][mavlink][telemetry]") {
    CHECK(ModeString(MakeHeartbeat(4U), MavlinkAutopilotProfile::kArdupilotCopter) == "GUIDED");
    CHECK(ModeString(MakeHeartbeat(5U, MAV_MODE_FLAG_SAFETY_ARMED),
                     MavlinkAutopilotProfile::kArdupilotCopter) == "LOITER");
    CHECK(ModeString(MakeHeartbeat(21U), MavlinkAutopilotProfile::kArdupilotCopter) ==
          "SMART_RTL");
    CHECK(ModeString(MakeHeartbeat(123U), MavlinkAutopilotProfile::kArdupilotCopter) ==
          "ARDUPILOT_COPTER(custom=123)");
}

TEST_CASE("MAVLink mode strings decode ArduPilot Plane custom modes",
          "[agent][mavlink][telemetry]") {
    CHECK(ModeString(MakeHeartbeat(0U), MavlinkAutopilotProfile::kArdupilotPlane) == "MANUAL");
    CHECK(ModeString(MakeHeartbeat(15U), MavlinkAutopilotProfile::kArdupilotPlane) == "GUIDED");
    CHECK(ModeString(MakeHeartbeat(20U), MavlinkAutopilotProfile::kArdupilotPlane) == "QLAND");
    CHECK(ModeString(MakeHeartbeat(123U), MavlinkAutopilotProfile::kArdupilotPlane) ==
          "ARDUPILOT_PLANE(custom=123)");
}

TEST_CASE("MAVLink mode strings decode PX4 custom mode bit fields",
          "[agent][mavlink][telemetry]") {
    CHECK(ModeString(MakeHeartbeat(MakePx4CustomMode(1U)), MavlinkAutopilotProfile::kPx4) ==
          "MANUAL");
    CHECK(ModeString(MakeHeartbeat(MakePx4CustomMode(4U, 5U)), MavlinkAutopilotProfile::kPx4) ==
          "RTL");
    CHECK(ModeString(MakeHeartbeat(MakePx4CustomMode(4U, 6U)), MavlinkAutopilotProfile::kPx4) ==
          "LAND");
    CHECK(ModeString(MakeHeartbeat(MakePx4CustomMode(6U)), MavlinkAutopilotProfile::kPx4) ==
          "OFFBOARD");
    CHECK(ModeString(MakeHeartbeat(MakePx4CustomMode(4U, 99U)),
                     MavlinkAutopilotProfile::kPx4) == "AUTO(sub=99)");
}

}  // namespace
}  // namespace swarmkit::agent::mavlink
