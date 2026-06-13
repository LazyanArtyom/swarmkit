// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string>

#include "mavlink_command_executor.h"
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

TEST_CASE("MAVLink mode strings decode ArduPilot Copter custom modes",
          "[agent][mavlink][telemetry]") {
    CHECK(ModeString(MakeHeartbeat(4U), MavlinkAutopilotProfile::kArdupilotCopter) == "GUIDED");
    CHECK(ModeString(MakeHeartbeat(5U, MAV_MODE_FLAG_SAFETY_ARMED),
                     MavlinkAutopilotProfile::kArdupilotCopter) == "LOITER");
    CHECK(ModeString(MakeHeartbeat(21U), MavlinkAutopilotProfile::kArdupilotCopter) == "SMART_RTL");
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

TEST_CASE("MAVLink command ACK failure includes STATUSTEXT reason", "[agent][mavlink][commands]") {
    MavlinkCommandAckResult ack;
    ack.has_ack = true;
    ack.ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
    ack.ack.result = MAV_RESULT_FAILED;
    ack.has_status_text = true;
    ack.status_text.text = "PreArm: Bad GPS Position";

    const auto result = ack.ToCoreResult();
    CHECK_FALSE(result.IsOk());
    CHECK(result.message.find("COMMAND_ACK command=400") != std::string::npos);
    CHECK(result.message.find("PreArm: Bad GPS Position") != std::string::npos);
}

TEST_CASE("MAVLink arm/disarm command specs use ArduPilot force magic",
          "[agent][mavlink][commands]") {
    const auto arm = MavlinkCommandExecutor::ArmDisarmCommand(/*arm=*/true, /*force=*/false);
    CHECK(arm.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(arm.params[0] == 1.0F);
    CHECK(arm.params[1] == 0.0F);

    const auto force_arm = MavlinkCommandExecutor::ArmDisarmCommand(/*arm=*/true, /*force=*/true);
    CHECK(force_arm.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(force_arm.params[0] == 1.0F);
    CHECK(force_arm.params[1] == kMavlinkForceArmDisarmMagic);

    const auto disarm = MavlinkCommandExecutor::ArmDisarmCommand(/*arm=*/false, /*force=*/false);
    CHECK(disarm.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(disarm.params[0] == 0.0F);
    CHECK(disarm.params[1] == 0.0F);

    const auto force_disarm =
        MavlinkCommandExecutor::ArmDisarmCommand(/*arm=*/false, /*force=*/true);
    CHECK(force_disarm.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(force_disarm.params[0] == 0.0F);
    CHECK(force_disarm.params[1] == kMavlinkForceArmDisarmMagic);
}

}  // namespace
}  // namespace swarmkit::agent::mavlink
