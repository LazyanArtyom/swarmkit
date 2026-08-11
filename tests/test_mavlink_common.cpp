// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <deque>
#include <string>

#include "mavlink_command_executor.h"
#include "mavlink_common.h"
#include "mavlink_telemetry_decoder.h"

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

TEST_CASE("MAVLink command ACK lookup survives later unrelated ACKs",
          "[agent][mavlink][commands]") {
    constexpr std::uint8_t kSourceSystem = 245;
    constexpr std::uint8_t kSourceComponent = 191;
    std::deque<SequencedCommandAck> history{
        SequencedCommandAck{
            .sequence = 1,
            .ack =
                CommandAck{
                    .command = MAV_CMD_SET_MESSAGE_INTERVAL,
                    .result = MAV_RESULT_ACCEPTED,
                    .target_system = 0,
                    .target_component = 0,
                },
        },
        SequencedCommandAck{
            .sequence = 2,
            .ack =
                CommandAck{
                    .command = MAV_CMD_COMPONENT_ARM_DISARM,
                    .result = MAV_RESULT_ACCEPTED,
                    .target_system = kSourceSystem,
                    .target_component = kSourceComponent,
                },
        },
        SequencedCommandAck{
            .sequence = 3,
            .ack =
                CommandAck{
                    .command = MAV_CMD_SET_MESSAGE_INTERVAL,
                    .result = MAV_RESULT_ACCEPTED,
                    .target_system = 0,
                    .target_component = 0,
                },
        },
    };

    const auto ack = FindCommandAckAfter(history, MAV_CMD_COMPONENT_ARM_DISARM, 1, kSourceSystem,
                                         kSourceComponent);
    const CommandAck matched_ack = ack.value_or(CommandAck{});
    REQUIRE(ack.has_value());
    CHECK(matched_ack.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(matched_ack.result == MAV_RESULT_ACCEPTED);
}

TEST_CASE("MAVLink command ACK lookup ignores in-progress ACKs", "[agent][mavlink][commands]") {
    std::deque<SequencedCommandAck> history{
        SequencedCommandAck{
            .sequence = 1,
            .ack =
                CommandAck{
                    .command = MAV_CMD_COMPONENT_ARM_DISARM,
                    .result = MAV_RESULT_IN_PROGRESS,
                    .target_system = 0,
                    .target_component = 0,
                },
        },
    };

    const auto ack = FindCommandAckAfter(history, MAV_CMD_COMPONENT_ARM_DISARM, 0, 245, 191);
    CHECK_FALSE(ack.has_value());
}

TEST_CASE("MAVLink measurement provenance does not refresh position on heartbeat or attitude",
          "[agent][mavlink][telemetry][provenance]") {
    MavlinkTelemetryDecoder decoder;
    TelemetryCache cache;
    MavlinkStateCache state;

    mavlink_global_position_int_t position{};
    position.time_boot_ms = 1234;
    position.lat = 400'000'000;
    position.lon = 440'000'000;
    position.alt = 10'000;
    position.relative_alt = 8'000;
    mavlink_message_t position_message{};
    mavlink_msg_global_position_int_encode(1, 1, &position_message, &position);
    const auto position_result =
        decoder.Decode(position_message, &cache, &state, MavlinkAutopilotProfile::kArdupilotCopter);
    REQUIRE(position_result.should_publish);
    CHECK(position_result.provenance.position.updated);
    CHECK(position_result.provenance.velocity.updated);
    CHECK(position_result.provenance.position.source == "mavlink.GLOBAL_POSITION_INT");
    REQUIRE(position_result.provenance.position.source_time.timestamp_ms.has_value());
    CHECK(*position_result.provenance.position.source_time.timestamp_ms == 1234);
    CHECK(position_result.provenance.position.source_time.clock_domain ==
          core::ClockDomain::kVehicleBoot);

    mavlink_heartbeat_t heartbeat{};
    heartbeat.system_status = MAV_STATE_ACTIVE;
    mavlink_message_t heartbeat_message{};
    mavlink_msg_heartbeat_encode(1, 1, &heartbeat_message, &heartbeat);
    const auto heartbeat_result = decoder.Decode(heartbeat_message, &cache, &state,
                                                 MavlinkAutopilotProfile::kArdupilotCopter);
    REQUIRE(heartbeat_result.should_publish);
    CHECK_FALSE(heartbeat_result.provenance.position.updated);
    CHECK(heartbeat_result.provenance.vehicle_state.updated);

    mavlink_attitude_t attitude{};
    attitude.time_boot_ms = 9999;
    mavlink_message_t attitude_message{};
    mavlink_msg_attitude_encode(1, 1, &attitude_message, &attitude);
    const auto attitude_result =
        decoder.Decode(attitude_message, &cache, &state, MavlinkAutopilotProfile::kArdupilotCopter);
    REQUIRE(attitude_result.should_publish);
    CHECK_FALSE(attitude_result.provenance.position.updated);
}

TEST_CASE("MAVLink GPS accuracy retains backend-specific uncertainty semantics",
          "[agent][mavlink][telemetry][uncertainty]") {
    MavlinkTelemetryDecoder decoder;
    TelemetryCache cache;
    MavlinkStateCache state;

    mavlink_gps_raw_int_t gps{};
    gps.time_usec = 1'234'000;
    gps.fix_type = GPS_FIX_TYPE_3D_FIX;
    gps.h_acc = 700;
    gps.v_acc = 1'200;
    gps.vel_acc = 250;
    mavlink_message_t message{};
    mavlink_msg_gps_raw_int_encode(1, 1, &message, &gps);

    const auto result =
        decoder.Decode(message, &cache, &state, MavlinkAutopilotProfile::kArdupilotCopter);
    REQUIRE(result.should_publish);
    REQUIRE(cache.accuracy.horizontal_position.has_value());
    CHECK(cache.accuracy.horizontal_position->value == 0.7F);
    CHECK(cache.accuracy.horizontal_position->descriptor.semantics ==
          core::UncertaintySemantics::kBackendSpecific);
    CHECK(cache.accuracy.horizontal_position->descriptor.source == "mavlink.GPS_RAW_INT.h_acc");
    REQUIRE(cache.accuracy.vertical_position.has_value());
    CHECK(cache.accuracy.vertical_position->descriptor.semantics ==
          core::UncertaintySemantics::kBackendSpecific);
    REQUIRE(cache.accuracy.speed.has_value());
    CHECK(cache.accuracy.speed->descriptor.semantics ==
          core::UncertaintySemantics::kBackendSpecific);
    CHECK(cache.accuracy.horizontal_position->descriptor.semantics !=
          core::UncertaintySemantics::kDeterministicHardBound);
    CHECK_FALSE(cache.accuracy.horizontal_position->descriptor.confidence_level.has_value());
}

TEST_CASE("MAVLink capability evidence is typed and motion bounds remain unknown",
          "[agent][mavlink][capabilities]") {
    MavlinkBackendConfig config;
    const core::BackendCapabilities capabilities = MavlinkCommandExecutor::Capabilities(config);

    CHECK(capabilities.evidence.source_timestamp == core::CapabilitySupport::kSupported);
    CHECK(capabilities.evidence.position_estimate == core::CapabilitySupport::kSupported);
    CHECK(capabilities.evidence.horizontal_position_uncertainty ==
          core::CapabilitySupport::kSupported);
    CHECK(capabilities.evidence.horizontal_velocity_uncertainty ==
          core::CapabilitySupport::kUnsupported);
    CHECK(capabilities.evidence.estimator_health == core::CapabilitySupport::kSupported);
    CHECK(capabilities.evidence.failsafe_state == core::CapabilitySupport::kSupported);
    CHECK_FALSE(capabilities.max_horizontal_speed.has_value());
}

}  // namespace
}  // namespace swarmkit::agent::mavlink
