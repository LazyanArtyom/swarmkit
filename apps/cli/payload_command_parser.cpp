// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "payload_command_parser.h"

#include "common/arg_utils.h"
#include "constants.h"
#include "options.h"

namespace swarmkit::apps::cli::internal {

using namespace swarmkit::commands;  // NOLINT(google-build-using-namespace)

std::expected<Command, std::string> BuildPayloadCommand(const std::vector<std::string>& actions,
                                                        int argc, char** argv) {
    if (actions.empty()) {
        return std::unexpected("missing payload action");
    }

    const std::string& action = actions.front();
    const auto camera =
        ParseIntArg(common::GetOptionValue(argc, argv, "--camera", kDefaultZero), "--camera");
    if (!camera.has_value()) {
        return std::unexpected(camera.error());
    }

    if (action == "photo") {
        return PayloadCmd{CmdPhoto{*camera}};
    }
    if (action == "photo-interval-start") {
        const auto interval = ParseFloatArg(
            common::GetOptionValue(argc, argv, "--interval", kDefaultZero), "--interval");
        const auto count =
            ParseIntArg(common::GetOptionValue(argc, argv, "--count", kDefaultZero), "--count");
        if (!interval.has_value()) {
            return std::unexpected(interval.error());
        }
        if (!count.has_value()) {
            return std::unexpected(count.error());
        }
        return PayloadCmd{CmdPhotoIntervalStart{
            .interval_s = *interval,
            .count = *count,
            .camera_id = *camera,
        }};
    }
    if (action == "photo-interval-stop") {
        return PayloadCmd{CmdPhotoIntervalStop{*camera}};
    }
    if (action == "video-start" || action == "video-stop") {
        const auto stream =
            ParseIntArg(common::GetOptionValue(argc, argv, "--stream", kDefaultZero), "--stream");
        if (!stream.has_value()) {
            return std::unexpected(stream.error());
        }
        if (action == "video-start") {
            return PayloadCmd{CmdVideoStart{
                .stream_id = *stream,
                .camera_id = *camera,
            }};
        }
        return PayloadCmd{CmdVideoStop{
            .stream_id = *stream,
            .camera_id = *camera,
        }};
    }
    if (action == "gimbal-point") {
        const auto pitch =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--pitch", kDefaultZero), "--pitch");
        const auto roll =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--roll", kDefaultZero), "--roll");
        const auto yaw =
            ParseFloatArg(common::GetOptionValue(argc, argv, "--yaw", kDefaultZero), "--yaw");
        if (!pitch.has_value()) {
            return std::unexpected(pitch.error());
        }
        if (!roll.has_value()) {
            return std::unexpected(roll.error());
        }
        if (!yaw.has_value()) {
            return std::unexpected(yaw.error());
        }
        return PayloadCmd{CmdGimbalPoint{
            .pitch_deg = *pitch,
            .roll_deg = *roll,
            .yaw_deg = *yaw,
        }};
    }
    if (action == "roi-location") {
        const auto lat = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lat"), "--lat");
        const auto lon = ParseDoubleArg(common::GetOptionValue(argc, argv, "--lon"), "--lon");
        const auto alt = ParseDoubleArg(common::GetOptionValue(argc, argv, "--alt"), "--alt");
        const auto gimbal =
            ParseIntArg(common::GetOptionValue(argc, argv, "--gimbal", kDefaultZero), "--gimbal");
        if (!lat.has_value()) {
            return std::unexpected(lat.error());
        }
        if (!lon.has_value()) {
            return std::unexpected(lon.error());
        }
        if (!alt.has_value()) {
            return std::unexpected(alt.error());
        }
        if (!gimbal.has_value()) {
            return std::unexpected(gimbal.error());
        }
        return PayloadCmd{CmdRoiLocation{
            .lat_deg = *lat,
            .lon_deg = *lon,
            .alt_m = *alt,
            .gimbal_id = *gimbal,
        }};
    }
    if (action == "roi-clear") {
        const auto gimbal =
            ParseIntArg(common::GetOptionValue(argc, argv, "--gimbal", kDefaultZero), "--gimbal");
        if (!gimbal.has_value()) {
            return std::unexpected(gimbal.error());
        }
        return PayloadCmd{CmdRoiClear{*gimbal}};
    }
    if (action == "servo") {
        const auto servo =
            ParseIntArg(common::GetOptionValue(argc, argv, "--servo", kDefaultZero), "--servo");
        const auto pwm =
            ParseIntArg(common::GetOptionValue(argc, argv, "--pwm", kDefaultZero), "--pwm");
        if (!servo.has_value()) {
            return std::unexpected(servo.error());
        }
        if (!pwm.has_value()) {
            return std::unexpected(pwm.error());
        }
        return PayloadCmd{CmdServo{
            .servo = *servo,
            .pwm = *pwm,
        }};
    }
    if (action == "relay") {
        const auto relay =
            ParseIntArg(common::GetOptionValue(argc, argv, "--relay", kDefaultZero), "--relay");
        if (!relay.has_value()) {
            return std::unexpected(relay.error());
        }
        return PayloadCmd{CmdRelay{
            .relay = *relay,
            .enabled = common::HasFlag(argc, argv, "--on"),
        }};
    }
    if (action == "gripper") {
        const auto gripper =
            ParseIntArg(common::GetOptionValue(argc, argv, "--gripper", kDefaultZero), "--gripper");
        if (!gripper.has_value()) {
            return std::unexpected(gripper.error());
        }
        return PayloadCmd{CmdGripper{
            .gripper = *gripper,
            .release = common::HasFlag(argc, argv, "--release"),
        }};
    }
    return std::unexpected("Unknown payload action: " + action);
}

}  // namespace swarmkit::apps::cli::internal
