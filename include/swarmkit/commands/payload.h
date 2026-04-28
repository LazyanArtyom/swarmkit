// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <variant>

namespace swarmkit::commands {

/// ---------------------------------------------------------------------------
/// Payload commands -- cameras, gimbals, sensors, and other onboard hardware.
/// Populated as hardware integrations are added.
/// ---------------------------------------------------------------------------

struct CmdPhoto {
    int camera_id{};
};

struct CmdPhotoIntervalStart {
    float interval_s{};
    int count{};       ///< 0 = keep capturing until stopped.
    int camera_id{};
};

struct CmdPhotoIntervalStop {
    int camera_id{};
};

struct CmdVideoStart {
    int stream_id{};
    int camera_id{};
};

struct CmdVideoStop {
    int stream_id{};
    int camera_id{};
};

struct CmdGimbalPoint {
    float pitch_deg{};
    float roll_deg{};
    float yaw_deg{};
};

struct CmdRoiLocation {
    double lat_deg{};
    double lon_deg{};
    double alt_m{};
    int gimbal_id{};
};

struct CmdRoiClear {
    int gimbal_id{};
};

struct CmdServo {
    int servo{};
    int pwm{};
};

struct CmdRelay {
    int relay{};
    bool enabled{};
};

struct CmdGripper {
    int gripper{};
    bool release{};
};

/**
 * @brief Variant of all payload-control commands.
 *
 * Backends that have no payload should return
 * core::Result::Rejected("payload commands not supported").
 */
using PayloadCmd = std::variant<CmdPhoto, CmdPhotoIntervalStart, CmdPhotoIntervalStop,
                                CmdVideoStart, CmdVideoStop, CmdGimbalPoint, CmdRoiLocation,
                                CmdRoiClear, CmdServo, CmdRelay, CmdGripper>;

}  // namespace swarmkit::commands
