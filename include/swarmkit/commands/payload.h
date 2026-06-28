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

/// @brief Capture one still image.
struct CmdPhoto {
    int camera_id{};  ///< Camera identifier; 0 lets the backend choose the default camera.
};

/// @brief Start periodic still-image capture.
struct CmdPhotoIntervalStart {
    float interval_s{};  ///< Interval between captures in seconds.
    int count{};         ///< Number of captures; 0 = keep capturing until stopped.
    int camera_id{};     ///< Camera identifier; 0 lets the backend choose the default camera.
};

/// @brief Stop periodic still-image capture.
struct CmdPhotoIntervalStop {
    int camera_id{};  ///< Camera identifier; 0 lets the backend choose the default camera.
};

/// @brief Start video recording or streaming.
struct CmdVideoStart {
    int stream_id{};  ///< Backend-defined stream or recording identifier.
    int camera_id{};  ///< Camera identifier; 0 lets the backend choose the default camera.
};

/// @brief Stop video recording or streaming.
struct CmdVideoStop {
    int stream_id{};  ///< Backend-defined stream or recording identifier.
    int camera_id{};  ///< Camera identifier; 0 lets the backend choose the default camera.
};

/// @brief Point a gimbal by Euler angles.
struct CmdGimbalPoint {
    float pitch_deg{};  ///< Pitch angle in degrees.
    float roll_deg{};   ///< Roll angle in degrees.
    float yaw_deg{};    ///< Yaw angle in degrees.
};

/// @brief Point a gimbal at a geographic region of interest.
struct CmdRoiLocation {
    double lat_deg{};  ///< ROI latitude in decimal degrees.
    double lon_deg{};  ///< ROI longitude in decimal degrees.
    double alt_m{};    ///< ROI altitude in metres.
    int gimbal_id{};   ///< Gimbal identifier; 0 lets the backend choose the default gimbal.
};

/// @brief Clear the active region of interest.
struct CmdRoiClear {
    int gimbal_id{};  ///< Gimbal identifier; 0 lets the backend choose the default gimbal.
};

/// @brief Set a PWM value on a servo output.
struct CmdServo {
    int servo{};  ///< Servo output number.
    int pwm{};    ///< Pulse width in microseconds.
};

/// @brief Set a relay output state.
struct CmdRelay {
    int relay{};     ///< Relay output number.
    bool enabled{};  ///< true to enable/close, false to disable/open.
};

/// @brief Control a gripper payload.
struct CmdGripper {
    int gripper{};   ///< Gripper identifier.
    bool release{};  ///< true to release, false to grip/close.
};

/**
 * @brief Variant of all payload-control commands.
 *
 * Backends that have no payload should return
 * core::Result::Rejected("payload commands not supported").
 */
using PayloadCmd =
    std::variant<CmdPhoto, CmdPhotoIntervalStart, CmdPhotoIntervalStop, CmdVideoStart, CmdVideoStop,
                 CmdGimbalPoint, CmdRoiLocation, CmdRoiClear, CmdServo, CmdRelay, CmdGripper>;

}  // namespace swarmkit::commands
