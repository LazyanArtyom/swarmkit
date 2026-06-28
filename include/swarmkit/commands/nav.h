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
/// Navigation commands -- point-to-point movement and position holding.
/// ---------------------------------------------------------------------------

/// @brief Fly to an absolute geographic position.
struct CmdSetWaypoint {
    double lat_deg{};   ///< Target latitude in decimal degrees.
    double lon_deg{};   ///< Target longitude in decimal degrees.
    double alt_m{};     ///< Target altitude in metres (AGL).
    float speed_mps{};  ///< Maximum travel speed in m/s (0 = backend default).
};

/// @brief Return to the home / launch position and land.
struct CmdReturnHome {};

/// @brief Hold the current position until the next command arrives.
struct CmdHoldPosition {};

/// @brief Change the vehicle speed setpoint.
struct CmdSetSpeed {
    float ground_mps{};  ///< Ground speed in m/s. <= 0 asks backend/autopilot default.
};

/// @brief Reposition the vehicle in guided mode.
struct CmdGoto {
    double lat_deg{};     ///< Target latitude in decimal degrees.
    double lon_deg{};     ///< Target longitude in decimal degrees.
    double alt_m{};       ///< Target altitude in metres AGL.
    float speed_mps{};    ///< Optional travel speed in m/s; 0 = backend default.
    float yaw_deg{};      ///< Optional target heading in degrees.
    bool use_yaw{false};  ///< true when yaw_deg should be applied.
};

/// @brief Pause current guided motion and hold.
struct CmdPause {};

/// @brief Resume current guided motion.
struct CmdResume {};

/// @brief Rotate vehicle heading.
struct CmdSetYaw {
    float yaw_deg{};       ///< Target or relative yaw in degrees.
    float rate_deg_s{};    ///< Yaw rate in degrees per second; 0 = backend default.
    bool relative{false};  ///< true = yaw_deg is relative to current heading.
};

/// @brief Send a local/body velocity setpoint for manual-style control.
struct CmdVelocity {
    float vx_mps{};              ///< North/body-forward velocity depending on body_frame.
    float vy_mps{};              ///< East/body-right velocity depending on body_frame.
    float vz_mps{};              ///< Down velocity in NED; negative climbs.
    std::int32_t duration_ms{};  ///< Intended client command duration; 0 = one setpoint.
    bool body_frame{false};      ///< true = body frame, false = local NED.
};

/// @brief Set home to current location or explicit coordinates.
struct CmdSetHome {
    bool use_current{true};  ///< true = use current vehicle position.
    double lat_deg{};        ///< Explicit home latitude in decimal degrees.
    double lon_deg{};        ///< Explicit home longitude in decimal degrees.
    double alt_m{};          ///< Explicit home altitude in metres.
};

/**
 * @brief Variant of all navigation commands.
 *
 * Backends that do not support navigation should return
 * core::Result::Rejected("nav commands not supported").
 */
using NavCmd = std::variant<CmdSetWaypoint, CmdReturnHome, CmdHoldPosition, CmdSetSpeed, CmdGoto,
                            CmdPause, CmdResume, CmdSetYaw, CmdVelocity, CmdSetHome>;

}  // namespace swarmkit::commands
