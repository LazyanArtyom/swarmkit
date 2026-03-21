// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

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

/**
 * @brief Variant of all navigation commands.
 *
 * Backends that do not support navigation should return
 * core::Result::Rejected("nav commands not supported").
 */
using NavCmd = std::variant<CmdSetWaypoint, CmdReturnHome, CmdHoldPosition>;

}  // namespace swarmkit::commands
