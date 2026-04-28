// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <string>
#include <variant>

namespace swarmkit::commands {

/// ---------------------------------------------------------------------------
/// Flight commands -- fundamental vehicle control.
/// These map to direct flight-controller actions (arm, disarm, takeoff, land).
/// ---------------------------------------------------------------------------

/// @brief Arm the vehicle's motors.
struct CmdArm {};

/// @brief Disarm the vehicle's motors.
struct CmdDisarm {};

/// @brief Land the vehicle at the current position.
struct CmdLand {};

/// @brief Climb to the given altitude above the launch point.
struct CmdTakeoff {
    double alt_m{};  ///< Target altitude in metres (AGL).
};

/// @brief Switch to an autopilot mode by name or explicit custom mode.
struct CmdSetMode {
    std::string mode;     ///< Friendly mode name, e.g. guided, loiter, auto, rtl.
    int custom_mode{-1};  ///< Autopilot-specific custom mode; -1 means use mode name.
};

/// @brief Force-disarm the vehicle, bypassing normal landed checks where supported.
struct CmdForceDisarm {};

/// @brief Request immediate flight termination where supported.
struct CmdFlightTerminate {};

/**
 * @brief Variant of all fundamental vehicle control commands.
 *
 * Implemented by every IDroneBackend.  Adding a new flight command here
 * without updating all backends produces a compile error (exhaustive visit).
 */
using FlightCmd = std::variant<CmdArm, CmdDisarm, CmdTakeoff, CmdLand, CmdSetMode, CmdForceDisarm,
                               CmdFlightTerminate>;

}  // namespace swarmkit::commands
