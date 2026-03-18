#pragma once

#include <variant>

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// Flight commands — fundamental vehicle control.
// These map to direct flight-controller actions (arm, disarm, takeoff, land).
// ---------------------------------------------------------------------------

struct CmdArm {};
struct CmdDisarm {};
struct CmdLand {};

/// @brief Climb to the given altitude above the launch point.
struct CmdTakeoff {
    double alt_m{};  ///< Target altitude in metres (AGL).
};

/**
 * @brief Variant of all fundamental vehicle control commands.
 *
 * Implemented by every IDroneBackend.  Adding a new flight command here
 * without updating all backends produces a compile error (exhaustive visit).
 */
using FlightCmd = std::variant<CmdArm, CmdDisarm, CmdTakeoff, CmdLand>;

}  // namespace swarmkit::agent
