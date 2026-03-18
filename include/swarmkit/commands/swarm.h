#pragma once

#include <cstdint>
#include <string>
#include <variant>

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// Swarm commands — multi-agent coordination and choreography.
// ---------------------------------------------------------------------------

/// @brief Assign a logical role label to the drone within a swarm.
struct CmdSetRole {
    std::string role;  ///< Freeform role string (e.g. "leader", "follower-3").
};

/// @brief Assign the drone a slot in a named geometric formation.
struct CmdSetFormation {
    std::string formation_id;  ///< Identifier of the formation template.
    int         slot_index{};  ///< Zero-based position within the formation.
};

/**
 * @brief Execute a pre-defined choreography sequence.
 *
 * @par Swarm animation synchronisation
 * @c sync_unix_ms is an absolute UNIX timestamp (milliseconds) at which all
 * agents in the swarm start the sequence simultaneously.  The ground-control
 * station sends the same @c sync_unix_ms to every drone so that the animation
 * begins at the same wall-clock instant regardless of network jitter.
 * Agents that receive the command after the sync time should start
 * immediately (they missed the window) and log a warning.
 */
struct CmdRunSequence {
    std::string  sequence_id;   ///< Identifier of the choreography sequence.
    std::int64_t sync_unix_ms{};  ///< Absolute start time; 0 = start immediately.
};

/**
 * @brief Variant of all multi-agent coordination commands.
 *
 * Backends that do not participate in a swarm should return
 * core::Result::Rejected("swarm commands not supported").
 */
using SwarmCmd = std::variant<CmdSetRole, CmdSetFormation, CmdRunSequence>;

}  // namespace swarmkit::agent
