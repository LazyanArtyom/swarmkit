#pragma once

/// @file commands.h
/// @brief Public aggregator for all agent command types.
///
/// Include this file to get the full Command variant, CommandContext, and
/// CommandEnvelope.  To include only a specific command category (e.g. to
/// reduce compile-time dependencies in a backend), include the relevant
/// sub-header directly:
///
///   #include "swarmkit/commands/flight.h"
///   #include "swarmkit/commands/nav.h"
///   #include "swarmkit/commands/swarm.h"
///   #include "swarmkit/commands/payload.h"

#include <chrono>
#include <cstdint>
#include <variant>

#include "swarmkit/commands/flight.h"
#include "swarmkit/commands/nav.h"
#include "swarmkit/commands/payload.h"
#include "swarmkit/commands/swarm.h"

namespace swarmkit::agent {

// ---------------------------------------------------------------------------
// CommandPriority
// ---------------------------------------------------------------------------

/**
 * @brief Standard priority levels for command arbitration.
 *
 * Values map to the @c CommandPriority proto enum and are enforced by
 * CommandArbiter.  Higher numeric value = higher authority.
 *
 * New levels may be inserted between existing ones without breaking the
 * relative ordering (e.g. @c kLocalSafety = 15 between kSupervisor and
 * kOverride).
 */
enum class CommandPriority : std::uint8_t {
    kOperator   =  0,   ///< Default: CLI operators and SDK users.
    kSupervisor = 10,   ///< Automated local systems (on-board mission executor).
    kOverride   = 20,   ///< Remote deviation-correction or safety override server.
    kEmergency  = 100,  ///< Bypasses arbitration; always executes immediately.
};

// ---------------------------------------------------------------------------
// Command — top-level variant of all command categories.
// ---------------------------------------------------------------------------

/**
 * @brief Top-level discriminated union of all command categories.
 *
 * Each alternative is itself a category variant (FlightCmd, NavCmd, …).
 * Dispatch with two levels of core::Overloaded:
 *
 * @code
 * std::visit(core::Overloaded{
 *     [&](const FlightCmd& cmd) {
 *         std::visit(core::Overloaded{
 *             [&](const CmdArm&)         { ... },
 *             [&](const CmdDisarm&)      { ... },
 *             [&](const CmdTakeoff& cmd) { ... },
 *             [&](const CmdLand&)        { ... },
 *         }, cmd);
 *     },
 *     [&](const NavCmd&)     { ... },
 *     [&](const SwarmCmd&)   { ... },
 *     [&](const PayloadCmd&) { ... },
 * }, envelope.command);
 * @endcode
 *
 * @par Adding a new category
 * 1. Create @c commands/my_category.h and define @c MyCategoryCmd.
 * 2. Add @c MyCategoryCmd to this variant.
 * 3. Add @c #include above.
 * 4. Handle it in every IDroneBackend implementation (compile error if missed).
 * 5. Add proto messages and map them in server.cpp.
 */
using Command = std::variant<FlightCmd, NavCmd, SwarmCmd, PayloadCmd>;

// ---------------------------------------------------------------------------
// CommandContext
// ---------------------------------------------------------------------------

/// @brief Routing and scheduling metadata attached to every command.
struct CommandContext {
    std::string     drone_id;    ///< Target drone identifier.
    std::string     client_id;   ///< Originating client identifier (used by CommandArbiter).
    CommandPriority priority{CommandPriority::kOperator};  ///< Arbitration priority.
    std::chrono::system_clock::time_point deadline;        ///< Execution deadline (zero = none).
    std::string     correlation_id;  ///< Caller-assigned ID for tracing/deduplication.
};

// ---------------------------------------------------------------------------
// CommandEnvelope
// ---------------------------------------------------------------------------

/**
 * @brief Single dispatch unit passed to IDroneBackend::Execute().
 *
 * Bundles the command payload and its routing/priority context so that
 * backends and the arbiter never need to correlate two separate arguments.
 */
struct CommandEnvelope {
    CommandContext context;
    Command        command;
};

}  // namespace swarmkit::agent
