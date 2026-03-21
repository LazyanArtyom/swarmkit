#pragma once

#include <variant>

namespace swarmkit::commands {

/// ---------------------------------------------------------------------------
/// Payload commands -- cameras, gimbals, sensors, and other onboard hardware.
/// Populated as hardware integrations are added.
/// ---------------------------------------------------------------------------

/// @brief Placeholder — no payload commands defined yet.
/// @internal Remove this and replace with real payload command structs.
struct CmdPayloadReserved {};

/**
 * @brief Variant of all payload-control commands.
 *
 * Backends that have no payload should return
 * core::Result::Rejected("payload commands not supported").
 */
using PayloadCmd = std::variant<CmdPayloadReserved>;

}  // namespace swarmkit::commands
