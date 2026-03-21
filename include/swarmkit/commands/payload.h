// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

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
