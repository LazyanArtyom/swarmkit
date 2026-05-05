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
/// Swarm commands -- multi-agent coordination metadata.
/// ---------------------------------------------------------------------------

/// @brief Assign a logical role label to the drone within a swarm.
struct CmdSetRole {
    std::string role;  ///< Freeform role string (e.g. "leader", "follower-3").
};

/// @brief Assign the drone a slot in a named geometric formation.
struct CmdSetFormation {
    std::string formation_id;  ///< Identifier of the formation template.
    int slot_index{};          ///< Zero-based position within the formation.
};

/**
 * @brief Variant of all multi-agent coordination commands.
 *
 * Backends that do not participate in a swarm should return
 * core::Result::Rejected("swarm commands not supported").
 */
using SwarmCmd = std::variant<CmdSetRole, CmdSetFormation>;

}  // namespace swarmkit::commands
