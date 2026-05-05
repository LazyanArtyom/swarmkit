// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <string>
#include <unordered_map>
#include <variant>

namespace swarmkit::commands {

/**
 * @brief Backend-specific command escape hatch.
 *
 * Use typed core commands first.  BackendCommand is intentionally namespaced
 * so advanced protocol features can be exposed without leaking one protocol's
 * details into the generic command model.
 */
struct CmdBackendCommand {
    std::string backend_namespace;  ///< Example: "mavlink", "dji", "vendor-x".
    std::string name;               ///< Backend-defined command name.
    std::unordered_map<std::string, std::string> params;
};

using BackendCmd = std::variant<CmdBackendCommand>;

}  // namespace swarmkit::commands
