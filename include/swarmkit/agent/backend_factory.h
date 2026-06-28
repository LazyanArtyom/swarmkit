// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <expected>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent {

using BackendCreator =
    std::function<std::expected<DroneBackendPtr, core::Result>(const BackendFactoryRequest&)>;

/// @brief Registry of named drone backend factories.
///
/// Applications embedding the agent library can register custom backends and
/// select them by name from CLI/YAML backend configuration.
class BackendRegistry {
   public:
    /// @brief Register or replace a backend factory.
    /// @param name Non-empty backend name, for example "sim" or "mavlink".
    /// @param creator Callable that creates one backend instance for a request.
    /// @return Success on registration, Rejected if the name or creator is invalid.
    [[nodiscard]] core::Result Register(std::string name, BackendCreator creator);

    /// @brief Create a backend from a factory request.
    /// @param request Backend name plus string key/value options.
    /// @return Newly created backend, or a typed error when the name is unknown or creation fails.
    [[nodiscard]] std::expected<DroneBackendPtr, core::Result> Create(
        const BackendFactoryRequest& request) const;

    /// @brief List registered backend names.
    [[nodiscard]] std::vector<std::string> Names() const;

   private:
    std::unordered_map<std::string, BackendCreator> creators_;
};

/// @brief Register the built-in "sim" and "mavlink" backends.
/// @param registry Registry to update; ignored when null.
void RegisterBuiltinBackends(BackendRegistry* registry);

}  // namespace swarmkit::agent
