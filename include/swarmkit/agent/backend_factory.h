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

class BackendRegistry {
   public:
    [[nodiscard]] core::Result Register(std::string name, BackendCreator creator);
    [[nodiscard]] std::expected<DroneBackendPtr, core::Result> Create(
        const BackendFactoryRequest& request) const;
    [[nodiscard]] std::vector<std::string> Names() const;

   private:
    std::unordered_map<std::string, BackendCreator> creators_;
};

void RegisterBuiltinBackends(BackendRegistry* registry);

}  // namespace swarmkit::agent
