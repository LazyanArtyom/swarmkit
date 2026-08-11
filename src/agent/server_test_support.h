// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <memory>

#include "runtime_providers.h"
#include "swarmkit/agent/server.h"
#include "swarmkit/v1/swarmkit.grpc.pb.h"

namespace swarmkit::agent::internal {

struct AgentServiceBundle {
    std::unique_ptr<swarmkit::v1::AgentService::Service> agent_service;
    std::unique_ptr<swarmkit::v1::DataService::Service> data_service;
};

[[nodiscard]] std::unique_ptr<swarmkit::v1::AgentService::Service> MakeAgentServiceForTesting(
    const AgentConfig& config, DroneBackendPtr backend);

[[nodiscard]] std::unique_ptr<swarmkit::v1::AgentService::Service> MakeAgentServiceForTesting(
    const AgentConfig& config, DroneBackendPtr backend, RuntimeProviders providers);

[[nodiscard]] std::unique_ptr<AgentServiceBundle> MakeAgentServicesForTesting(
    const AgentConfig& config, DroneBackendPtr backend);

[[nodiscard]] std::unique_ptr<AgentServiceBundle> MakeAgentServicesForTesting(
    const AgentConfig& config, DroneBackendPtr backend, RuntimeProviders providers);

}  // namespace swarmkit::agent::internal
