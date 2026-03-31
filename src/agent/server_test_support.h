// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <memory>

#include <grpcpp/impl/service_type.h>

#include "swarmkit/agent/server.h"

namespace swarmkit::agent::internal {

[[nodiscard]] std::unique_ptr<grpc::Service> MakeAgentServiceForTesting(const AgentConfig& config,
                                                                        DroneBackendPtr backend);

}  // namespace swarmkit::agent::internal
