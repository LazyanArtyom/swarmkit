// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"

namespace swarmkit::agent {

enum class CommandPreconditionAction : std::uint8_t {
    kExecute,
    kAlreadySatisfied,
    kReject,
};

struct CommandPreconditionDecision {
    CommandPreconditionAction action{CommandPreconditionAction::kExecute};
    core::Result result{core::Result::Success()};
};

[[nodiscard]] CommandPreconditionDecision EvaluateCommandPreconditions(
    const CommandEnvelope& envelope, const BackendHealth& health);

}  // namespace swarmkit::agent
