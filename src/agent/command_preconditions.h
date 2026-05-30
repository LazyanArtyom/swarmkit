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

struct AutonomousReadinessRequirements {
    bool require_armed{true};
    bool require_gps{true};
    bool require_ekf{true};
};

[[nodiscard]] core::Result ValidateAutonomousReadiness(
    const BackendHealth& health, std::string_view operation, bool allow_unsafe_bench_commands,
    AutonomousReadinessRequirements requirements = {});

[[nodiscard]] CommandPreconditionDecision EvaluateCommandPreconditions(
    const CommandEnvelope& envelope, const BackendHealth& health,
    bool allow_unsafe_bench_commands = false);

}  // namespace swarmkit::agent
