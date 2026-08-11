// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace swarmkit::core {

/// Controller-supplied identity for an algorithm-neutral unit of work.
/// When present, the context is complete; absence is represented by
/// std::optional at the call site rather than partially empty fields.
struct ExecutionContext {
    std::string mission_id;
    std::uint64_t mission_revision{};
    std::string model_hash;
    std::string operation_id;
    std::uint64_t operation_attempt_revision{};

    [[nodiscard]] bool IsComplete() const {
        return !mission_id.empty() && mission_revision > 0 && !model_hash.empty() &&
               !operation_id.empty() && operation_attempt_revision > 0;
    }

    bool operator==(const ExecutionContext&) const = default;
};

/// Agent-finalized identity for exactly one accepted physical goal attempt.
/// The revision is ordered only within agent_session_id.
struct ExecutionHandle {
    std::string agent_session_id;
    std::string drone_id;
    std::string goal_id;
    std::uint64_t goal_revision{};
    std::string physical_attempt_id;
    std::uint64_t physical_attempt_revision{};
    std::string client_id;
    std::string correlation_id;
    std::optional<ExecutionContext> context;

    [[nodiscard]] bool IsComplete() const {
        return !agent_session_id.empty() && !drone_id.empty() && !goal_id.empty() &&
               goal_revision > 0 && !physical_attempt_id.empty() && physical_attempt_revision > 0 &&
               !client_id.empty() && !correlation_id.empty() &&
               (!context.has_value() || context->IsComplete());
    }

    bool operator==(const ExecutionHandle&) const = default;
};

}  // namespace swarmkit::core
