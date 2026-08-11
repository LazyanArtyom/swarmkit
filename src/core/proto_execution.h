// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include "swarmkit/core/execution.h"
#include "swarmkit/v1/swarmkit.pb.h"

namespace swarmkit::core::internal {

[[nodiscard]] inline ExecutionContext ToCoreExecutionContext(
    const swarmkit::v1::ExecutionContext& value) {
    return {
        .mission_id = value.mission_id(),
        .mission_revision = value.mission_revision(),
        .model_hash = value.model_hash(),
        .operation_id = value.operation_id(),
        .operation_attempt_revision = value.operation_attempt_revision(),
    };
}

inline void PopulateExecutionContext(const ExecutionContext& value,
                                     swarmkit::v1::ExecutionContext* out) {
    if (out == nullptr) {
        return;
    }
    out->set_mission_id(value.mission_id);
    out->set_mission_revision(value.mission_revision);
    out->set_model_hash(value.model_hash);
    out->set_operation_id(value.operation_id);
    out->set_operation_attempt_revision(value.operation_attempt_revision);
}

[[nodiscard]] inline ExecutionHandle ToCoreExecutionHandle(
    const swarmkit::v1::ExecutionHandle& value) {
    ExecutionHandle out{
        .agent_session_id = value.agent_session_id(),
        .drone_id = value.drone_id(),
        .goal_id = value.goal_id(),
        .goal_revision = value.goal_revision(),
        .physical_attempt_id = value.physical_attempt_id(),
        .physical_attempt_revision = value.physical_attempt_revision(),
        .client_id = value.client_id(),
        .correlation_id = value.correlation_id(),
    };
    if (value.has_context()) {
        out.context = ToCoreExecutionContext(value.context());
    }
    return out;
}

inline void PopulateExecutionHandle(const ExecutionHandle& value,
                                    swarmkit::v1::ExecutionHandle* out) {
    if (out == nullptr) {
        return;
    }
    out->set_agent_session_id(value.agent_session_id);
    out->set_drone_id(value.drone_id);
    out->set_goal_id(value.goal_id);
    out->set_goal_revision(value.goal_revision);
    out->set_physical_attempt_id(value.physical_attempt_id);
    out->set_physical_attempt_revision(value.physical_attempt_revision);
    out->set_client_id(value.client_id);
    out->set_correlation_id(value.correlation_id);
    if (value.context.has_value()) {
        PopulateExecutionContext(*value.context, out->mutable_context());
    }
}

}  // namespace swarmkit::core::internal
