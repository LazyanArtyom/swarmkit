// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "swarmkit/agent/backend.h"

namespace swarmkit::experiment {

struct ScriptedBackendConfig;
struct ScriptedBackendInstance;

struct ScriptedCommandStep {
    std::optional<std::string> expected_correlation_id;
    core::BackendCommandOutcome outcome;
};

struct ScriptedBackendConfig {
    agent::BackendHealth initial_health;
    core::BackendCapabilities capabilities;
    bool reject_unexpected_commands{true};
};

class ScriptedBackendControl {
   public:
    struct State;
    ScriptedBackendControl() = delete;

    void QueueCommandStep(ScriptedCommandStep step);
    void SetHealth(agent::BackendHealth health);
    void SetTelemetryStartResult(core::Result result);
    void SetTelemetryStopResult(core::Result result);

    /// Emit exactly one backend frame synchronously. The Agent normalization
    /// layer, not this source, assigns session, sequence, and receive times.
    [[nodiscard]] core::Result EmitTelemetry(const std::string& drone_id,
                                             const core::TelemetryFrame& frame) const;

    [[nodiscard]] std::vector<commands::CommandEnvelope> Commands() const;
    [[nodiscard]] std::size_t PendingCommandSteps() const;
    [[nodiscard]] bool HasTelemetrySubscriber(const std::string& drone_id) const;

   private:
    explicit ScriptedBackendControl(std::shared_ptr<State> state);
    std::shared_ptr<State> state_;
    friend struct ScriptedBackendInstance;
    friend ScriptedBackendInstance MakeScriptedBackend(ScriptedBackendConfig config);
};

struct ScriptedBackendInstance {
    agent::DroneBackendPtr backend;
    std::shared_ptr<ScriptedBackendControl> control;
};

[[nodiscard]] ScriptedBackendInstance MakeScriptedBackend(ScriptedBackendConfig config = {});

}  // namespace swarmkit::experiment
