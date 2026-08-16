// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/experiment/scripted_backend.h"

#include <deque>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace swarmkit::experiment {

struct ScriptedBackendControl::State {
    struct Stream {
        int rate_hertz{};
        agent::IDroneBackend::TelemetryCallback callback;
    };

    mutable std::mutex mutex;
    ScriptedBackendConfig config;
    agent::BackendHealth health;
    core::Result start_result{core::Result::Success()};
    core::Result stop_result{core::Result::Success()};
    std::deque<ScriptedCommandStep> command_steps;
    std::vector<commands::CommandEnvelope> commands;
    std::unordered_map<std::string, Stream> streams;
    agent::IDroneBackend::EvidenceCallback evidence_callback;
    std::uint64_t evidence_sequence{};
};

namespace {

class ScriptedBackend final : public agent::IDroneBackend {
   public:
    explicit ScriptedBackend(std::shared_ptr<ScriptedBackendControl::State> state)
        : state_(std::move(state)) {}

    core::BackendCommandOutcome Execute(const commands::CommandEnvelope& envelope) override {
        EvidenceCallback evidence_callback;
        agent::BackendEvidenceEvent evidence;
        core::BackendCommandOutcome outcome;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->commands.push_back(envelope);
            evidence_callback = state_->evidence_callback;
            evidence.source = "scripted-backend";
            evidence.source_sequence = ++state_->evidence_sequence;
            evidence.attributes.emplace("correlation_id", envelope.context.correlation_id);
            if (state_->command_steps.empty()) {
                evidence.kind = "unexpected-command";
                if (state_->config.reject_unexpected_commands) {
                    outcome = {
                        .result = core::Result::Rejected("unexpected scripted backend command"),
                        .dispatch_state = core::BackendDispatchState::kRejected,
                    };
                } else {
                    outcome = {
                        .result = core::Result::Success("scripted backend accepted command"),
                        .dispatch_state = core::BackendDispatchState::kAccepted,
                    };
                }
            } else {
                ScriptedCommandStep step = std::move(state_->command_steps.front());
                state_->command_steps.pop_front();
                if (step.expected_correlation_id.has_value() &&
                    *step.expected_correlation_id != envelope.context.correlation_id) {
                    evidence.kind = "command-script-mismatch";
                    evidence.attributes.emplace("expected_correlation_id",
                                                *step.expected_correlation_id);
                    outcome = {
                        .result = core::Result::Failed(
                            "scripted command correlation does not match scenario"),
                        .dispatch_state = core::BackendDispatchState::kFailed,
                    };
                } else {
                    evidence.kind = "command-step-consumed";
                    outcome = std::move(step.outcome);
                }
            }
        }
        if (evidence_callback) {
            evidence_callback(evidence);
        }
        return outcome;
    }

    core::Result StartTelemetry(const std::string& drone_id, int rate_hertz,
                                TelemetryCallback callback) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!callback) {
            return core::Result::Rejected("scripted telemetry callback must not be empty");
        }
        if (rate_hertz <= 0) {
            return core::Result::Rejected("scripted telemetry rate must be greater than zero");
        }
        if (!state_->start_result.IsOk()) {
            return state_->start_result;
        }
        if (state_->streams.contains(drone_id)) {
            return core::Result::Rejected("scripted telemetry already active");
        }
        state_->streams.emplace(
            drone_id, ScriptedBackendControl::State::Stream{.rate_hertz = rate_hertz,
                                                            .callback = std::move(callback)});
        return core::Result::Success();
    }

    core::Result StopTelemetry(const std::string& drone_id) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->stop_result.IsOk()) {
            return state_->stop_result;
        }
        state_->streams.erase(drone_id);
        return core::Result::Success();
    }

    [[nodiscard]] agent::BackendHealth GetHealth() const override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->health;
    }

    [[nodiscard]] core::BackendCapabilities GetCapabilities() const override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->config.capabilities;
    }

    void SetEvidenceCallback(const EvidenceCallback& callback) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->evidence_callback = callback;
    }

   private:
    std::shared_ptr<ScriptedBackendControl::State> state_;
};

}  // namespace

ScriptedBackendControl::ScriptedBackendControl(std::shared_ptr<State> state)
    : state_(std::move(state)) {}

void ScriptedBackendControl::QueueCommandStep(ScriptedCommandStep step) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->command_steps.push_back(std::move(step));
}

void ScriptedBackendControl::SetHealth(agent::BackendHealth health) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->health = std::move(health);
}

void ScriptedBackendControl::SetTelemetryStartResult(core::Result result) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->start_result = std::move(result);
}

void ScriptedBackendControl::SetTelemetryStopResult(core::Result result) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stop_result = std::move(result);
}

core::Result ScriptedBackendControl::EmitTelemetry(const std::string& drone_id,
                                                   const core::TelemetryFrame& frame) const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto iter = state_->streams.find(drone_id);
    if (iter == state_->streams.end()) {
        return core::Result::Rejected("no scripted telemetry subscriber for drone '" + drone_id +
                                      "'");
    }
    iter->second.callback(frame);
    return core::Result::Success();
}

std::vector<commands::CommandEnvelope> ScriptedBackendControl::Commands() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->commands;
}

std::size_t ScriptedBackendControl::PendingCommandSteps() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->command_steps.size();
}

bool ScriptedBackendControl::HasTelemetrySubscriber(const std::string& drone_id) const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->streams.contains(drone_id);
}

ScriptedBackendInstance MakeScriptedBackend(ScriptedBackendConfig config) {
    auto state = std::make_shared<ScriptedBackendControl::State>();
    state->config = std::move(config);
    state->health = state->config.initial_health;
    return {
        .backend = std::make_unique<ScriptedBackend>(state),
        .control = std::shared_ptr<ScriptedBackendControl>(new ScriptedBackendControl(state)),
    };
}

}  // namespace swarmkit::experiment
