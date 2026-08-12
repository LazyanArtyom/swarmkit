// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/experiment/fault_injection.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <numbers>
#include <optional>
#include <utility>

namespace swarmkit::experiment {
namespace {

constexpr double kEarthRadiusM = 6'371'000.0;

[[nodiscard]] bool ValidProbability(double value) {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

class DeterministicRng {
   public:
    explicit DeterministicRng(std::uint64_t seed) : state_(seed) {}

    [[nodiscard]] std::uint64_t Next() {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t value = state_;
        value = (value ^ (value >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        value = (value ^ (value >> 27U)) * 0x94D049BB133111EBULL;
        return value ^ (value >> 31U);
    }

    [[nodiscard]] double Unit() {
        return static_cast<double>(Next() >> 11U) * (1.0 / 9'007'199'254'740'992.0);
    }

    [[nodiscard]] double Symmetric(double magnitude) {
        return (Unit() * 2.0 - 1.0) * magnitude;
    }

   private:
    std::uint64_t state_;
};

void OffsetSourceTime(core::MeasurementProvenance* provenance, std::int64_t offset_ms,
                      double uncertainty_ms) {
    if (provenance == nullptr) {
        return;
    }
    if (provenance->source_time.timestamp_ms.has_value()) {
        *provenance->source_time.timestamp_ms += offset_ms;
    }
    if (uncertainty_ms > 0.0) {
        provenance->source_time.clock_uncertainty_ms = uncertainty_ms;
        if (provenance->source_time.synchronization == core::ClockSynchronization::kSynchronized) {
            provenance->source_time.synchronization = core::ClockSynchronization::kEstimated;
        }
    }
}

void ApplyPositionOffset(core::TelemetryFrame* frame, double north_m, double east_m,
                         double vertical_m) {
    if (frame == nullptr) {
        return;
    }
    if (frame->validity.position) {
        const double latitude_radians = frame->lat_deg * std::numbers::pi / 180.0;
        frame->lat_deg += north_m / kEarthRadiusM * 180.0 / std::numbers::pi;
        const double longitude_radius = kEarthRadiusM * std::max(0.01, std::cos(latitude_radians));
        frame->lon_deg += east_m / longitude_radius * 180.0 / std::numbers::pi;
    }
    if (frame->validity.relative_altitude) {
        frame->rel_alt_m += static_cast<float>(vertical_m);
    }
    if (frame->validity.absolute_altitude) {
        frame->abs_alt_m += static_cast<float>(vertical_m);
    }
}

}  // namespace

struct FaultInjectionControl::State {
    struct Delivery {
        std::uint64_t due_frame_index{};
        agent::IDroneBackend::TelemetryCallback callback;
        core::TelemetryFrame frame;
    };

    explicit State(FaultInjectionConfig input) : config(input), rng(config.seed) {}

    mutable std::mutex mutex;
    FaultInjectionConfig config;
    DeterministicRng rng;
    std::uint64_t source_frame_index{};
    std::uint64_t decision_sequence{};
    std::uint64_t evidence_sequence{};
    std::vector<FaultDecision> decisions;
    std::deque<Delivery> delayed;
    std::optional<Delivery> reorder_held;
    agent::IDroneBackend::EvidenceCallback evidence_callback;
    bool configuration_emitted{false};
};

namespace {

struct DecisionOutput {
    FaultDecision decision;
    std::optional<agent::BackendEvidenceEvent> evidence;
};

[[nodiscard]] DecisionOutput MakeDecision(FaultInjectionControl::State* state, std::string drone_id,
                                          std::string kind, bool applied, std::string detail = {}) {
    FaultDecision decision{
        .decision_sequence = ++state->decision_sequence,
        .source_frame_index = state->source_frame_index,
        .drone_id = std::move(drone_id),
        .kind = std::move(kind),
        .applied = applied,
        .detail = std::move(detail),
    };
    state->decisions.push_back(decision);
    std::optional<agent::BackendEvidenceEvent> evidence;
    if (applied || state->config.record_unapplied_decisions) {
        evidence = agent::BackendEvidenceEvent{
            .source = "deterministic-fault-injector",
            .kind = decision.kind,
            .source_sequence = ++state->evidence_sequence,
            .random_seed = state->config.seed,
            .attributes =
                {
                    {"applied", applied ? "true" : "false"},
                    {"decision_sequence", std::to_string(decision.decision_sequence)},
                    {"source_frame_index", std::to_string(decision.source_frame_index)},
                    {"drone_id", decision.drone_id},
                    {"detail", decision.detail},
                },
        };
    }
    return {.decision = std::move(decision), .evidence = std::move(evidence)};
}

[[nodiscard]] bool RandomDecision(FaultInjectionControl::State* state, double probability) {
    return state->rng.Unit() < probability;
}

void EmitEvidence(const agent::IDroneBackend::EvidenceCallback& callback,
                  const std::vector<agent::BackendEvidenceEvent>& events) {
    if (!callback) {
        return;
    }
    for (const auto& event : events) {
        callback(event);
    }
}

void Deliver(std::vector<FaultInjectionControl::State::Delivery>&& deliveries) {
    for (auto& delivery : deliveries) {
        delivery.callback(delivery.frame);
    }
}

class FaultInjectingBackend final : public agent::IDroneBackend {
   public:
    FaultInjectingBackend(agent::DroneBackendPtr inner,
                          std::shared_ptr<FaultInjectionControl::State> state)
        : inner_(std::move(inner)), state_(std::move(state)) {}

    core::Result Start() override {
        return inner_->Start();
    }

    core::BackendCommandOutcome Execute(const commands::CommandEnvelope& envelope) override {
        std::vector<agent::BackendEvidenceEvent> evidence_events;
        EvidenceCallback evidence_callback;
        bool fail = false;
        bool acknowledge_without_motion = false;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            fail = RandomDecision(state_.get(), state_->config.command_failure_probability);
            if (auto output =
                    MakeDecision(state_.get(), envelope.context.drone_id, "command-failure", fail,
                                 envelope.context.correlation_id);
                output.evidence.has_value()) {
                evidence_events.push_back(std::move(*output.evidence));
            }
            acknowledge_without_motion =
                !fail &&
                RandomDecision(state_.get(), state_->config.ack_without_motion_probability);
            if (auto output =
                    MakeDecision(state_.get(), envelope.context.drone_id, "ack-without-motion",
                                 acknowledge_without_motion, envelope.context.correlation_id);
                output.evidence.has_value()) {
                evidence_events.push_back(std::move(*output.evidence));
            }
            evidence_callback = state_->evidence_callback;
        }
        EmitEvidence(evidence_callback, evidence_events);
        if (fail) {
            return {
                .result = core::Result::Failed("fault injector rejected backend dispatch"),
                .dispatch_state = core::BackendDispatchState::kFailed,
            };
        }
        if (acknowledge_without_motion) {
            return {
                .result = core::Result::Success(
                    "fault injector synthesized ACK without dispatching physical motion"),
                .dispatch_state = core::BackendDispatchState::kAccepted,
                .protocol_responses = {{.protocol = "fault-injector",
                                        .command_name = "synthetic-ack-without-motion",
                                        .response_expected = true,
                                        .response_received = true,
                                        .response_timed_out = false,
                                        .result_code = 0,
                                        .result_name = "ACCEPTED",
                                        .status_text = "command intentionally not forwarded"}},
            };
        }
        return inner_->Execute(envelope);
    }

    core::Result StartTelemetry(const std::string& drone_id, int rate_hertz,
                                TelemetryCallback callback) override {
        return inner_->StartTelemetry(
            drone_id, rate_hertz,
            [state = state_, callback = std::move(callback)](const core::TelemetryFrame& input) {
                std::vector<State::Delivery> ready;
                std::vector<agent::BackendEvidenceEvent> evidence_events;
                EvidenceCallback evidence_callback;
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    ++state->source_frame_index;
                    while (!state->delayed.empty() &&
                           state->delayed.front().due_frame_index <= state->source_frame_index) {
                        ready.push_back(std::move(state->delayed.front()));
                        state->delayed.pop_front();
                    }

                    core::TelemetryFrame frame = input;
                    const std::string event_drone = input.drone_id;
                    const bool loss =
                        RandomDecision(state.get(), state->config.telemetry_loss_probability);
                    if (auto output =
                            MakeDecision(state.get(), event_drone, "telemetry-loss", loss);
                        output.evidence.has_value()) {
                        evidence_events.push_back(std::move(*output.evidence));
                    }
                    const bool duplicate = RandomDecision(
                        state.get(), state->config.telemetry_duplication_probability);
                    if (auto output = MakeDecision(state.get(), event_drone,
                                                   "telemetry-duplication", duplicate);
                        output.evidence.has_value()) {
                        evidence_events.push_back(std::move(*output.evidence));
                    }
                    const bool reorder =
                        RandomDecision(state.get(), state->config.telemetry_reorder_probability);
                    if (auto output =
                            MakeDecision(state.get(), event_drone, "telemetry-reorder", reorder);
                        output.evidence.has_value()) {
                        evidence_events.push_back(std::move(*output.evidence));
                    }
                    const bool spike =
                        RandomDecision(state.get(), state->config.position_spike_probability);
                    if (auto output =
                            MakeDecision(state.get(), event_drone, "position-spike", spike);
                        output.evidence.has_value()) {
                        evidence_events.push_back(std::move(*output.evidence));
                    }
                    const bool degrade = RandomDecision(
                        state.get(), state->config.estimator_degradation_probability);
                    if (auto output = MakeDecision(state.get(), event_drone,
                                                   "estimator-degradation", degrade);
                        output.evidence.has_value()) {
                        evidence_events.push_back(std::move(*output.evidence));
                    }
                    const bool invalidate_accuracy =
                        RandomDecision(state.get(), state->config.invalid_accuracy_probability);
                    if (auto output = MakeDecision(state.get(), event_drone, "invalid-accuracy",
                                                   invalidate_accuracy);
                        output.evidence.has_value()) {
                        evidence_events.push_back(std::move(*output.evidence));
                    }

                    const double noise_north =
                        state->rng.Symmetric(state->config.uniform_horizontal_noise_m);
                    const double noise_east =
                        state->rng.Symmetric(state->config.uniform_horizontal_noise_m);
                    const double noise_vertical =
                        state->rng.Symmetric(state->config.uniform_vertical_noise_m);
                    const double spike_offset = spike ? state->config.position_spike_m : 0.0;
                    ApplyPositionOffset(
                        &frame, state->config.horizontal_bias_north_m + noise_north + spike_offset,
                        state->config.horizontal_bias_east_m + noise_east,
                        state->config.vertical_bias_m + noise_vertical);
                    if (degrade) {
                        frame.estimator_state = core::EstimatorState::kDegraded;
                        frame.estimator_position_ok = false;
                        frame.estimator_velocity_ok = false;
                        frame.validity.estimator = true;
                    }
                    if (invalidate_accuracy) {
                        frame.accuracy = {};
                    }
                    OffsetSourceTime(&frame.provenance.position,
                                     state->config.source_clock_offset_ms,
                                     state->config.source_clock_uncertainty_ms);
                    OffsetSourceTime(&frame.provenance.velocity,
                                     state->config.source_clock_offset_ms,
                                     state->config.source_clock_uncertainty_ms);
                    OffsetSourceTime(&frame.provenance.accuracy,
                                     state->config.source_clock_offset_ms,
                                     state->config.source_clock_uncertainty_ms);
                    OffsetSourceTime(&frame.provenance.estimator,
                                     state->config.source_clock_offset_ms,
                                     state->config.source_clock_uncertainty_ms);
                    OffsetSourceTime(&frame.provenance.vehicle_state,
                                     state->config.source_clock_offset_ms,
                                     state->config.source_clock_uncertainty_ms);

                    const auto enqueue = [&](State::Delivery delivery) {
                        if (state->config.telemetry_delay_frames == 0) {
                            ready.push_back(std::move(delivery));
                        } else {
                            delivery.due_frame_index =
                                state->source_frame_index + state->config.telemetry_delay_frames;
                            state->delayed.push_back(std::move(delivery));
                        }
                    };

                    if (loss) {
                        if (state->reorder_held.has_value()) {
                            enqueue(std::move(*state->reorder_held));
                            state->reorder_held.reset();
                        }
                    } else {
                        State::Delivery current{
                            .callback = callback,
                            .frame = std::move(frame),
                        };
                        if (state->reorder_held.has_value()) {
                            enqueue(current);
                            if (duplicate) {
                                enqueue(current);
                            }
                            enqueue(std::move(*state->reorder_held));
                            state->reorder_held.reset();
                        } else if (reorder) {
                            state->reorder_held = std::move(current);
                        } else {
                            enqueue(current);
                            if (duplicate) {
                                enqueue(std::move(current));
                            }
                        }
                    }
                    if (state->config.telemetry_delay_frames > 0) {
                        auto output =
                            MakeDecision(state.get(), event_drone, "telemetry-delay", true,
                                         std::to_string(state->config.telemetry_delay_frames));
                        if (output.evidence.has_value()) {
                            evidence_events.push_back(std::move(*output.evidence));
                        }
                    }
                    evidence_callback = state->evidence_callback;
                }
                EmitEvidence(evidence_callback, evidence_events);
                Deliver(std::move(ready));
            });
    }

    core::Result StopTelemetry(const std::string& drone_id) override {
        return inner_->StopTelemetry(drone_id);
    }

    [[nodiscard]] agent::BackendHealth GetHealth() const override {
        return inner_->GetHealth();
    }

    [[nodiscard]] core::BackendCapabilities GetCapabilities() const override {
        return inner_->GetCapabilities();
    }

    void SetEvidenceCallback(const EvidenceCallback& callback) override {
        std::optional<agent::BackendEvidenceEvent> configuration_event;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->evidence_callback = callback;
            if (callback && !state_->configuration_emitted) {
                state_->configuration_emitted = true;
                const auto& config = state_->config;
                configuration_event = agent::BackendEvidenceEvent{
                    .source = "deterministic-fault-injector",
                    .kind = "configuration",
                    .source_sequence = ++state_->evidence_sequence,
                    .random_seed = config.seed,
                    .attributes =
                        {{"telemetry_loss_probability",
                          std::to_string(config.telemetry_loss_probability)},
                         {"telemetry_duplication_probability",
                          std::to_string(config.telemetry_duplication_probability)},
                         {"telemetry_reorder_probability",
                          std::to_string(config.telemetry_reorder_probability)},
                         {"position_spike_probability",
                          std::to_string(config.position_spike_probability)},
                         {"estimator_degradation_probability",
                          std::to_string(config.estimator_degradation_probability)},
                         {"invalid_accuracy_probability",
                          std::to_string(config.invalid_accuracy_probability)},
                         {"command_failure_probability",
                          std::to_string(config.command_failure_probability)},
                         {"ack_without_motion_probability",
                          std::to_string(config.ack_without_motion_probability)},
                         {"telemetry_delay_frames", std::to_string(config.telemetry_delay_frames)},
                         {"horizontal_bias_north_m",
                          std::to_string(config.horizontal_bias_north_m)},
                         {"horizontal_bias_east_m", std::to_string(config.horizontal_bias_east_m)},
                         {"vertical_bias_m", std::to_string(config.vertical_bias_m)},
                         {"uniform_horizontal_noise_m",
                          std::to_string(config.uniform_horizontal_noise_m)},
                         {"uniform_vertical_noise_m",
                          std::to_string(config.uniform_vertical_noise_m)},
                         {"position_spike_m", std::to_string(config.position_spike_m)},
                         {"source_clock_offset_ms", std::to_string(config.source_clock_offset_ms)},
                         {"source_clock_uncertainty_ms",
                          std::to_string(config.source_clock_uncertainty_ms)}},
                };
            }
        }
        if (configuration_event.has_value()) {
            callback(*configuration_event);
        }
        std::weak_ptr<State> weak_state = state_;
        inner_->SetEvidenceCallback([weak_state](const agent::BackendEvidenceEvent& event) {
            if (auto state = weak_state.lock()) {
                EvidenceCallback callback;
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    callback = state->evidence_callback;
                }
                if (callback) {
                    callback(event);
                }
            }
        });
    }

   private:
    using State = FaultInjectionControl::State;
    agent::DroneBackendPtr inner_;
    std::shared_ptr<State> state_;
};

}  // namespace

core::Result FaultInjectionConfig::Validate() const {
    const std::array probabilities{
        telemetry_loss_probability,        telemetry_duplication_probability,
        telemetry_reorder_probability,     position_spike_probability,
        estimator_degradation_probability, invalid_accuracy_probability,
        command_failure_probability,       ack_without_motion_probability,
    };
    if (!std::ranges::all_of(probabilities, ValidProbability)) {
        return core::Result::Rejected("fault probabilities must be finite values in [0, 1]");
    }
    const std::array magnitudes{
        uniform_horizontal_noise_m,
        uniform_vertical_noise_m,
        position_spike_m,
        source_clock_uncertainty_ms,
    };
    if (!std::ranges::all_of(magnitudes,
                             [](double value) { return std::isfinite(value) && value >= 0.0; })) {
        return core::Result::Rejected("fault magnitudes must be finite and non-negative");
    }
    if (!std::isfinite(horizontal_bias_north_m) || !std::isfinite(horizontal_bias_east_m) ||
        !std::isfinite(vertical_bias_m)) {
        return core::Result::Rejected("fault biases must be finite");
    }
    return core::Result::Success();
}

FaultInjectionControl::FaultInjectionControl(std::shared_ptr<State> state)
    : state_(std::move(state)) {}

void FaultInjectionControl::Flush() {
    std::vector<State::Delivery> ready;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        while (!state_->delayed.empty()) {
            ready.push_back(std::move(state_->delayed.front()));
            state_->delayed.pop_front();
        }
        if (state_->reorder_held.has_value()) {
            // Guarded immediately above; move preserves the exact queued frame.
            // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
            ready.push_back(std::move(state_->reorder_held).value());
            state_->reorder_held.reset();
        }
    }
    Deliver(std::move(ready));
}

std::vector<FaultDecision> FaultInjectionControl::Decisions() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->decisions;
}

std::uint64_t FaultInjectionControl::Seed() const {
    return state_->config.seed;
}

std::expected<FaultInjectingBackendInstance, core::Result> MakeFaultInjectingBackend(
    agent::DroneBackendPtr inner, FaultInjectionConfig config) {
    if (!inner) {
        return std::unexpected(core::Result::Rejected("fault injector requires an inner backend"));
    }
    if (const core::Result validation = config.Validate(); !validation.IsOk()) {
        return std::unexpected(validation);
    }
    auto state = std::make_shared<FaultInjectionControl::State>(config);
    return FaultInjectingBackendInstance{
        .backend = std::make_unique<FaultInjectingBackend>(std::move(inner), state),
        .control =
            std::shared_ptr<FaultInjectionControl>(new FaultInjectionControl(std::move(state))),
    };
}

}  // namespace swarmkit::experiment
