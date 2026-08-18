// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <vector>

#include "swarmkit/agent/backend.h"
#include "swarmkit/core/result.h"

namespace swarmkit::experiment {

struct FaultInjectionConfig;
struct FaultInjectingBackendInstance;

struct FaultInjectionConfig {
    std::uint64_t seed{1};
    double telemetry_loss_probability{};
    double telemetry_duplication_probability{};
    double telemetry_reorder_probability{};
    double position_spike_probability{};
    double estimator_degradation_probability{};
    double invalid_accuracy_probability{};
    double command_failure_probability{};
    double ack_without_motion_probability{};
    std::size_t telemetry_delay_frames{};
    double horizontal_bias_north_m{};
    double horizontal_bias_east_m{};
    double vertical_bias_m{};
    double uniform_horizontal_noise_m{};
    double uniform_vertical_noise_m{};
    double position_spike_m{25.0};
    std::int64_t source_clock_offset_ms{};
    double source_clock_drift_ms_per_frame{};
    double source_clock_uncertainty_ms{};
    bool record_unapplied_decisions{true};

    [[nodiscard]] core::Result Validate() const;
};

struct FaultDecision {
    std::uint64_t decision_sequence{};
    std::uint64_t source_frame_index{};
    std::string drone_id;
    std::string kind;
    bool applied{false};
    std::string detail;

    bool operator==(const FaultDecision&) const = default;
};

class FaultInjectionControl {
   public:
    struct State;
    FaultInjectionControl() = delete;

    /// Release every delayed/reordered frame in deterministic queue order.
    void Flush();
    [[nodiscard]] std::vector<FaultDecision> Decisions() const;
    [[nodiscard]] std::uint64_t Seed() const;

   private:
    explicit FaultInjectionControl(std::shared_ptr<State> state);
    std::shared_ptr<State> state_;
    friend struct FaultInjectingBackendInstance;
    friend std::expected<FaultInjectingBackendInstance, core::Result> MakeFaultInjectingBackend(
        agent::DroneBackendPtr inner, FaultInjectionConfig config);
};

struct FaultInjectingBackendInstance {
    agent::DroneBackendPtr backend;
    std::shared_ptr<FaultInjectionControl> control;
};

/// Decorate any backend with deterministic test faults. Production MAVLink
/// code remains unchanged; randomness exists only in this explicit layer.
[[nodiscard]] std::expected<FaultInjectingBackendInstance, core::Result> MakeFaultInjectingBackend(
    agent::DroneBackendPtr inner, FaultInjectionConfig config);

}  // namespace swarmkit::experiment
