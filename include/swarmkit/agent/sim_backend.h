// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "swarmkit/agent/backend.h"

namespace swarmkit::agent {

struct SimBackendConfig;
struct SimBackendInstance;

enum class SimulationClockMode : std::uint8_t {
    kRealtime,
    kManual,
};

/// Independent simulator state. This type is intentionally unrelated to
/// TelemetryFrame so truth cannot be passed to a controller by accident.
struct SimulationTruthFrame {
    std::string drone_id;
    std::uint64_t truth_sequence{};
    std::int64_t simulation_time_ms{};
    double home_lat_deg{};
    double home_lon_deg{};
    double home_alt_m{};
    double north_m{};
    double east_m{};
    double up_m{};
    double velocity_north_mps{};
    double velocity_east_mps{};
    double velocity_up_mps{};
    bool armed{false};
    bool landed{true};
    bool failed{false};
    std::string mode;

    bool operator==(const SimulationTruthFrame&) const = default;
};

using SimulationTruthObserver = std::function<void(const SimulationTruthFrame&)>;

struct SimBackendConfig {
    SimulationClockMode clock_mode{SimulationClockMode::kRealtime};
    int integration_step_ms{20};
    std::int64_t initial_source_unix_time_ms{1'700'000'000'000LL};
    double home_lat_deg{40.1811};
    double home_lon_deg{44.5136};
    double home_alt_m{0.0};
    float initial_battery_percent{95.0F};
    float max_horizontal_speed_mps{10.0F};
    float max_climb_speed_mps{5.0F};
    float max_descent_speed_mps{3.0F};
    float max_altitude_m{120.0F};
    float default_cruise_speed_mps{4.0F};
    float battery_drain_percent_per_second{0.002F};
    bool stop_at_target{true};
    SimulationTruthObserver truth_observer;

    [[nodiscard]] core::Result Validate() const;
};

class SimBackendControl {
   public:
    struct State;
    SimBackendControl() = delete;

    /// Deterministically advance one vehicle. Valid only in manual mode.
    [[nodiscard]] core::Result Advance(const std::string& drone_id,
                                       std::chrono::milliseconds duration);

    /// Deterministically advance every vehicle in lexical drone-id order.
    [[nodiscard]] core::Result AdvanceAll(std::chrono::milliseconds duration);

    /// Inject or clear a model-level failure without changing telemetry truth.
    void SetFailure(const std::string& drone_id, bool failed, std::string detail = {});

    [[nodiscard]] std::optional<SimulationTruthFrame> Truth(const std::string& drone_id) const;
    [[nodiscard]] std::vector<std::string> DroneIds() const;

   private:
    explicit SimBackendControl(std::shared_ptr<State> state);
    std::shared_ptr<State> state_;
    friend struct SimBackendInstance;
    friend std::expected<SimBackendInstance, core::Result> MakeSimBackend(SimBackendConfig config);
};

struct SimBackendInstance {
    DroneBackendPtr backend;
    std::shared_ptr<SimBackendControl> control;
};

/// Create the single canonical command-responsive simulator backend.
[[nodiscard]] std::expected<SimBackendInstance, core::Result> MakeSimBackend(
    SimBackendConfig config = {});

}  // namespace swarmkit::agent
