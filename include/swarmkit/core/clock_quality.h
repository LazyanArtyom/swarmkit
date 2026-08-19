// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "swarmkit/core/telemetry.h"

namespace swarmkit::core {

/// Generation-time interval [g⁻, g⁺] in the reference clock domain (§7).
///
/// The true physical generation time lies somewhere in this interval.
/// The interval accounts for clock-offset estimation uncertainty ρ.
struct GenerationTimeInterval {
    double lower_ms{};  ///< g⁻ = s - θ̂ - ρ  (earliest possible generation).
    double upper_ms{};  ///< g⁺ = s - θ̂ + ρ  (latest possible generation).

    /// Check causality: evidence must have been generated before t* (§8 Eq.7).
    [[nodiscard]] bool IsCausal(double evaluation_time_ms) const {
        return upper_ms <= evaluation_time_ms;
    }

    /// Conservative maximum elapsed time since evidence generation (§8 Eq.8).
    /// Δ⁺ = t* - g⁻
    [[nodiscard]] double ConservativeElapsed(double evaluation_time_ms) const {
        return evaluation_time_ms - lower_ms;
    }

    /// Interval width: 2ρ (captures clock uncertainty contribution).
    [[nodiscard]] double Width() const {
        return upper_ms - lower_ms;
    }

    bool operator==(const GenerationTimeInterval&) const = default;
};

/// Runtime clock-quality state maintained per agent (§7).
///
/// Tracks the estimated offset from the agent's source clock to the
/// runtime reference clock, plus the deterministic uncertainty radius ρ.
///
/// When deterministic semantics are claimed:
///   |θ_true - θ̂| ≤ ρ
struct ClockQualityState {
    /// Estimated source-to-reference clock offset in milliseconds (θ̂).
    /// Positive means source clock is ahead of reference.
    double offset_estimate_ms{};

    /// Base synchronization uncertainty radius in milliseconds (ρ_sync ≥ 0).
    /// This is the uncertainty at the time of the last clock update.
    double uncertainty_radius_ms{};

    /// Maximum clock drift rate in parts-per-million (ppm).
    /// Used to compute drift budget: ρ_drift = drift_rate_ppm * 1e-3 * elapsed_ms.
    /// A value of 0.0 means no drift model (base uncertainty only).
    double max_drift_rate_ppm{0.0};

    /// Clock domain of the source.
    ClockDomain source_domain{ClockDomain::kUnknown};

    /// Synchronization quality assessment.
    ClockSynchronization synchronization{ClockSynchronization::kUnknown};

    /// When this state was last updated (reference clock, Unix ms).
    std::int64_t last_update_ms{};

    /// Whether the uncertainty radius has deterministic semantics.
    /// When false, the radius may be a statistical estimate and the
    /// deterministic soundness theorem (§12) does not apply.
    bool deterministic_bound{false};

    /// Agent incarnation ID this clock model is bound to (§6).
    /// On an incarnation change, the previous clock model is invalid
    /// until synchronization is re-established.
    std::string agent_incarnation_id;

    /// Clock model version identifier for certificate binding.
    std::string clock_model_version{"clock-v1"};

    /// Compute effective uncertainty radius including bounded drift (§7).
    ///
    /// ρ_eff = ρ_sync + ρ_drift
    /// where ρ_drift = max_drift_rate_ppm * 1e-3 * (reference_time_ms - last_update_ms)
    ///
    /// @param reference_time_ms  The reference time at which to evaluate.
    /// @return Effective uncertainty radius in milliseconds.
    [[nodiscard]] double ComputeEffectiveUncertainty(
        double reference_time_ms) const {
        double rho = uncertainty_radius_ms;
        if (max_drift_rate_ppm > 0.0 && last_update_ms > 0) {
            const double elapsed = reference_time_ms -
                                   static_cast<double>(last_update_ms);
            if (elapsed > 0.0) {
                // drift_rate_ppm * 1e-6 (ppm→ratio) * elapsed_ms
                rho += max_drift_rate_ppm * 1e-6 * elapsed;
            }
        }
        return rho;
    }

    /// Compute the generation-time interval for a source timestamp (§7).
    /// Uses effective uncertainty including drift budget.
    ///
    /// g⁻ = s - θ̂ - ρ_eff
    /// g⁺ = s - θ̂ + ρ_eff
    ///
    /// @param source_time_ms  Source timestamp s in the source clock domain.
    /// @return Generation-time interval in the reference domain.
    [[nodiscard]] GenerationTimeInterval ComputeGenerationInterval(
        double source_time_ms) const {
        const double rho = ComputeEffectiveUncertainty(source_time_ms);
        const double reference_time = source_time_ms - offset_estimate_ms;
        return {
            .lower_ms = reference_time - rho,
            .upper_ms = reference_time + rho,
        };
    }

    /// Whether this clock state has been initialized with a real estimate.
    [[nodiscard]] bool IsValid() const {
        return std::isfinite(offset_estimate_ms) &&
               std::isfinite(uncertainty_radius_ms) &&
               uncertainty_radius_ms >= 0.0 &&
               synchronization != ClockSynchronization::kUnknown &&
               source_domain != ClockDomain::kUnknown &&
               last_update_ms > 0;
    }

    bool operator==(const ClockQualityState&) const = default;
};

/// Compute a generation-time interval from per-sample clock evidence (§7).
///
/// This function is used when clock quality is carried per-sample in
/// TimestampEvidence rather than maintained as persistent ClockQualityState.
///
/// @param source_time  Per-sample timestamp evidence from telemetry.
/// @param clock_state  Runtime-maintained clock quality for the agent.
/// @return Generation interval, or nullopt if the source time is absent
///         or the clock state is insufficient.
[[nodiscard]] inline std::optional<GenerationTimeInterval>
ComputeGenerationInterval(const TimestampEvidence& source_time,
                          const ClockQualityState& clock_state) {
    if (!source_time.timestamp_ms.has_value()) return std::nullopt;
    if (!clock_state.IsValid()) return std::nullopt;

    const double s = static_cast<double>(*source_time.timestamp_ms);
    if (!std::isfinite(s)) return std::nullopt;

    return clock_state.ComputeGenerationInterval(s);
}

/// Compute a generation-time interval using only per-sample evidence.
///
/// This is used when per-sample clock uncertainty is available in
/// TimestampEvidence. If clock_uncertainty_ms is absent or negative/NaN,
/// this returns nullopt (missing clock evidence is NOT zero error).
///
/// @param source_time  Per-sample timestamp evidence.
/// @return Generation interval, or nullopt if timestamp or clock uncertainty is absent.
[[nodiscard]] inline std::optional<GenerationTimeInterval>
ComputeGenerationIntervalFromSample(const TimestampEvidence& source_time) {
    if (!source_time.timestamp_ms.has_value()) return std::nullopt;
    if (!source_time.clock_uncertainty_ms.has_value()) return std::nullopt;

    const double rho = *source_time.clock_uncertainty_ms;
    if (!std::isfinite(rho) || rho < 0.0) return std::nullopt;

    const double s = static_cast<double>(*source_time.timestamp_ms);
    if (!std::isfinite(s)) return std::nullopt;

    return GenerationTimeInterval{
        .lower_ms = s - rho,
        .upper_ms = s + rho,
    };
}

/// Compute the conservative propagated position uncertainty at evaluation
/// time t* (§9 Eq.10).
///
///   ε_p(t*) = e_p + V_max · Δ⁺(t*)
///
/// @param observation_uncertainty  Estimator error bound at observation time (e_p).
/// @param max_speed_mps            Maximum physical speed V_max in m/s.
/// @param conservative_elapsed_ms  Conservative elapsed time Δ⁺ in milliseconds.
/// @return Propagated position uncertainty radius in metres.
[[nodiscard]] inline double ComputePropagatedUncertainty(
    double observation_uncertainty, double max_speed_mps,
    double conservative_elapsed_ms) {
    return observation_uncertainty +
           max_speed_mps * (conservative_elapsed_ms / 1000.0);
}

}  // namespace swarmkit::core
