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

    /// Deterministic uncertainty radius in milliseconds (ρ ≥ 0).
    /// The true offset lies within [θ̂ - ρ, θ̂ + ρ].
    double uncertainty_radius_ms{};

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

    /// Compute the generation-time interval for a source timestamp (§7).
    ///
    /// g⁻ = s - θ̂ - ρ
    /// g⁺ = s - θ̂ + ρ
    ///
    /// @param source_time_ms  Source timestamp s in the source clock domain.
    /// @return Generation-time interval in the reference domain.
    [[nodiscard]] GenerationTimeInterval ComputeGenerationInterval(
        double source_time_ms) const {
        const double reference_time = source_time_ms - offset_estimate_ms;
        return {
            .lower_ms = reference_time - uncertainty_radius_ms,
            .upper_ms = reference_time + uncertainty_radius_ms,
        };
    }

    /// Whether this clock state has been initialized with a real estimate.
    [[nodiscard]] bool IsValid() const {
        return synchronization != ClockSynchronization::kUnknown &&
               source_domain != ClockDomain::kUnknown &&
               uncertainty_radius_ms >= 0.0 && last_update_ms > 0;
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

    return clock_state.ComputeGenerationInterval(
        static_cast<double>(*source_time.timestamp_ms));
}

/// Compute a generation-time interval using only per-sample evidence.
///
/// This is a simpler fallback when no persistent ClockQualityState is
/// maintained.  It uses the per-sample clock_uncertainty_ms directly.
/// If clock_uncertainty_ms is absent, the interval degenerates to a point
/// (uncertainty = 0), which is explicit about unknown error rather than
/// silently treating it as zero.
///
/// @param source_time  Per-sample timestamp evidence.
/// @return Generation interval, or nullopt if timestamp is absent.
[[nodiscard]] inline std::optional<GenerationTimeInterval>
ComputeGenerationIntervalFromSample(const TimestampEvidence& source_time) {
    if (!source_time.timestamp_ms.has_value()) return std::nullopt;

    const double s = static_cast<double>(*source_time.timestamp_ms);
    // Per-sample clock uncertainty, if provided.  When absent, we do NOT
    // default to zero — the caller must handle the missing uncertainty.
    const double rho = source_time.clock_uncertainty_ms.value_or(0.0);

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
