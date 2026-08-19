// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace swarmkit::experiment {

/// Per-replicate, per-scenario, per-method measurement record (§28, P2.1).
struct ReplicateRecord {
    std::size_t replicate_id{};
    std::uint64_t motion_seed{};
    std::uint64_t fault_seed{};
    std::uint8_t scenario_id{};
    std::uint8_t method{};

    std::size_t requests{};
    std::size_t accepted{};
    std::size_t false_accepts{};
    std::size_t true_accepts{};
    std::size_t true_rejects{};
    std::size_t false_rejects{};
    std::size_t enclosures_tested{};
    std::size_t containment_failures{};

    [[nodiscard]] double FalseValidRate() const {
        return accepted > 0 ? static_cast<double>(false_accepts) / static_cast<double>(accepted) : 0.0;
    }
    [[nodiscard]] double Availability() const {
        return requests > 0 ? static_cast<double>(accepted) / static_cast<double>(requests) : 0.0;
    }
    [[nodiscard]] double UnsafeAcceptanceRate() const {
        return requests > 0 ? static_cast<double>(false_accepts) / static_cast<double>(requests) : 0.0;
    }
};

/// Confidence interval summary [lower_95, point_estimate, upper_95].
struct ConfidenceInterval {
    double point_estimate{};
    double lower_95{};
    double upper_95{};
};

/// Bootstrap statistics results (§31, P2.4).
struct BootstrapAnalysisResult {
    ConfidenceInterval proposed_fv;
    ConfidenceInterval b0_fv;
    ConfidenceInterval b1_fv;

    ConfidenceInterval proposed_availability;
    ConfidenceInterval b0_availability;
    ConfidenceInterval b1_availability;

    ConfidenceInterval proposed_uar;
    ConfidenceInterval b0_uar;
    ConfidenceInterval b1_uar;

    /// Paired comparison: Δ_FV = FV_Proposed - FV_B1
    ConfidenceInterval delta_fv_proposed_vs_b1;
};

/// Compute run-cluster bootstrap confidence intervals (§31, P2.4).
///
/// @param records         All per-replicate records across runs, scenarios, methods.
/// @param num_replicates  Number of distinct mission replicates R.
/// @param bootstrap_iterations Number of bootstrap samples B (default 10,000).
/// @param analysis_seed   Deterministic RNG seed for bootstrap.
[[nodiscard]] BootstrapAnalysisResult ComputeClusterBootstrap(
    const std::vector<ReplicateRecord>& records,
    std::size_t num_replicates,
    std::size_t bootstrap_iterations = 10000,
    std::uint64_t analysis_seed = 123456789ULL);

}  // namespace swarmkit::experiment
