// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary

#include "swarmkit/experiment/bootstrap_statistics.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

namespace swarmkit::experiment {

namespace {

struct MethodTotals {
    std::size_t requests{};
    std::size_t accepted{};
    std::size_t false_accepts{};

    [[nodiscard]] double FV() const {
        return accepted > 0 ? static_cast<double>(false_accepts) / static_cast<double>(accepted) : 0.0;
    }
    [[nodiscard]] double Availability() const {
        return requests > 0 ? static_cast<double>(accepted) / static_cast<double>(requests) : 0.0;
    }
    [[nodiscard]] double UAR() const {
        return requests > 0 ? static_cast<double>(false_accepts) / static_cast<double>(requests) : 0.0;
    }
};

ConfidenceInterval MakeCI(std::vector<double>& samples, double point_est) {
    if (samples.empty()) {
        return {point_est, point_est, point_est};
    }
    std::sort(samples.begin(), samples.end());
    const std::size_t n = samples.size();
    const std::size_t idx_025 = static_cast<std::size_t>(std::floor(0.025 * static_cast<double>(n)));
    const std::size_t idx_975 = std::min(n - 1, static_cast<std::size_t>(std::ceil(0.975 * static_cast<double>(n))));
    return {
        .point_estimate = point_est,
        .lower_95 = samples[idx_025],
        .upper_95 = samples[idx_975],
    };
}

}  // namespace

BootstrapAnalysisResult ComputeClusterBootstrap(
    const std::vector<ReplicateRecord>& records,
    std::size_t num_replicates,
    std::size_t bootstrap_iterations,
    std::uint64_t analysis_seed) {

    // Index records by replicate_id
    // replicate_id -> vector of records
    std::unordered_map<std::size_t, std::vector<ReplicateRecord>> by_replicate;
    for (const auto& r : records) {
        by_replicate[r.replicate_id].push_back(r);
    }

    // Compute point estimates across all records
    MethodTotals total_b0, total_b1, total_p;
    for (const auto& r : records) {
        if (r.method == 0) { // B0
            total_b0.requests += r.requests;
            total_b0.accepted += r.accepted;
            total_b0.false_accepts += r.false_accepts;
        } else if (r.method == 1) { // B1
            total_b1.requests += r.requests;
            total_b1.accepted += r.accepted;
            total_b1.false_accepts += r.false_accepts;
        } else if (r.method == 2) { // Proposed
            total_p.requests += r.requests;
            total_p.accepted += r.accepted;
            total_p.false_accepts += r.false_accepts;
        }
    }

    const double pt_b0_fv = total_b0.FV();
    const double pt_b1_fv = total_b1.FV();
    const double pt_p_fv = total_p.FV();
    const double pt_delta_fv = pt_p_fv - pt_b1_fv;

    const double pt_b0_av = total_b0.Availability();
    const double pt_b1_av = total_b1.Availability();
    const double pt_p_av = total_p.Availability();

    const double pt_b0_uar = total_b0.UAR();
    const double pt_b1_uar = total_b1.UAR();
    const double pt_p_uar = total_p.UAR();

    // Bootstrap resampling
    std::mt19937_64 rng(analysis_seed);
    std::uniform_int_distribution<std::size_t> dist(0, num_replicates > 0 ? num_replicates - 1 : 0);

    std::vector<double> boot_b0_fv, boot_b1_fv, boot_p_fv, boot_delta_fv;
    std::vector<double> boot_b0_av, boot_b1_av, boot_p_av;
    std::vector<double> boot_b0_uar, boot_b1_uar, boot_p_uar;

    boot_b0_fv.reserve(bootstrap_iterations);
    boot_b1_fv.reserve(bootstrap_iterations);
    boot_p_fv.reserve(bootstrap_iterations);
    boot_delta_fv.reserve(bootstrap_iterations);

    boot_b0_av.reserve(bootstrap_iterations);
    boot_b1_av.reserve(bootstrap_iterations);
    boot_p_av.reserve(bootstrap_iterations);

    boot_b0_uar.reserve(bootstrap_iterations);
    boot_b1_uar.reserve(bootstrap_iterations);
    boot_p_uar.reserve(bootstrap_iterations);

    for (std::size_t b = 0; b < bootstrap_iterations; ++b) {
        MethodTotals sample_b0, sample_b1, sample_p;

        for (std::size_t i = 0; i < num_replicates; ++i) {
            const std::size_t rep_id = dist(rng);
            const auto it = by_replicate.find(rep_id);
            if (it == by_replicate.end()) continue;

            for (const auto& r : it->second) {
                if (r.method == 0) {
                    sample_b0.requests += r.requests;
                    sample_b0.accepted += r.accepted;
                    sample_b0.false_accepts += r.false_accepts;
                } else if (r.method == 1) {
                    sample_b1.requests += r.requests;
                    sample_b1.accepted += r.accepted;
                    sample_b1.false_accepts += r.false_accepts;
                } else if (r.method == 2) {
                    sample_p.requests += r.requests;
                    sample_p.accepted += r.accepted;
                    sample_p.false_accepts += r.false_accepts;
                }
            }
        }

        const double b0_fv = sample_b0.FV();
        const double b1_fv = sample_b1.FV();
        const double p_fv = sample_p.FV();

        boot_b0_fv.push_back(b0_fv);
        boot_b1_fv.push_back(b1_fv);
        boot_p_fv.push_back(p_fv);
        boot_delta_fv.push_back(p_fv - b1_fv);

        boot_b0_av.push_back(sample_b0.Availability());
        boot_b1_av.push_back(sample_b1.Availability());
        boot_p_av.push_back(sample_p.Availability());

        boot_b0_uar.push_back(sample_b0.UAR());
        boot_b1_uar.push_back(sample_b1.UAR());
        boot_p_uar.push_back(sample_p.UAR());
    }

    BootstrapAnalysisResult result;
    result.proposed_fv = MakeCI(boot_p_fv, pt_p_fv);
    result.b0_fv = MakeCI(boot_b0_fv, pt_b0_fv);
    result.b1_fv = MakeCI(boot_b1_fv, pt_b1_fv);
    result.delta_fv_proposed_vs_b1 = MakeCI(boot_delta_fv, pt_delta_fv);

    result.proposed_availability = MakeCI(boot_p_av, pt_p_av);
    result.b0_availability = MakeCI(boot_b0_av, pt_b0_av);
    result.b1_availability = MakeCI(boot_b1_av, pt_b1_av);

    result.proposed_uar = MakeCI(boot_p_uar, pt_p_uar);
    result.b0_uar = MakeCI(boot_b0_uar, pt_b0_uar);
    result.b1_uar = MakeCI(boot_b1_uar, pt_b1_uar);

    return result;
}

}  // namespace swarmkit::experiment
