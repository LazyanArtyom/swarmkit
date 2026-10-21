// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "vecr_truth_oracle.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarmkit::testsupport {

VecrTruthOracle::VecrTruthOracle(double edge_length_m, double cell_m, double strip_half_width_m)
    : edge_length_m_(edge_length_m),
      cell_m_(cell_m),
      strip_half_width_m_(strip_half_width_m),
      covered_(static_cast<std::size_t>(std::ceil(edge_length_m / cell_m)), false) {
    if (!std::isfinite(edge_length_m) || !std::isfinite(cell_m) ||
        !std::isfinite(strip_half_width_m) || edge_length_m <= 0 || cell_m <= 0 ||
        strip_half_width_m <= 0)
        throw std::invalid_argument("invalid truth raster");
}

void VecrTruthOracle::ObservePose(const agent::SimulationTruthFrame& truth) {
    if (last_truth_) {
        if (truth.simulation_time_ms <= last_truth_->simulation_time_ms ||
            truth.drone_id != last_truth_->drone_id)
            throw std::invalid_argument("nonmonotonic or foreign simulator truth");
        distance_m_ +=
            std::hypot(truth.east_m - last_truth_->east_m, truth.north_m - last_truth_->north_m);
    }
    last_truth_ = truth;
}

void VecrTruthOracle::ObserveActualCapture(const agent::SimulationTruthFrame& truth,
                                           bool image_acquired, double along_half_length_m,
                                           double cross_half_width_m) {
    if (!image_acquired || truth.failed || !std::isfinite(along_half_length_m) ||
        along_half_length_m <= 0 || !std::isfinite(cross_half_width_m) ||
        std::abs(truth.north_m) + strip_half_width_m_ > cross_half_width_m)
        return;
    for (std::size_t cell = 0; cell < covered_.size(); ++cell) {
        const double center = (static_cast<double>(cell) + 0.5) * cell_m_;
        if (center <= edge_length_m_ && center >= truth.east_m - along_half_length_m &&
            center <= truth.east_m + along_half_length_m)
            covered_[cell] = true;
    }
}

bool VecrTruthOracle::Complete() const {
    return std::ranges::all_of(covered_, [](bool covered) { return covered; });
}

bool VecrTruthOracle::AtB(double arrival_radius_m, double max_speed_mps,
                          double target_altitude_m) const {
    if (!last_truth_ || last_truth_->failed)
        return false;
    return std::hypot(std::hypot(last_truth_->east_m - edge_length_m_, last_truth_->north_m),
                      last_truth_->up_m - target_altitude_m) <= arrival_radius_m &&
           std::hypot(last_truth_->velocity_east_mps, last_truth_->velocity_north_mps) <=
               max_speed_mps;
}

std::size_t VecrTruthOracle::CoveredCells() const {
    return static_cast<std::size_t>(std::ranges::count(covered_, true));
}

double VecrTruthOracle::MissingFraction() const {
    return covered_.empty()
               ? 0.0
               : static_cast<double>(covered_.size() - CoveredCells()) / covered_.size();
}

double VecrTruthOracle::LongestMissingM() const {
    std::size_t longest = 0;
    std::size_t current = 0;
    for (const bool covered : covered_) {
        current = covered ? 0 : current + 1;
        longest = std::max(longest, current);
    }
    return static_cast<double>(longest) * cell_m_;
}

}  // namespace swarmkit::testsupport
