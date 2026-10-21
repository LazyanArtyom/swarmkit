// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "swarmkit/agent/sim_backend.h"

namespace swarmkit::testsupport {

// Private experiment oracle. No method returns a verifier input or an edge
// status; callers may only use this for held-out scoring and raw traces.
class VecrTruthOracle {
   public:
    VecrTruthOracle(double edge_length_m, double cell_m, double strip_half_width_m);

    void ObservePose(const agent::SimulationTruthFrame& truth);
    void ObserveActualCapture(const agent::SimulationTruthFrame& truth, bool image_acquired,
                              double along_half_length_m, double cross_half_width_m);
    [[nodiscard]] bool Complete() const;
    [[nodiscard]] bool AtB(double arrival_radius_m, double max_speed_mps,
                           double target_altitude_m) const;
    [[nodiscard]] std::size_t CoveredCells() const;
    [[nodiscard]] std::size_t CellCount() const {
        return covered_.size();
    }
    [[nodiscard]] double DistanceM() const {
        return distance_m_;
    }
    [[nodiscard]] double MissingFraction() const;
    [[nodiscard]] double LongestMissingM() const;

   private:
    double edge_length_m_{};
    double cell_m_{};
    double strip_half_width_m_{};
    std::vector<bool> covered_;
    std::optional<agent::SimulationTruthFrame> last_truth_;
    double distance_m_{};
};

}  // namespace swarmkit::testsupport
