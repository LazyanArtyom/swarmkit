// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "swarmkit/core/telemetry.h"
#include "swarmkit/edge/vecr.h"

namespace swarmkit::testsupport::v5 {

// Accepts measured telemetry only. It has no simulator/control/truth dependency.
class CameraEstimator {
   public:
    CameraEstimator(double lat, double lon, double along_half, double cross_half)
        : lat_(lat), lon_(lon), along_(along_half), cross_(cross_half) {}
    std::optional<edge::NadirCaptureInput> Estimate(const core::TelemetryFrame& measured,
                                                    std::int64_t time_ms, bool frame_success,
                                                    bool aligned,
                                                    const std::string& calibration) const {
        if (!measured.HasPosition() || measured.position_frame != core::CoordinateFrame::kWgs84 ||
            !measured.HasRelativeAltitude() || !measured.accuracy.horizontal_position ||
            !measured.accuracy.vertical_position || !measured.validity.estimator ||
            measured.estimator_state != core::EstimatorState::kHealthy ||
            measured.accuracy.horizontal_position->descriptor.semantics !=
                core::UncertaintySemantics::kDeterministicHardBound ||
            measured.accuracy.vertical_position->descriptor.semantics !=
                core::UncertaintySemantics::kDeterministicHardBound)
            return std::nullopt;
        edge::NadirCaptureInput input;
        input.monotonic_ms = time_ms;
        input.camera_center = edge::Wgs84ToLocal(lat_, lon_, measured.lat_deg, measured.lon_deg);
        input.altitude_above_ground_m = measured.rel_alt_m;
        input.horizontal_error_hard_bound_m = measured.accuracy.horizontal_position->value;
        input.altitude_error_hard_bound_m = measured.accuracy.vertical_position->value;
        // Fixed calibration envelope, not recalculated from actual altitude or footprint.
        // Actual optics are 0..2% wider than this conservative calibration.
        input.along_fov_deg = 2 * std::atan(along_ / 10) * 180 / std::numbers::pi;
        input.cross_fov_deg = 2 * std::atan(cross_ / 10) * 180 / std::numbers::pi;
        input.calibration_id = calibration;
        input.capture_succeeded = frame_success;
        input.image_quality_established = frame_success;
        input.nadir_route_alignment_established = aligned;
        input.flat_ground_established = true;
        return input;
    }

   private:
    double lat_, lon_, along_, cross_;
};

// Independent implementation: no VECR IntervalSet, projection, or evidence builder.
// Full-strip x ranges are intersections of a convex actual footprint at both strip
// boundaries. Multiple partial-width images are deliberately not mosaicked.
class ExactStripOracle {
   public:
    explicit ExactStripOracle(double length, double strip = 4) : length_(length), strip_(strip) {}
    std::optional<std::pair<double, double>> Footprint(double east, double north, double along,
                                                       double cross, double yaw_deg) const {
        const double angle = yaw_deg * std::numbers::pi / 180;
        std::vector<std::pair<double, double>> corners;
        for (const auto [x, y] : std::vector<std::pair<double, double>>{
                 {-along, -cross}, {along, -cross}, {along, cross}, {-along, cross}})
            corners.emplace_back(east + x * std::cos(angle) - y * std::sin(angle),
                                 north + x * std::sin(angle) + y * std::cos(angle));
        double low = 0, high = length_;
        for (const double boundary : {-strip_, strip_}) {
            std::vector<double> intersections;
            for (std::size_t i = 0; i < 4; ++i) {
                const auto [x1, y1] = corners[i];
                const auto [x2, y2] = corners[(i + 1) % 4];
                if (boundary < std::min(y1, y2) - 1e-10 || boundary > std::max(y1, y2) + 1e-10)
                    continue;
                if (std::abs(y2 - y1) < 1e-10) {
                    intersections.push_back(x1);
                    intersections.push_back(x2);
                } else
                    intersections.push_back(x1 + (x2 - x1) * (boundary - y1) / (y2 - y1));
            }
            if (intersections.empty())
                return std::nullopt;
            low = std::max(low, *std::min_element(intersections.begin(), intersections.end()));
            high = std::min(high, *std::max_element(intersections.begin(), intersections.end()));
        }
        if (high <= low)
            return std::nullopt;
        return std::pair{low, high};
    }
    void Add(std::optional<std::pair<double, double>> interval) {
        if (!interval)
            return;
        ranges_.push_back(*interval);
        std::sort(ranges_.begin(), ranges_.end());
        std::vector<std::pair<double, double>> merged;
        for (const auto& item : ranges_) {
            if (merged.empty() || item.first > merged.back().second + 1e-9)
                merged.push_back(item);
            else
                merged.back().second = std::max(merged.back().second, item.second);
        }
        ranges_ = std::move(merged);
    }
    bool Contains(double a, double b) const {
        if (b <= a + 1e-8)
            return true;
        for (const auto& interval : ranges_)
            if (interval.first <= a + 1e-7 && interval.second >= b - 1e-7)
                return true;
        return false;
    }
    double Covered() const {
        double sum = 0;
        for (const auto& x : ranges_)
            sum += x.second - x.first;
        return sum;
    }
    bool Complete() const {
        return Contains(0, length_);
    }
    double RasterFraction(double cell, bool centers) const {
        int total = 0, covered = 0;
        for (double a = 0; a < length_ - 1e-9; a += cell) {
            const double b = std::min(a + cell, length_);
            ++total;
            const double p = (a + b) / 2;
            if (centers ? Contains(p, p + 1e-6) : Contains(a, b))
                ++covered;
        }
        return static_cast<double>(covered) / total;
    }

   private:
    double length_, strip_;
    std::vector<std::pair<double, double>> ranges_;
};
}  // namespace swarmkit::testsupport::v5
