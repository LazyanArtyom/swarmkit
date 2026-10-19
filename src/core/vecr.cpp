// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#include "swarmkit/edge/vecr.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace swarmkit::edge {
namespace {
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kGeometryToleranceM = 1e-6;
constexpr std::int64_t kArrivalDwellMs = 200;
constexpr std::int64_t kArrivalEvidenceFreshnessMs = 1000;

[[nodiscard]] double Distance(Point a, Point b) {
    return std::hypot(a.east_m - b.east_m, a.north_m - b.north_m);
}
[[nodiscard]] bool Finite(Point p) {
    return std::isfinite(p.east_m) && std::isfinite(p.north_m);
}
[[nodiscard]] bool Terminal(Phase phase) {
    return phase == Phase::kFulfilled || phase == Phase::kInterrupted;
}
}  // namespace

double Length(const Route& route) {
    if (route.points.size() < 2) return 0.0;
    double length = 0.0;
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        if (!Finite(route.points[i - 1]) || !Finite(route.points[i])) return 0.0;
        const double segment = Distance(route.points[i - 1], route.points[i]);
        if (segment <= kGeometryToleranceM) return 0.0;
        length += segment;
    }
    return std::isfinite(length) ? length : 0.0;
}

std::optional<Projection> Project(const Route& route, Point point) {
    if (!Finite(point) || Length(route) <= 0.0) return std::nullopt;
    Projection best{.cross_track_m = std::numeric_limits<double>::infinity()};
    double offset = 0.0;
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        const Point a = route.points[i - 1];
        const Point b = route.points[i];
        const double dx = b.east_m - a.east_m;
        const double dy = b.north_m - a.north_m;
        const double segment = std::hypot(dx, dy);
        const double t = std::clamp(((point.east_m - a.east_m) * dx +
                                     (point.north_m - a.north_m) * dy) /
                                        (segment * segment),
                                    0.0, 1.0);
        const Point projected{a.east_m + t * dx, a.north_m + t * dy};
        const double cross = Distance(point, projected);
        const double s = offset + t * segment;
        if (cross + kGeometryToleranceM < best.cross_track_m) {
            best = {.arclength_m = s, .cross_track_m = cross, .segment = i - 1};
        } else if (std::abs(cross - best.cross_track_m) <= kGeometryToleranceM &&
                   std::abs(s - best.arclength_m) > kGeometryToleranceM) {
            best.ambiguous = true;
        }
        offset += segment;
    }
    return best;
}

std::optional<CaptureEvidence> BuildNadirStripEvidence(const Route& route,
                                                       double required_strip_half_width_m,
                                                       const NadirCaptureInput& input) {
    if (route.points.size() != 2 || input.mission_id.empty() ||
        input.mission_revision == 0 || input.edge_id.empty() ||
        input.attempt_id.empty() || input.capture_id.empty() ||
        input.calibration_id.empty() || input.monotonic_ms < 0 ||
        !input.capture_succeeded || !input.image_quality_established ||
        !input.nadir_route_alignment_established || !input.flat_ground_established ||
        !Finite(input.camera_center) || !std::isfinite(input.altitude_above_ground_m) ||
        !std::isfinite(input.altitude_error_hard_bound_m) ||
        !std::isfinite(input.horizontal_error_hard_bound_m) ||
        !std::isfinite(input.along_fov_deg) || !std::isfinite(input.cross_fov_deg) ||
        !std::isfinite(required_strip_half_width_m) ||
        input.altitude_error_hard_bound_m < 0.0 ||
        input.horizontal_error_hard_bound_m < 0.0 ||
        required_strip_half_width_m <= 0.0 ||
        input.along_fov_deg <= 0.0 || input.along_fov_deg >= 170.0 ||
        input.cross_fov_deg <= 0.0 || input.cross_fov_deg >= 170.0) return std::nullopt;
    const double lower_altitude = input.altitude_above_ground_m -
                                  input.altitude_error_hard_bound_m;
    if (lower_altitude <= 0.0) return std::nullopt;
    const auto projection = Project(route, input.camera_center);
    if (!projection || projection->ambiguous) return std::nullopt;
    const double radians = std::numbers::pi / 180.0;
    const double half_along = lower_altitude *
        std::tan(input.along_fov_deg * radians / 2.0) -
        input.horizontal_error_hard_bound_m;
    const double half_cross = lower_altitude *
        std::tan(input.cross_fov_deg * radians / 2.0) -
        input.horizontal_error_hard_bound_m;
    if (!std::isfinite(half_along) || !std::isfinite(half_cross) ||
        half_along <= 0.0 ||
        projection->cross_track_m + required_strip_half_width_m >
            half_cross + kGeometryToleranceM) return std::nullopt;
    const double length = Length(route);
    const Interval covered{std::max(0.0, projection->arclength_m - half_along),
                           std::min(length, projection->arclength_m + half_along)};
    if (covered.end_m - covered.begin_m <= kGeometryToleranceM) return std::nullopt;
    return CaptureEvidence{.mission_id = input.mission_id,
                           .mission_revision = input.mission_revision,
                           .edge_id = input.edge_id,
                           .attempt_id = input.attempt_id,
                           .capture_id = input.capture_id,
                           .calibration_id = input.calibration_id,
                           .monotonic_ms = input.monotonic_ms,
                           .guaranteed_full_strip_interval = covered,
                           .capture_succeeded = true,
                           .full_strip_quality_established = true};
}

std::optional<Point> PointAt(const Route& route, double arclength_m) {
    const double length = Length(route);
    if (length <= 0.0 || !std::isfinite(arclength_m)) return std::nullopt;
    double remaining = std::clamp(arclength_m, 0.0, length);
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        const Point a = route.points[i - 1];
        const Point b = route.points[i];
        const double segment = Distance(a, b);
        if (remaining <= segment || i + 1 == route.points.size()) {
            const double t = std::clamp(remaining / segment, 0.0, 1.0);
            return Point{a.east_m + t * (b.east_m - a.east_m),
                         a.north_m + t * (b.north_m - a.north_m)};
        }
        remaining -= segment;
    }
    return std::nullopt;
}

Point Wgs84ToLocal(double origin_lat_deg, double origin_lon_deg, double lat_deg, double lon_deg) {
    const double radians = std::numbers::pi / 180.0;
    return {(lon_deg - origin_lon_deg) * radians * kEarthRadiusM *
                std::cos(origin_lat_deg * radians),
            (lat_deg - origin_lat_deg) * radians * kEarthRadiusM};
}

GeoPoint LocalToWgs84(double origin_lat_deg, double origin_lon_deg, Point local) {
    const double degrees = 180.0 / std::numbers::pi;
    return {origin_lat_deg + local.north_m / kEarthRadiusM * degrees,
            origin_lon_deg + local.east_m / (kEarthRadiusM *
                                            std::cos(origin_lat_deg / degrees)) * degrees};
}

IntervalSet::IntervalSet(double length_m) : length_m_(length_m) {}

bool IntervalSet::Add(Interval interval) {
    if (!std::isfinite(interval.begin_m) || !std::isfinite(interval.end_m) ||
        !std::isfinite(length_m_) || length_m_ <= 0.0 ||
        interval.begin_m > interval.end_m) return false;
    interval.begin_m = std::clamp(interval.begin_m, 0.0, length_m_);
    interval.end_m = std::clamp(interval.end_m, 0.0, length_m_);
    if (interval.end_m - interval.begin_m <= kGeometryToleranceM) return false;
    intervals_.push_back(interval);
    std::sort(intervals_.begin(), intervals_.end(),
              [](Interval a, Interval b) { return a.begin_m < b.begin_m; });
    std::vector<Interval> merged;
    merged.reserve(intervals_.size());
    for (const auto& item : intervals_) {
        if (!merged.empty() && item.begin_m <= merged.back().end_m + kGeometryToleranceM) {
            merged.back().end_m = std::max(merged.back().end_m, item.end_m);
        } else {
            merged.push_back(item);
        }
    }
    intervals_ = std::move(merged);
    return true;
}

std::vector<Interval> IntervalSet::Missing() const {
    std::vector<Interval> missing;
    double cursor = 0.0;
    for (const auto& item : intervals_) {
        if (item.begin_m > cursor + kGeometryToleranceM) missing.push_back({cursor, item.begin_m});
        cursor = item.end_m;
    }
    if (cursor < length_m_ - kGeometryToleranceM) missing.push_back({cursor, length_m_});
    return missing;
}

double IntervalSet::CoveredMeters() const {
    double total = 0.0;
    for (const auto& item : intervals_) total += item.end_m - item.begin_m;
    return total;
}

double IntervalSet::PrefixMeters() const {
    return !intervals_.empty() && intervals_.front().begin_m <= kGeometryToleranceM
               ? intervals_.front().end_m
               : 0.0;
}

EdgeExecutionSupervisor::EdgeExecutionSupervisor(EdgeRequest request)
    : request_(std::move(request)),
      length_m_(Length(request_.route)),
      traversal_(length_m_),
      survey_(length_m_) {
    status_.phase = Valid() ? Phase::kMoving : Phase::kInterrupted;
    status_.reason = Valid() ? "awaiting observation" : "invalid edge request";
    Recompute();
}

bool EdgeExecutionSupervisor::Valid() const {
    return !request_.mission_id.empty() && request_.mission_revision > 0 &&
           !request_.edge_id.empty() && !request_.attempt_id.empty() &&
           !request_.drone_id.empty() && length_m_ > 0.0 &&
           std::isfinite(request_.corridor_half_width_m) &&
           request_.corridor_half_width_m > 0.0 &&
           std::isfinite(request_.arrival_radius_m) && request_.arrival_radius_m > 0.0 &&
           std::isfinite(request_.arrival_max_speed_mps) &&
           request_.arrival_max_speed_mps >= 0.0 &&
           std::isfinite(request_.target_altitude_m) &&
           std::isfinite(request_.maximum_missing_m) && request_.maximum_missing_m >= 0.0 &&
           std::isfinite(request_.maximum_speed_mps) && request_.maximum_speed_mps >= 0.0 &&
           std::isfinite(request_.minimum_recovery_battery_percent) &&
           request_.minimum_recovery_battery_percent >= 0.0 &&
           request_.minimum_recovery_battery_percent <= 100.0 &&
           std::isfinite(request_.recovery_overlap_m) && request_.recovery_overlap_m >= 0.0 &&
           request_.sustained_deviation_samples > 0 && request_.recovery_cooldown_ms >= 0 &&
           request_.timeout_ms > 0;
}

void EdgeExecutionSupervisor::Recompute() {
    status_.survey_covered_m = survey_.CoveredMeters();
    status_.survey_prefix_m = survey_.PrefixMeters();
    status_.survey_missing = survey_.Missing();
    status_.traversal_missing = traversal_.Missing();
    status_.survey_verified = length_m_ - survey_.CoveredMeters() <= request_.maximum_missing_m +
                                                         kGeometryToleranceM;
    status_.traversal_verified = length_m_ - traversal_.CoveredMeters() <=
                                 request_.maximum_missing_m + kGeometryToleranceM;
    if (Terminal(status_.phase)) return;
    const bool obligation_met =
        status_.at_b_verified &&
        (request_.obligation == Obligation::kArrival ||
         (request_.obligation == Obligation::kTraversal && status_.traversal_verified) ||
         (request_.obligation == Obligation::kSurvey && status_.survey_verified) ||
         (request_.obligation == Obligation::kTraversalAndSurvey &&
          status_.traversal_verified && status_.survey_verified));
    if (obligation_met) {
        status_.phase = Phase::kFulfilled;
        status_.reason = "edge obligation verified";
        status_.pending_reentry.reset();
    }
}

void EdgeExecutionSupervisor::ConsiderRecovery(std::int64_t now_ms, bool safe_to_navigate) {
    if (Terminal(status_.phase) || status_.pending_reentry || !safe_to_navigate ||
        request_.max_recovery_attempts == 0 ||
        status_.recovery_attempts >= request_.max_recovery_attempts ||
        (last_recovery_ms_ >= 0 && now_ms - last_recovery_ms_ < request_.recovery_cooldown_ms))
        return;
    auto gaps = request_.obligation == Obligation::kTraversal
                    ? traversal_.Missing() : survey_.Missing();
    if (request_.obligation == Obligation::kTraversalAndSurvey) {
        const auto traversal_gaps = traversal_.Missing();
        gaps.insert(gaps.end(), traversal_gaps.begin(), traversal_gaps.end());
        std::sort(gaps.begin(), gaps.end(),
                  [](Interval a, Interval b) { return a.begin_m < b.begin_m; });
    }
    if (request_.obligation == Obligation::kArrival || gaps.empty()) return;
    const double s = std::max(0.0, gaps.front().begin_m - request_.recovery_overlap_m);
    status_.pending_reentry = PointAt(request_.route, s);
    status_.phase = Phase::kRecovering;
    status_.reason = "re-entry proposed for earliest unverified gap";
}

Status EdgeExecutionSupervisor::ObservePosition(const PositionObservation& observation) {
    if (Terminal(status_.phase)) return Snapshot();
    if (observation.monotonic_ms < 0 ||
        observation.monotonic_ms <= latest_position_ms_) return Snapshot();
    if (started_ms_ < 0) started_ms_ = observation.monotonic_ms;
    latest_position_ms_ = observation.monotonic_ms;
    if (observation.monotonic_ms - started_ms_ > request_.timeout_ms) {
        status_.phase = Phase::kInterrupted;
        status_.reason = "edge timeout";
        status_.pending_reentry.reset();
        return Snapshot();
    }
    if (observation.failsafe) return Interrupt("vehicle failsafe");
    if (observation.battery_percent &&
        (!std::isfinite(*observation.battery_percent) ||
         *observation.battery_percent < request_.minimum_recovery_battery_percent))
        return Interrupt("battery below edge recovery threshold");
    if (!observation.valid || !Finite(observation.position) ||
        (observation.horizontal_error_hard_bound_m &&
         (!std::isfinite(*observation.horizontal_error_hard_bound_m) ||
          *observation.horizontal_error_hard_bound_m < 0.0)) ||
        (observation.speed_error_hard_bound_mps &&
         (!std::isfinite(*observation.speed_error_hard_bound_mps) ||
          *observation.speed_error_hard_bound_mps < 0.0)) ||
        (observation.altitude_m && !std::isfinite(*observation.altitude_m)) ||
        (observation.vertical_error_hard_bound_m &&
         (!std::isfinite(*observation.vertical_error_hard_bound_m) ||
          *observation.vertical_error_hard_bound_m < 0.0))) {
        previous_.reset();
        previous_projection_.reset();
        status_.at_b_verified = false;
        arrival_candidate_ms_ = -1;
        status_.phase = Phase::kUnknown;
        status_.traversal_quality = EvidenceQuality::kUnknown;
        status_.reason = "position unavailable or invalid";
        return Snapshot();
    }
    const auto projection = Project(request_.route, observation.position);
    if (!projection || projection->ambiguous) {
        previous_.reset();
        previous_projection_.reset();
        status_.at_b_verified = false;
        arrival_candidate_ms_ = -1;
        status_.phase = Phase::kUnknown;
        status_.reason = "route projection ambiguous";
        return Snapshot();
    }
    const double endpoint_distance = Distance(observation.position, request_.route.points.back());
    last_valid_position_ms_ = observation.monotonic_ms;
    const bool qualified_at_b = observation.horizontal_error_hard_bound_m &&
        observation.speed_mps && std::isfinite(*observation.speed_mps) &&
        *observation.speed_mps >= 0.0 && observation.speed_error_hard_bound_mps &&
        observation.altitude_m && observation.vertical_error_hard_bound_m &&
        *observation.speed_mps + *observation.speed_error_hard_bound_mps <=
            request_.arrival_max_speed_mps &&
        std::hypot(endpoint_distance + *observation.horizontal_error_hard_bound_m,
                   std::abs(*observation.altitude_m - request_.target_altitude_m) +
                       *observation.vertical_error_hard_bound_m) <= request_.arrival_radius_m;
    if (qualified_at_b) {
        if (arrival_candidate_ms_ < 0) arrival_candidate_ms_ = observation.monotonic_ms;
        status_.at_b_verified = observation.monotonic_ms - arrival_candidate_ms_ >=
                                kArrivalDwellMs;
    } else {
        arrival_candidate_ms_ = -1;
        status_.at_b_verified = false;
    }

    const double error = observation.horizontal_error_hard_bound_m.value_or(0.0);
    const bool within_sample = projection->cross_track_m + error <= request_.corridor_half_width_m;
    if (!within_sample) ++deviation_samples_;
    else deviation_samples_ = 0;
    if (request_.route.points.size() == 2 && previous_ && previous_projection_ && within_sample &&
        !previous_projection_->ambiguous && previous_->horizontal_error_hard_bound_m &&
        observation.horizontal_error_hard_bound_m && request_.maximum_speed_mps > 0.0 &&
        projection->arclength_m > previous_projection_->arclength_m) {
        const double dt = static_cast<double>(observation.monotonic_ms - previous_->monotonic_ms) /
                          1000.0;
        const double previous_bound = previous_projection_->cross_track_m +
                                      *previous_->horizontal_error_hard_bound_m;
        const double displacement_bound = request_.maximum_speed_mps * dt +
            *previous_->horizontal_error_hard_bound_m +
            *observation.horizontal_error_hard_bound_m;
        const double whole_interval_bound =
            (previous_bound + projection->cross_track_m + error +
             request_.maximum_speed_mps * dt) / 2.0;
        if (Distance(previous_->position, observation.position) <=
                displacement_bound + kGeometryToleranceM &&
            whole_interval_bound <= request_.corridor_half_width_m) {
            static_cast<void>(traversal_.Add({std::min(previous_projection_->arclength_m,
                                                       projection->arclength_m),
                                              std::max(previous_projection_->arclength_m,
                                                       projection->arclength_m)}));
            status_.traversal_quality = EvidenceQuality::kBounded;
        }
    } else if (within_sample && status_.traversal_quality == EvidenceQuality::kUnknown) {
        status_.traversal_quality = EvidenceQuality::kEstimated;
    }
    previous_ = observation;
    previous_projection_ = projection;
    can_recover_ = observation.safe_to_navigate && observation.battery_percent.has_value() &&
                   observation.horizontal_error_hard_bound_m.has_value() &&
                   *observation.horizontal_error_hard_bound_m <=
                       request_.corridor_half_width_m;
    if (status_.phase == Phase::kUnknown) status_.phase = Phase::kMoving;
    Recompute();
    if (deviation_samples_ >= request_.sustained_deviation_samples ||
        (status_.at_b_verified && status_.phase != Phase::kFulfilled)) {
        ConsiderRecovery(observation.monotonic_ms, can_recover_);
    }
    return Snapshot();
}

Status EdgeExecutionSupervisor::ObserveCapture(const CaptureEvidence& evidence) {
    if (Terminal(status_.phase) || evidence.mission_id != request_.mission_id ||
        evidence.mission_revision != request_.mission_revision ||
        evidence.edge_id != request_.edge_id ||
        evidence.attempt_id != request_.attempt_id ||
        evidence.capture_id.empty() || evidence.calibration_id.empty() ||
        !evidence.capture_succeeded || !evidence.full_strip_quality_established ||
        started_ms_ < 0 || evidence.monotonic_ms < started_ms_ ||
        evidence.monotonic_ms > latest_position_ms_ + kArrivalEvidenceFreshnessMs ||
        captures_.size() >= 4096 || captures_.contains(evidence.capture_id)) return Snapshot();
    if (status_.at_b_verified &&
        evidence.monotonic_ms - last_valid_position_ms_ > kArrivalEvidenceFreshnessMs) {
        status_.at_b_verified = false;
        arrival_candidate_ms_ = -1;
    }
    if (survey_.Add(evidence.guaranteed_full_strip_interval)) {
        captures_.emplace(evidence.capture_id, evidence);
        Recompute();
    }
    return Snapshot();
}

Status EdgeExecutionSupervisor::RecoveryDispatched(bool accepted, std::int64_t monotonic_ms) {
    if (Terminal(status_.phase) || !status_.pending_reentry) return Snapshot();
    status_.pending_reentry.reset();
    if (!accepted) return Interrupt("re-entry command rejected by safety/backend");
    ++status_.recovery_attempts;
    last_recovery_ms_ = monotonic_ms;
    deviation_samples_ = 0;
    status_.at_b_verified = false;  // A repaired edge must finish at B again.
    arrival_candidate_ms_ = -1;
    status_.phase = Phase::kMoving;
    status_.reason = "re-entry command accepted; physical outcome pending";
    return Snapshot();
}

Status EdgeExecutionSupervisor::Interrupt(std::string reason) {
    if (!Terminal(status_.phase)) {
        status_.phase = Phase::kInterrupted;
        status_.reason = std::move(reason);
        status_.pending_reentry.reset();
    }
    return Snapshot();
}

Status EdgeExecutionSupervisor::Snapshot() const { return status_; }

bool EdgeExecutionSupervisor::WasCaptureAccepted(const CaptureEvidence& evidence) const {
    const auto it = captures_.find(evidence.capture_id);
    return it != captures_.end() && it->second == evidence;
}

}  // namespace swarmkit::edge
