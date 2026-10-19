// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace swarmkit::edge {

struct Point {
    double east_m{};
    double north_m{};
};

struct GeoPoint {
    double lat_deg{};
    double lon_deg{};
};

struct Interval {
    double begin_m{};
    double end_m{};
    bool operator==(const Interval&) const = default;
};

// All route points and observations use one explicitly chosen local metric frame.
// The caller must transform WGS84 observations against the same fixed origin.
struct Route {
    std::vector<Point> points;
};

struct Projection {
    double arclength_m{};
    double cross_track_m{};
    std::size_t segment{};
    bool ambiguous{};
};

[[nodiscard]] double Length(const Route& route);
[[nodiscard]] std::optional<Projection> Project(const Route& route, Point point);
[[nodiscard]] std::optional<Point> PointAt(const Route& route, double arclength_m);
[[nodiscard]] Point Wgs84ToLocal(double origin_lat_deg, double origin_lon_deg, double lat_deg,
                                 double lon_deg);
[[nodiscard]] GeoPoint LocalToWgs84(double origin_lat_deg, double origin_lon_deg, Point local);

class IntervalSet {
   public:
    explicit IntervalSet(double length_m);
    [[nodiscard]] bool Add(Interval interval);
    [[nodiscard]] std::vector<Interval> Missing() const;
    [[nodiscard]] double CoveredMeters() const;
    [[nodiscard]] double PrefixMeters() const;
    [[nodiscard]] const std::vector<Interval>& Intervals() const { return intervals_; }

   private:
    double length_m_{};
    std::vector<Interval> intervals_;
};

enum class Obligation { kArrival, kTraversal, kSurvey, kTraversalAndSurvey };
enum class Phase { kMoving, kRecovering, kFulfilled, kInterrupted, kUnknown };
enum class EvidenceQuality { kUnknown, kEstimated, kBounded };

struct EdgeRequest {
    std::string mission_id;
    std::uint64_t mission_revision{};
    std::string edge_id;
    std::string attempt_id;
    std::string drone_id;
    Route route;
    Obligation obligation{Obligation::kSurvey};
    double corridor_half_width_m{};
    double arrival_radius_m{};
    double arrival_max_speed_mps{};
    double target_altitude_m{};  // Relative altitude in the command/telemetry frame.
    double maximum_missing_m{};
    double maximum_speed_mps{};  // Required for whole-interval traversal proof.
    double minimum_recovery_battery_percent{};
    double recovery_overlap_m{};
    std::uint32_t max_recovery_attempts{};
    std::uint32_t sustained_deviation_samples{3};
    std::int64_t recovery_cooldown_ms{1000};
    std::int64_t timeout_ms{};
};

struct PositionObservation {
    Point position;
    std::int64_t monotonic_ms{};
    bool valid{};
    bool safe_to_navigate{};
    bool failsafe{};
    std::optional<double> speed_mps;
    // Presence is permitted only for a documented deterministic hard bound.
    std::optional<double> horizontal_error_hard_bound_m;
    std::optional<double> speed_error_hard_bound_mps;
    std::optional<double> altitude_m;
    std::optional<double> vertical_error_hard_bound_m;
    std::optional<double> battery_percent;
};

// A sensor adapter must establish that a successful capture genuinely observed
// the entire required strip cross section over this conservative interval.
// This record is not generated from arrival or from vehicle position alone.
struct CaptureEvidence {
    std::string mission_id;
    std::uint64_t mission_revision{};
    std::string edge_id;
    std::string attempt_id;
    std::string capture_id;
    std::string calibration_id;
    std::int64_t monotonic_ms{};
    Interval guaranteed_full_strip_interval;
    bool capture_succeeded{};
    bool full_strip_quality_established{};
    bool operator==(const CaptureEvidence&) const = default;
};

// A restricted flat-ground, nadir camera adapter. Every input bound and quality
// assertion must come from a calibrated sensor/pose source, not a camera ACK.
// The footprint is conservative only for a straight route, rectangular FOV,
// known route-aligned camera axes, and a horizontal flat survey plane.
struct NadirCaptureInput {
    std::string mission_id;
    std::uint64_t mission_revision{};
    std::string edge_id;
    std::string attempt_id;
    std::string capture_id;
    std::string calibration_id;
    std::int64_t monotonic_ms{};
    Point camera_center;
    double altitude_above_ground_m{};
    double altitude_error_hard_bound_m{};
    double horizontal_error_hard_bound_m{};
    double along_fov_deg{};
    double cross_fov_deg{};
    bool capture_succeeded{};
    bool image_quality_established{};
    bool nadir_route_alignment_established{};
    bool flat_ground_established{};
};

[[nodiscard]] std::optional<CaptureEvidence> BuildNadirStripEvidence(
    const Route& route, double required_strip_half_width_m,
    const NadirCaptureInput& input);

struct Status {
    Phase phase{Phase::kUnknown};
    bool at_b_verified{};
    bool traversal_verified{};
    bool survey_verified{};
    EvidenceQuality traversal_quality{EvidenceQuality::kUnknown};
    double survey_covered_m{};
    double survey_prefix_m{};
    std::vector<Interval> survey_missing;
    std::vector<Interval> traversal_missing;
    std::uint32_t recovery_attempts{};
    std::optional<Point> pending_reentry;
    std::string reason;
};

// Single-writer event processor. The agent owns command arbitration; a pending
// re-entry is only a proposal and must pass backend/safety checks before dispatch.
class EdgeExecutionSupervisor {
   public:
    explicit EdgeExecutionSupervisor(EdgeRequest request);
    [[nodiscard]] bool Valid() const;
    [[nodiscard]] const EdgeRequest& Request() const { return request_; }
    [[nodiscard]] Status ObservePosition(const PositionObservation& observation);
    [[nodiscard]] Status ObserveCapture(const CaptureEvidence& evidence);
    [[nodiscard]] Status RecoveryDispatched(bool accepted, std::int64_t monotonic_ms);
    [[nodiscard]] Status Interrupt(std::string reason);
    [[nodiscard]] Status Snapshot() const;
    [[nodiscard]] bool WasCaptureAccepted(const CaptureEvidence& evidence) const;

   private:
    void Recompute();
    void ConsiderRecovery(std::int64_t now_ms, bool safe_to_navigate);
    EdgeRequest request_;
    double length_m_{};
    IntervalSet traversal_;
    IntervalSet survey_;
    Status status_;
    std::optional<PositionObservation> previous_;
    std::optional<Projection> previous_projection_;
    std::int64_t started_ms_{-1};
    std::int64_t latest_position_ms_{-1};
    std::int64_t arrival_candidate_ms_{-1};
    std::int64_t last_valid_position_ms_{-1};
    std::int64_t last_recovery_ms_{-1};
    std::uint32_t deviation_samples_{};
    bool can_recover_{};
    std::unordered_map<std::string, CaptureEvidence> captures_;
};

}  // namespace swarmkit::edge
