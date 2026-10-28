// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "swarmkit/edge/vecr.h"

namespace swarmkit::testsupport {

enum class TrialFault {
    kNone,
    kTransientCameraGap,
    kPersistentCameraGap,
    kMotorFailureOnRepair,
    kMetadataDrop,
    kDishonestCamera,
    kDetectableBadMetadata,
    kPersistentWind,
    kDetourWind,
    kOffcenterWideCamera,
    kGpsJitter,
    kSparseTelemetry,
    kGnssOutage,
    kEstimatorJump,
    kUnboundedAccuracy,
    kRecoveryCommandRejected,
    kSafetyBattery,
    kFastFlyThrough,
};

struct VecrTrialConfig {
    std::string scenario_id;
    std::uint64_t seed{};
    TrialFault fault{TrialFault::kNone};
    double edge_length_m{20};
    double gap_begin_m{8};
    double gap_end_m{12};
    double camera_along_half_m{2};
    double camera_cross_half_m{10};
    double commanded_speed_mps{2};
    double wind_north_mps{};
    int capture_period_ticks{5};
    int max_ticks{650};
    double max_horizontal_acceleration_mps2{2};
    bool stop_at_target{true};
};

struct VecrTrialEvent {
    int tick{};
    std::int64_t simulation_time_ms{};
    std::string kind;
    bool has_truth_pose{};
    double truth_east_m{};
    double truth_north_m{};
    std::string detail;
};

struct VecrTrialTraceRow {
    int tick{};
    std::int64_t simulation_time_ms{};
    double truth_east_m{};
    double truth_north_m{};
    double truth_up_m{};
    double physical_distance_m{};
    std::size_t truth_covered_cells{};
    double reported_covered_m{};
    edge::Phase reported_phase{edge::Phase::kUnknown};
    std::uint32_t recovery_attempts{};
    bool capture_acquired{};
    bool capture_submitted{};
    bool capture_accepted{};
};

struct VecrTrialResult {
    VecrTrialConfig config;
    std::string attempt_id;
    edge::Phase reported_phase{edge::Phase::kUnknown};
    bool truth_survey_complete{};
    bool truth_at_b{};
    bool infrastructure_valid{true};
    std::string failure_detail;
    int ticks{};
    std::int64_t elapsed_simulation_ms{};
    double physical_distance_m{};
    double max_truth_cross_track_m{};
    double truth_missing_fraction{};
    double truth_longest_gap_m{};
    std::size_t truth_covered_cells{};
    std::size_t truth_cells{};
    double reported_covered_m{};
    std::uint32_t recovery_attempts{};
    int backend_goto_accepted{};
    int backend_goto_rejected{};
    int capture_acquired{};
    int capture_submitted{};
    int capture_accepted{};
    bool requested_fault_realized{};
    std::vector<VecrTrialTraceRow> trace;
    std::vector<VecrTrialEvent> events;
};

[[nodiscard]] VecrTrialResult RunVecrAgentTrial(const VecrTrialConfig& config);

}  // namespace swarmkit::testsupport
