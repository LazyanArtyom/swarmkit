// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "active_goal_supervisor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include "swarmkit/core/logger.h"
#include "swarmkit/core/telemetry.h"

namespace swarmkit::agent::internal {
namespace {

constexpr auto kTelemetryWaitTimeout = std::chrono::milliseconds{200};

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] double DegToRad(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] double DistanceMeters(double lat_a_deg, double lon_a_deg, double lat_b_deg,
                                    double lon_b_deg) {
    constexpr double kEarthRadiusMeters = 6371000.0;
    const double lat_a = DegToRad(lat_a_deg);
    const double lat_b = DegToRad(lat_b_deg);
    const double delta_lat = DegToRad(lat_b_deg - lat_a_deg);
    const double delta_lon = DegToRad(lon_b_deg - lon_a_deg);
    const double sin_lat = std::sin(delta_lat / 2.0);
    const double sin_lon = std::sin(delta_lon / 2.0);
    const double haversine =
        (sin_lat * sin_lat) + (std::cos(lat_a) * std::cos(lat_b) * sin_lon * sin_lon);
    return 2.0 * kEarthRadiusMeters * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
}

struct LocalPointMeters {
    double x{};
    double y{};
};

[[nodiscard]] LocalPointMeters ProjectMeters(const core::TelemetryFrame& origin, double lat_deg,
                                             double lon_deg) {
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_degree_lon = kMetersPerDegreeLat * std::cos(DegToRad(origin.lat_deg));
    return {
        .x = (lon_deg - origin.lon_deg) * meters_per_degree_lon,
        .y = (lat_deg - origin.lat_deg) * kMetersPerDegreeLat,
    };
}

[[nodiscard]] double CorridorDeviationMeters(const core::TelemetryFrame& origin,
                                             const core::TelemetryFrame& current,
                                             const swarmkit::v1::GeoPoint& target) {
    const LocalPointMeters target_point = ProjectMeters(origin, target.lat_deg(), target.lon_deg());
    const LocalPointMeters current_point = ProjectMeters(origin, current.lat_deg, current.lon_deg);
    const double segment_len_sq =
        (target_point.x * target_point.x) + (target_point.y * target_point.y);
    if (segment_len_sq <= 1.0) {
        return DistanceMeters(current.lat_deg, current.lon_deg, target.lat_deg(), target.lon_deg());
    }
    const double projection =
        ((current_point.x * target_point.x) + (current_point.y * target_point.y)) / segment_len_sq;
    const double clamped_projection = std::clamp(projection, 0.0, 1.0);
    const double closest_x = target_point.x * clamped_projection;
    const double closest_y = target_point.y * clamped_projection;
    const double delta_x = current_point.x - closest_x;
    const double delta_y = current_point.y - closest_y;
    return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

[[nodiscard]] std::int64_t ComputeGoalTimeoutMs(const VehicleProfile& profile,
                                                const swarmkit::v1::ActiveGoal& goal,
                                                const std::optional<core::TelemetryFrame>& frame) {
    if (goal.timeout_ms() > 0) {
        return goal.timeout_ms();
    }
    double travel_seconds = 30.0;
    if (frame.has_value()) {
        const double distance_m =
            DistanceMeters(frame->lat_deg, frame->lon_deg, goal.target().lat_deg(),
                           goal.target().lon_deg());
        const float speed_mps =
            goal.speed_mps() > 0.0F ? goal.speed_mps() : profile.cruise_speed_mps;
        const double horizontal_seconds = speed_mps > 0.0F ? distance_m / speed_mps : 0.0;
        const double alt_delta = goal.target().alt_m() - frame->rel_alt_m;
        const float vertical_speed =
            alt_delta >= 0.0 ? profile.climb_speed_mps : profile.descent_speed_mps;
        const double vertical_seconds =
            vertical_speed > 0.0F ? std::abs(alt_delta) / vertical_speed : 0.0;
        travel_seconds = std::max(horizontal_seconds, vertical_seconds);
    }
    const auto calculated_ms =
        static_cast<std::int64_t>((travel_seconds * 1000.0) + profile.goal_margin_ms);
    return std::clamp<std::int64_t>(calculated_ms, profile.goal_margin_ms,
                                    profile.max_goal_timeout_ms);
}

}  // namespace

ActiveGoalSupervisor::ActiveGoalSupervisor(TelemetryManager* telemetry, ReportHub* reports,
                                           const AgentConfig* config)
    : telemetry_(telemetry), reports_(reports), config_(config) {}

ActiveGoalSupervisor::~ActiveGoalSupervisor() noexcept {
    try {
        Shutdown();
    } catch (const std::exception& exc) {
        core::Logger::ErrorFmt("ActiveGoalSupervisor shutdown failed: {}", exc.what());
    } catch (...) {
        core::Logger::Error("ActiveGoalSupervisor shutdown failed with unknown exception");
    }
}

core::Result ActiveGoalSupervisor::ValidateGoal(const swarmkit::v1::ActiveGoal& goal) {
    if (goal.drone_id().empty()) {
        return core::Result::Rejected("goal.drone_id must not be empty");
    }
    if (goal.goal_id().empty()) {
        return core::Result::Rejected("goal.goal_id must not be empty");
    }
    if (!goal.has_target()) {
        return core::Result::Rejected("goal.target must be set");
    }
    if (goal.target().lat_deg() < -90.0 || goal.target().lat_deg() > 90.0 ||
        goal.target().lon_deg() < -180.0 || goal.target().lon_deg() > 180.0) {
        return core::Result::Rejected("goal target latitude/longitude out of range");
    }
    if (goal.speed_mps() < 0.0F) {
        return core::Result::Rejected("goal.speed_mps must be >= 0");
    }
    if (goal.acceptance_radius_m() <= 0.0F) {
        return core::Result::Rejected("goal.acceptance_radius_m must be > 0");
    }
    if (goal.deviation_radius_m() < 0.0F) {
        return core::Result::Rejected("goal.deviation_radius_m must be >= 0");
    }
    if (goal.timeout_ms() < 0) {
        return core::Result::Rejected("goal.timeout_ms must be >= 0");
    }
    return core::Result::Success();
}

std::int64_t ActiveGoalSupervisor::StartGoal(swarmkit::v1::ActiveGoal goal,
                                             std::string correlation_id) {
    std::thread old_worker;
    std::shared_ptr<std::atomic<bool>> old_stop;
    std::int64_t out_timeout{};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto iter = goals_.find(goal.drone_id()); iter != goals_.end()) {
            old_stop = iter->second.stop;
            PublishGoalReport(iter->second.goal, swarmkit::v1::GOAL_SUPERSEDED,
                              swarmkit::v1::REPORT_INFO, 0.0, 0.0, 0.0,
                              iter->second.computed_timeout_ms, iter->second.started_unix_ms,
                              "superseded by newer active goal", iter->second.correlation_id);
            old_worker = std::move(iter->second.worker);
            goals_.erase(iter);
        }
        if (old_stop) {
            old_stop->store(true, std::memory_order_relaxed);
        }

        ActiveGoalRuntime runtime;
        runtime.goal = std::move(goal);
        runtime.correlation_id = std::move(correlation_id);
        runtime.started_unix_ms = NowUnixMs();
        runtime.computed_timeout_ms =
            ComputeGoalTimeoutMs(config_->vehicle_profile, runtime.goal, std::nullopt);
        runtime.status = swarmkit::v1::GOAL_ACTIVE;
        runtime.stop = std::make_shared<std::atomic<bool>>(false);
        const std::string drone_id = runtime.goal.drone_id();
        const auto stop = runtime.stop;
        const auto runtime_goal = runtime.goal;
        const std::int64_t timeout_ms = runtime.computed_timeout_ms;
        const std::string runtime_correlation_id = runtime.correlation_id;
        const std::int64_t started_ms = runtime.started_unix_ms;
        // NOLINTNEXTLINE(bugprone-exception-escape): the thread entry catches all exceptions.
        runtime.worker = std::thread([this, runtime_goal, timeout_ms, started_ms,
                                      runtime_correlation_id, stop] noexcept {
            try {
                MonitorGoal(runtime_goal, timeout_ms, started_ms, runtime_correlation_id, stop);
            } catch (...) {
                static_cast<void>(std::fputs("Active goal monitor failed\n", stderr));
            }
        });
        PublishGoalReport(runtime.goal, swarmkit::v1::GOAL_ACTIVE, swarmkit::v1::REPORT_INFO, 0.0,
                          0.0, 0.0, runtime.computed_timeout_ms, runtime.started_unix_ms,
                          "active goal accepted", runtime.correlation_id);
        out_timeout = runtime.computed_timeout_ms;
        goals_.emplace(drone_id, std::move(runtime));
    }
    if (old_worker.joinable()) {
        old_worker.join();
    }
    return out_timeout;
}

core::Result ActiveGoalSupervisor::CancelGoal(const std::string& drone_id, std::string_view goal_id,
                                              std::string_view correlation_id) {
    std::thread old_worker;
    std::shared_ptr<std::atomic<bool>> old_stop;
    swarmkit::v1::ActiveGoal old_goal;
    std::int64_t timeout_ms{};
    std::int64_t started_ms{};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = goals_.find(drone_id);
        if (iter == goals_.end()) {
            return core::Result::Rejected("no active goal for drone");
        }
        if (!goal_id.empty() && iter->second.goal.goal_id() != goal_id) {
            return core::Result::Rejected("active goal id does not match cancel request");
        }
        old_goal = iter->second.goal;
        timeout_ms = iter->second.computed_timeout_ms;
        started_ms = iter->second.started_unix_ms;
        old_stop = iter->second.stop;
        old_worker = std::move(iter->second.worker);
        goals_.erase(iter);
    }
    if (old_stop) {
        old_stop->store(true, std::memory_order_relaxed);
    }
    if (old_worker.joinable()) {
        old_worker.join();
    }
    PublishGoalReport(old_goal, swarmkit::v1::GOAL_CANCELLED, swarmkit::v1::REPORT_INFO, 0.0, 0.0,
                      0.0, timeout_ms, started_ms, "goal cancelled",
                      std::string(correlation_id));
    return core::Result::Success("goal cancelled");
}

std::optional<std::pair<swarmkit::v1::ActiveGoal, std::int64_t>> ActiveGoalSupervisor::GetGoal(
    const std::string& drone_id, swarmkit::v1::GoalStatus* status) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iter = goals_.find(drone_id);
    if (iter == goals_.end()) {
        return std::nullopt;
    }
    if (status != nullptr) {
        *status = iter->second.status;
    }
    return std::make_pair(iter->second.goal, iter->second.computed_timeout_ms);
}

void ActiveGoalSupervisor::Shutdown() {
    std::vector<std::thread> workers;
    std::vector<std::shared_ptr<std::atomic<bool>>> stops;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [_, runtime] : goals_) {
            stops.push_back(runtime.stop);
            workers.push_back(std::move(runtime.worker));
        }
        goals_.clear();
    }
    for (const auto& stop : stops) {
        if (stop) {
            stop->store(true, std::memory_order_relaxed);
        }
    }
    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void ActiveGoalSupervisor::SetTerminalStatus(const std::string& drone_id,
                                             swarmkit::v1::GoalStatus status) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto iter = goals_.find(drone_id); iter != goals_.end()) {
        iter->second.status = status;
    }
}

void ActiveGoalSupervisor::PublishGoalReport(
    const swarmkit::v1::ActiveGoal& goal, swarmkit::v1::GoalStatus status,
    swarmkit::v1::ReportSeverity severity, double distance_to_goal_m, double deviation_m,
    double altitude_error_m, std::int64_t timeout_ms, std::int64_t started_ms,
    std::string_view message, std::string_view correlation_id) {
    if (reports_ == nullptr) {
        return;
    }
    swarmkit::v1::AgentReport report;
    report.set_drone_id(goal.drone_id());
    report.set_correlation_id(std::string(correlation_id));
    report.set_type(swarmkit::v1::GOAL_REPORT);
    report.set_severity(severity);
    report.set_message(std::string(message));
    auto* goal_report = report.mutable_goal();
    goal_report->set_drone_id(goal.drone_id());
    goal_report->set_goal_id(goal.goal_id());
    goal_report->set_revision(goal.revision());
    goal_report->set_status(status);
    goal_report->set_distance_to_goal_m(distance_to_goal_m);
    goal_report->set_deviation_m(deviation_m);
    goal_report->set_altitude_error_m(altitude_error_m);
    goal_report->set_acceptance_radius_m(goal.acceptance_radius_m());
    goal_report->set_deviation_radius_m(goal.deviation_radius_m());
    goal_report->set_elapsed_ms(std::max<std::int64_t>(0, NowUnixMs() - started_ms));
    goal_report->set_timeout_ms(timeout_ms);
    goal_report->set_message(std::string(message));
    reports_->Publish(std::move(report));
}

void ActiveGoalSupervisor::MonitorGoal(const swarmkit::v1::ActiveGoal& goal,
                                       std::int64_t configured_timeout_ms,
                                       std::int64_t started_ms,
                                       const std::string& correlation_id,
                                       const std::shared_ptr<std::atomic<bool>>& stop) {
    TelemetryLease lease;
    const core::Result acquire_result = telemetry_->AcquireLease(
        goal.drone_id(), std::max(1, config_->default_telemetry_rate_hz), &lease);
    if (!acquire_result.IsOk()) {
        PublishGoalReport(goal, swarmkit::v1::GOAL_FAILED, swarmkit::v1::REPORT_ERROR, 0.0, 0.0,
                          0.0, configured_timeout_ms, started_ms,
                          "telemetry unavailable: " + acquire_result.message, correlation_id);
        SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_FAILED);
        return;
    }

    std::uint64_t last_sequence = 0;
    std::optional<core::TelemetryFrame> origin;
    bool reported_deviation = false;
    std::int64_t timeout_ms = configured_timeout_ms;

    while (stop && !stop->load(std::memory_order_relaxed)) {
        core::TelemetryFrame frame;
        if (!TelemetryManager::WaitForFrame(lease, &last_sequence, &frame, kTelemetryWaitTimeout)) {
            if (NowUnixMs() - started_ms >= timeout_ms) {
                PublishGoalReport(goal, swarmkit::v1::GOAL_TIMEOUT, swarmkit::v1::REPORT_ERROR,
                                  0.0, 0.0, 0.0, timeout_ms, started_ms,
                                  "goal timed out without fresh telemetry", correlation_id);
                SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_TIMEOUT);
                break;
            }
            continue;
        }
        if (!origin.has_value()) {
            origin = frame;
            timeout_ms = ComputeGoalTimeoutMs(config_->vehicle_profile, goal, origin);
        }

        const double distance_to_goal_m =
            DistanceMeters(frame.lat_deg, frame.lon_deg, goal.target().lat_deg(),
                           goal.target().lon_deg());
        const double deviation_m = CorridorDeviationMeters(*origin, frame, goal.target());
        const double altitude_error_m = std::abs(frame.rel_alt_m - goal.target().alt_m());
        const bool altitude_ok = altitude_error_m <= std::max(1.0F, goal.acceptance_radius_m());

        if (distance_to_goal_m <= goal.acceptance_radius_m() && altitude_ok) {
            PublishGoalReport(goal, swarmkit::v1::GOAL_REACHED, swarmkit::v1::REPORT_INFO,
                              distance_to_goal_m, deviation_m, altitude_error_m, timeout_ms,
                              started_ms, "goal reached", correlation_id);
            SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_REACHED);
            break;
        }

        if (goal.deviation_radius_m() > 0.0F && deviation_m > goal.deviation_radius_m()) {
            if (!reported_deviation) {
                PublishGoalReport(goal, swarmkit::v1::GOAL_DEVIATING,
                                  swarmkit::v1::REPORT_WARNING, distance_to_goal_m, deviation_m,
                                  altitude_error_m, timeout_ms, started_ms,
                                  "outside goal deviation radius", correlation_id);
                reported_deviation = true;
            }
        } else if (reported_deviation) {
            PublishGoalReport(goal, swarmkit::v1::GOAL_ACTIVE, swarmkit::v1::REPORT_INFO,
                              distance_to_goal_m, deviation_m, altitude_error_m, timeout_ms,
                              started_ms, "goal back inside deviation envelope", correlation_id);
            reported_deviation = false;
        }

        if (NowUnixMs() - started_ms >= timeout_ms) {
            PublishGoalReport(goal, swarmkit::v1::GOAL_TIMEOUT, swarmkit::v1::REPORT_ERROR,
                              distance_to_goal_m, deviation_m, altitude_error_m, timeout_ms,
                              started_ms, "goal timed out", correlation_id);
            SetTerminalStatus(goal.drone_id(), swarmkit::v1::GOAL_TIMEOUT);
            break;
        }
    }

    telemetry_->ReleaseLease(lease);
}

}  // namespace swarmkit::agent::internal
