// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "trajectory_execution_manager.h"

#include "command_preconditions.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <expected>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include "swarmkit/core/logger.h"

namespace swarmkit::agent::internal {
namespace {

constexpr auto kTelemetryWaitTimeout = std::chrono::milliseconds{200};
constexpr auto kTelemetryHealthSampleTimeout = std::chrono::milliseconds{1500};
constexpr auto kFinalTargetReportInterval = std::chrono::seconds{1};
constexpr std::int64_t kLateThresholdMs = 250;
constexpr float kDefaultTrackingToleranceM = 2.0F;
constexpr float kMinimumTrajectorySpeedMps = 0.1F;
constexpr float kMinimumSyncQualityPercent = 0.0F;
constexpr float kMaximumSyncQualityPercent = 100.0F;

[[nodiscard]] std::int64_t NowUnixMs() {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] double DegToRad(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] std::string ResolvedDroneId(std::string_view drone_id) {
    return drone_id.empty() ? "default" : std::string(drone_id);
}

[[nodiscard]] swarmkit::v1::TimeSyncState MakeDegradedTimeSyncState(
    std::string_view drone_id, std::string_view source, std::string_view message) {
    swarmkit::v1::TimeSyncState state;
    state.set_drone_id(ResolvedDroneId(drone_id));
    state.set_agent_unix_time_ms(NowUnixMs());
    state.set_vehicle_unix_time_ms(0);
    state.set_clock_offset_ms(0);
    state.set_sync_quality_percent(0.0F);
    state.set_synced(false);
    state.set_stale(true);
    state.set_source(std::string(source));
    state.set_message(std::string(message));
    return state;
}

[[nodiscard]] swarmkit::v1::TimeSyncState ToProtoTimeSyncState(
    const BackendTimeSyncState& backend_state) {
    swarmkit::v1::TimeSyncState state;
    state.set_drone_id(ResolvedDroneId(backend_state.drone_id));
    state.set_agent_unix_time_ms(backend_state.agent_unix_time_ms > 0
                                     ? backend_state.agent_unix_time_ms
                                     : NowUnixMs());
    state.set_vehicle_unix_time_ms(backend_state.vehicle_unix_time_ms);
    state.set_clock_offset_ms(backend_state.clock_offset_ms);

    float quality = std::clamp(backend_state.sync_quality_percent, kMinimumSyncQualityPercent,
                               kMaximumSyncQualityPercent);
    bool synced = backend_state.synced;
    bool stale = backend_state.stale;
    std::string source = backend_state.source.empty() ? "backend" : backend_state.source;
    std::string message = backend_state.message;

    if (synced && backend_state.vehicle_unix_time_ms <= 0) {
        synced = false;
        stale = true;
        quality = 0.0F;
        message = "vehicle clock timestamp unavailable; synchronized execution requires "
                  "proven vehicle time";
    }
    if (!synced && message.empty()) {
        message = "vehicle time synchronization is not established";
    }

    state.set_sync_quality_percent(quality);
    state.set_synced(synced);
    state.set_stale(stale);
    state.set_source(std::move(source));
    state.set_message(std::move(message));
    return state;
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

[[nodiscard]] double DistanceMeters(const swarmkit::v1::GeoPoint& first,
                                    const swarmkit::v1::GeoPoint& second) {
    return DistanceMeters(first.lat_deg(), first.lon_deg(), second.lat_deg(), second.lon_deg());
}

[[nodiscard]] double LocalDistanceMeters(const swarmkit::v1::LocalPoint& first,
                                         const swarmkit::v1::LocalPoint& second) {
    const double delta_x = second.x_m() - first.x_m();
    const double delta_y = second.y_m() - first.y_m();
    return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

[[nodiscard]] bool HasPosition(const swarmkit::v1::TrajectoryPoint& point) {
    return point.has_position() || point.has_local_position();
}

[[nodiscard]] bool HasDispatchTarget(const swarmkit::v1::TrajectoryPoint& point) {
    return HasPosition(point) || point.has_command();
}

[[nodiscard]] bool HasValidationError(const swarmkit::v1::ValidateTrajectoryResult& result) {
    return std::ranges::any_of(result.issues(), [](const auto& issue) {
        return issue.severity() == swarmkit::v1::VALIDATION_ERROR;
    });
}

void AddIssue(swarmkit::v1::ValidateTrajectoryResult* result,
              swarmkit::v1::ValidationSeverity severity, std::string_view code,
              std::string_view message, int point_index = -1) {
    if (result == nullptr) {
        return;
    }
    auto* issue = result->add_issues();
    issue->set_severity(severity);
    issue->set_code(std::string(code));
    issue->set_message(std::string(message));
    issue->set_point_index(point_index);
    if (severity == swarmkit::v1::VALIDATION_ERROR && result->first_failing_point_index() < 0) {
        result->set_first_failing_point_index(point_index);
    }
}

[[nodiscard]] float EffectiveMaxHorizontalSpeed(
    const VehicleProfile& profile, const swarmkit::v1::TrajectoryValidationPolicy& policy) {
    return policy.max_horizontal_speed_mps() > 0.0F ? policy.max_horizontal_speed_mps()
                                                    : profile.cruise_speed_mps;
}

[[nodiscard]] float EffectiveMaxClimbSpeed(const VehicleProfile& profile,
                                           const swarmkit::v1::TrajectoryValidationPolicy& policy) {
    return policy.max_climb_speed_mps() > 0.0F ? policy.max_climb_speed_mps()
                                               : profile.climb_speed_mps;
}

[[nodiscard]] float EffectiveMaxDescentSpeed(
    const VehicleProfile& profile, const swarmkit::v1::TrajectoryValidationPolicy& policy) {
    return policy.max_descent_speed_mps() > 0.0F ? policy.max_descent_speed_mps()
                                                 : profile.descent_speed_mps;
}

[[nodiscard]] float EffectiveMaxAltitude(const VehicleProfile& profile,
                                         const swarmkit::v1::TrajectoryValidationPolicy& policy) {
    return policy.max_altitude_m() > 0.0F ? policy.max_altitude_m() : profile.max_altitude_m;
}

[[nodiscard]] std::int64_t PointScheduleMs(const swarmkit::v1::TrajectoryPoint& point,
                                           std::int64_t start_unix_ms) {
    if (point.unix_time_ms() > 0) {
        return point.unix_time_ms();
    }
    return start_unix_ms + point.time_offset_ms();
}

[[nodiscard]] double DistanceToPointMeters(const core::TelemetryFrame& frame,
                                           const swarmkit::v1::TrajectoryPoint& point) {
    if (!point.has_position()) {
        return 0.0;
    }
    if (!frame.HasPosition() || !frame.HasRelativeAltitude()) {
        return std::numeric_limits<double>::infinity();
    }
    const double horizontal_m = DistanceMeters(
        frame.lat_deg, frame.lon_deg, point.position().lat_deg(), point.position().lon_deg());
    const double vertical_m = static_cast<double>(frame.rel_alt_m) - point.position().alt_m();
    return std::sqrt((horizontal_m * horizontal_m) + (vertical_m * vertical_m));
}

[[nodiscard]] std::expected<commands::Command, core::Result> ConvertTimedProtoCommand(
    const swarmkit::v1::Command& proto) {
    switch (proto.kind_case()) {
        case swarmkit::v1::Command::kArm:
            return commands::FlightCmd{commands::CmdArm{}};
        case swarmkit::v1::Command::kForceArm:
            return commands::FlightCmd{commands::CmdForceArm{}};
        case swarmkit::v1::Command::kDisarm:
            return commands::FlightCmd{commands::CmdDisarm{}};
        case swarmkit::v1::Command::kLand:
            return commands::FlightCmd{commands::CmdLand{}};
        case swarmkit::v1::Command::kTakeoff:
            return commands::FlightCmd{commands::CmdTakeoff{proto.takeoff().alt_m()}};
        case swarmkit::v1::Command::kSetMode:
            return commands::FlightCmd{commands::CmdSetMode{
                .mode = proto.set_mode().mode(),
                .custom_mode = proto.set_mode().custom_mode(),
            }};
        case swarmkit::v1::Command::kForceDisarm:
            return commands::FlightCmd{commands::CmdForceDisarm{}};
        case swarmkit::v1::Command::kFlightTerminate:
            return commands::FlightCmd{commands::CmdFlightTerminate{}};
        case swarmkit::v1::Command::kSetWaypoint:
            return commands::NavCmd{commands::CmdSetWaypoint{
                .lat_deg = proto.set_waypoint().lat_deg(),
                .lon_deg = proto.set_waypoint().lon_deg(),
                .alt_m = proto.set_waypoint().alt_m(),
                .speed_mps = proto.set_waypoint().speed_mps(),
            }};
        case swarmkit::v1::Command::kReturnHome:
            return commands::NavCmd{commands::CmdReturnHome{}};
        case swarmkit::v1::Command::kHoldPosition:
            return commands::NavCmd{commands::CmdHoldPosition{}};
        case swarmkit::v1::Command::kSetSpeed:
            return commands::NavCmd{commands::CmdSetSpeed{proto.set_speed().ground_mps()}};
        case swarmkit::v1::Command::kGotoPosition:
            return commands::NavCmd{commands::CmdGoto{
                .lat_deg = proto.goto_position().lat_deg(),
                .lon_deg = proto.goto_position().lon_deg(),
                .alt_m = proto.goto_position().alt_m(),
                .speed_mps = proto.goto_position().speed_mps(),
                .yaw_deg = proto.goto_position().yaw_deg(),
                .use_yaw = proto.goto_position().use_yaw(),
            }};
        case swarmkit::v1::Command::kPause:
            return commands::NavCmd{commands::CmdPause{}};
        case swarmkit::v1::Command::kResume:
            return commands::NavCmd{commands::CmdResume{}};
        case swarmkit::v1::Command::kSetYaw:
            return commands::NavCmd{commands::CmdSetYaw{
                .yaw_deg = proto.set_yaw().yaw_deg(),
                .rate_deg_s = proto.set_yaw().rate_deg_s(),
                .relative = proto.set_yaw().relative(),
            }};
        case swarmkit::v1::Command::kVelocity:
            return commands::NavCmd{commands::CmdVelocity{
                .vx_mps = proto.velocity().vx_mps(),
                .vy_mps = proto.velocity().vy_mps(),
                .vz_mps = proto.velocity().vz_mps(),
                .duration_ms = proto.velocity().duration_ms(),
                .body_frame = proto.velocity().body_frame(),
            }};
        case swarmkit::v1::Command::kSetHome:
            return commands::NavCmd{commands::CmdSetHome{
                .use_current = proto.set_home().use_current(),
                .lat_deg = proto.set_home().lat_deg(),
                .lon_deg = proto.set_home().lon_deg(),
                .alt_m = proto.set_home().alt_m(),
            }};
        case swarmkit::v1::Command::KIND_NOT_SET:
            return std::unexpected(core::Result::Rejected("timed command is not set"));
        default:
            return std::unexpected(
                core::Result::Rejected("timed trajectory command kind is not supported"));
    }
}

[[nodiscard]] commands::CommandPriority PriorityFromPlanLabels(
    const google::protobuf::Map<std::string, std::string>& labels) {
    const auto iter = labels.find("swarmkit.context.priority");
    if (iter == labels.end()) {
        return commands::CommandPriority::kSupervisor;
    }
    try {
        return static_cast<commands::CommandPriority>(std::stoi(iter->second));
    } catch (...) {
        return commands::CommandPriority::kSupervisor;
    }
}

}  // namespace

TrajectoryExecutionManager::TrajectoryExecutionManager(IDroneBackend* backend,
                                                       TelemetryManager* telemetry,
                                                       ReportHub* reports,
                                                       const AgentConfig* config)
    : backend_(backend), telemetry_(telemetry), reports_(reports), config_(config) {}

TrajectoryExecutionManager::~TrajectoryExecutionManager() noexcept {
    try {
        Shutdown();
    } catch (const std::exception& exc) {
        core::Logger::ErrorFmt("TrajectoryExecutionManager shutdown failed: {}", exc.what());
    } catch (...) {
        core::Logger::Error("TrajectoryExecutionManager shutdown failed with unknown exception");
    }
}

std::string TrajectoryExecutionManager::Key(std::string_view drone_id,
                                            std::string_view execution_id) {
    return std::string(drone_id) + "\n" + std::string(execution_id);
}

swarmkit::v1::ValidateTrajectoryResult TrajectoryExecutionManager::ValidatePlan(
    const swarmkit::v1::TrajectoryPlan& plan) const {
    swarmkit::v1::ValidateTrajectoryResult result;
    result.set_ok(false);
    result.set_first_failing_point_index(-1);

    if (config_ == nullptr) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "internal.config_missing",
                 "agent vehicle profile is unavailable");
        return result;
    }
    if (plan.execution_id().empty()) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "plan.execution_id_required",
                 "trajectory execution_id must not be empty");
    }
    if (plan.drone_id().empty()) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "plan.drone_id_required",
                 "trajectory drone_id must not be empty");
    }
    if (plan.points().empty()) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "plan.points_required",
                 "trajectory requires at least one point");
    }

    const auto& profile = config_->vehicle_profile;
    const auto& policy = plan.validation();
    const float max_horizontal_speed = EffectiveMaxHorizontalSpeed(profile, policy);
    const float max_climb_speed = EffectiveMaxClimbSpeed(profile, policy);
    const float max_descent_speed = EffectiveMaxDescentSpeed(profile, policy);
    const float max_altitude = EffectiveMaxAltitude(profile, policy);

    std::optional<core::TelemetryFrame> telemetry = LatestTelemetry(plan.drone_id());
    const float minimum_battery_percent = policy.min_battery_percent() > 0.0F
                                              ? policy.min_battery_percent()
                                              : profile.min_battery_percent;
    if (policy.require_gps() &&
        (!telemetry.has_value() || !telemetry->HasGpsQuality() ||
         telemetry->gps_fix_type < profile.min_gps_fix_type ||
         telemetry->satellites_visible < profile.min_satellites_visible ||
         !telemetry->validity.gps_hdop || telemetry->gps_hdop > profile.max_gps_hdop)) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "health.gps_required",
                 "validation policy requires GPS health within vehicle profile limits");
    }
    if (policy.require_ekf_ok() &&
        (!telemetry.has_value() || !telemetry->HasEstimatorState() || !telemetry->ekf_ok)) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "health.ekf_required",
                 "validation policy requires EKF healthy");
    }
    if (minimum_battery_percent > 0.0F && telemetry.has_value() &&
        telemetry->HasBattery() &&
        telemetry->battery_percent < minimum_battery_percent) {
        AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "battery.min",
                 "battery is below trajectory minimum");
    }

    std::int64_t previous_time_ms = -1;
    for (int index = 0; index < plan.points_size(); ++index) {
        const auto& point = plan.points(index);
        if (!HasDispatchTarget(point)) {
            AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "point.dispatch_required",
                     "trajectory point requires global/local position or timed command", index);
            continue;
        }
        const std::int64_t point_time =
            point.unix_time_ms() > 0 ? point.unix_time_ms() : point.time_offset_ms();
        if (point_time < 0) {
            AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "point.time_negative",
                     "trajectory point time must be >= 0", index);
        }
        if (previous_time_ms >= 0 && point_time <= previous_time_ms) {
            AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "point.time_order",
                     "trajectory point times must be strictly increasing", index);
        }
        previous_time_ms = point_time;

        if (point.has_position()) {
            const auto& pos = point.position();
            if (pos.lat_deg() < -90.0 || pos.lat_deg() > 90.0 || pos.lon_deg() < -180.0 ||
                pos.lon_deg() > 180.0) {
                AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "point.position_range",
                         "trajectory point latitude/longitude out of range", index);
            }
            if (point.position().alt_m() > max_altitude) {
                AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "point.altitude_max",
                         "trajectory point exceeds max altitude", index);
            }
            if (policy.has_geofence()) {
                const auto& fence = policy.geofence();
                const bool outside =
                    pos.lat_deg() < fence.min_lat_deg() || pos.lat_deg() > fence.max_lat_deg() ||
                    pos.lon_deg() < fence.min_lon_deg() || pos.lon_deg() > fence.max_lon_deg() ||
                    pos.alt_m() < fence.min_alt_m() || pos.alt_m() > fence.max_alt_m();
                if (outside) {
                    AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "geofence.outside",
                             "trajectory point is outside geofence", index);
                }
            }
        }

        if (index == 0) {
            continue;
        }
        const auto& previous = plan.points(index - 1);
        if (!HasPosition(point) || !HasPosition(previous)) {
            continue;
        }
        const std::int64_t prev_time =
            previous.unix_time_ms() > 0 ? previous.unix_time_ms() : previous.time_offset_ms();
        const double dt_s = static_cast<double>(point_time - prev_time) / 1000.0;
        if (dt_s <= 0.0) {
            continue;
        }
        const bool local_segment = point.use_local_position() || previous.use_local_position();
        const double horizontal_m =
            local_segment ? LocalDistanceMeters(previous.local_position(), point.local_position())
                          : DistanceMeters(previous.position(), point.position());
        const double vertical_m =
            point.has_position() ? point.position().alt_m() - previous.position().alt_m()
                                 : point.local_position().z_m() - previous.local_position().z_m();
        const auto horizontal_speed = static_cast<float>(horizontal_m / dt_s);
        const auto climb_speed = static_cast<float>(std::max(0.0, vertical_m) / dt_s);
        const auto descent_speed = static_cast<float>(std::max(0.0, -vertical_m) / dt_s);
        result.set_max_required_horizontal_speed_mps(
            std::max(result.max_required_horizontal_speed_mps(), horizontal_speed));
        result.set_max_required_climb_speed_mps(
            std::max(result.max_required_climb_speed_mps(), climb_speed));
        result.set_max_required_descent_speed_mps(
            std::max(result.max_required_descent_speed_mps(), descent_speed));
        if (horizontal_speed > max_horizontal_speed) {
            AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "speed.horizontal_max",
                     "trajectory segment exceeds max horizontal speed", index);
        }
        if (climb_speed > max_climb_speed) {
            AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "speed.climb_max",
                     "trajectory segment exceeds max climb speed", index);
        }
        if (descent_speed > max_descent_speed) {
            AddIssue(&result, swarmkit::v1::VALIDATION_ERROR, "speed.descent_max",
                     "trajectory segment exceeds max descent speed", index);
        }
    }

    result.set_ok(!HasValidationError(result));
    return result;
}

core::Result TrajectoryExecutionManager::Upload(
    swarmkit::v1::TrajectoryPlan plan, std::string_view correlation_id,
    swarmkit::v1::ExecutionHandle* out_handle,
    swarmkit::v1::ValidateTrajectoryResult* out_validation) {
    auto validation = ValidatePlan(plan);
    if (out_validation != nullptr) {
        *out_validation = validation;
    }
    if (plan.execution_id().empty() || plan.drone_id().empty()) {
        return core::Result::Rejected("trajectory plan requires execution_id and drone_id");
    }

    ExecutionRuntime runtime;
    runtime.plan = std::move(plan);
    runtime.validation = validation;
    runtime.stop = std::make_shared<std::atomic<bool>>(false);
    runtime.paused = std::make_shared<std::atomic<bool>>(false);
    runtime.handle.set_execution_id(runtime.plan.execution_id());
    runtime.handle.set_revision(runtime.plan.revision());
    runtime.handle.set_drone_id(runtime.plan.drone_id());
    runtime.handle.set_state(swarmkit::v1::EXECUTION_UPLOADED);
    runtime.handle.set_uploaded_unix_ms(NowUnixMs());
    runtime.handle.set_active_segment(0);
    runtime.handle.set_message(validation.ok() ? "trajectory uploaded"
                                               : "trajectory uploaded with validation errors");

    const std::string key = Key(runtime.plan.drone_id(), runtime.plan.execution_id());
    std::thread old_worker;
    std::shared_ptr<std::atomic<bool>> old_stop;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto iter = executions_.find(key); iter != executions_.end()) {
            old_stop = iter->second.stop;
            old_worker = std::move(iter->second.worker);
            executions_.erase(iter);
        }
        if (out_handle != nullptr) {
            *out_handle = runtime.handle;
        }
        executions_.emplace(key, std::move(runtime));
    }
    if (old_stop) {
        old_stop->store(true, std::memory_order_relaxed);
    }
    if (old_worker.joinable()) {
        old_worker.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto iter = executions_.find(key);
    if (iter != executions_.end()) {
        PublishReport(iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_UPLOADED,
                      validation.ok() ? swarmkit::v1::REPORT_INFO : swarmkit::v1::REPORT_WARNING, 0,
                      0.0, 0.0, 0, iter->second.handle.message(), correlation_id);
    }
    return core::Result::Success("trajectory uploaded");
}

core::Result TrajectoryExecutionManager::Clear(const std::string& drone_id,
                                               const std::string& execution_id,
                                               std::string_view correlation_id,
                                               swarmkit::v1::ExecutionHandle* out_handle) {
    std::thread old_worker;
    std::shared_ptr<std::atomic<bool>> old_stop;
    swarmkit::v1::TrajectoryPlan plan;
    swarmkit::v1::ExecutionHandle handle;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iter = executions_.find(Key(drone_id, execution_id));
        if (iter == executions_.end()) {
            return core::Result::Rejected("execution not found");
        }
        plan = iter->second.plan;
        handle = iter->second.handle;
        old_stop = iter->second.stop;
        old_worker = std::move(iter->second.worker);
        executions_.erase(iter);
    }
    if (old_stop) {
        old_stop->store(true, std::memory_order_relaxed);
    }
    if (old_worker.joinable()) {
        old_worker.join();
    }
    handle.set_state(swarmkit::v1::EXECUTION_ABORTED);
    handle.set_message("trajectory cleared");
    if (out_handle != nullptr) {
        *out_handle = handle;
    }
    PublishReport(plan, handle, swarmkit::v1::TRAJECTORY_ABORTED, swarmkit::v1::REPORT_INFO, 0, 0.0,
                  0.0, 0, "trajectory cleared", correlation_id);
    return core::Result::Success("trajectory cleared");
}

core::Result TrajectoryExecutionManager::Prepare(
    const std::string& drone_id, const std::string& execution_id, std::string_view correlation_id,
    swarmkit::v1::ExecutionHandle* out_handle,
    swarmkit::v1::ValidateTrajectoryResult* out_validation) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = executions_.find(Key(drone_id, execution_id));
    if (iter == executions_.end()) {
        return core::Result::Rejected("execution not found");
    }
    iter->second.validation = ValidatePlan(iter->second.plan);
    if (out_validation != nullptr) {
        *out_validation = iter->second.validation;
    }
    if (!iter->second.validation.ok()) {
        iter->second.handle.set_state(swarmkit::v1::EXECUTION_FAILED);
        iter->second.handle.set_message("trajectory validation failed");
        if (out_handle != nullptr) {
            *out_handle = iter->second.handle;
        }
        PublishReport(iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_FAILED,
                      swarmkit::v1::REPORT_ERROR, iter->second.handle.active_segment(), 0.0, 0.0, 0,
                      "trajectory validation failed", correlation_id);
        return core::Result::Rejected("trajectory validation failed");
    }
    iter->second.handle.set_state(swarmkit::v1::EXECUTION_READY);
    iter->second.handle.set_prepared_unix_ms(NowUnixMs());
    iter->second.handle.set_message("trajectory ready");
    if (out_handle != nullptr) {
        *out_handle = iter->second.handle;
    }
    PublishReport(iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_READY,
                  swarmkit::v1::REPORT_INFO, 0, 0.0, 0.0, 0, "trajectory ready", correlation_id);
    return core::Result::Success("trajectory ready");
}

core::Result TrajectoryExecutionManager::StartAt(const std::string& drone_id,
                                                 const std::string& execution_id,
                                                 std::int64_t unix_time_ms,
                                                 std::string_view correlation_id,
                                                 swarmkit::v1::ExecutionHandle* out_handle) {
    if (backend_ == nullptr || config_ == nullptr) {
        return core::Result::Failed("trajectory start safety state is unavailable");
    }
    if (const core::Result readiness =
            ValidateAutonomousReadiness(backend_->GetHealth(), "trajectory start",
                                        config_->safety.allow_unsafe_bench_commands);
        !readiness.IsOk()) {
        return readiness;
    }

    std::string key = Key(drone_id, execution_id);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = executions_.find(key);
        if (iter == executions_.end()) {
            return core::Result::Rejected("execution not found");
        }
        if (iter->second.handle.state() != swarmkit::v1::EXECUTION_READY &&
            iter->second.handle.state() != swarmkit::v1::EXECUTION_VALIDATED &&
            iter->second.handle.state() != swarmkit::v1::EXECUTION_UPLOADED) {
            return core::Result::Rejected("execution is not startable from current state");
        }
        if (!iter->second.validation.ok()) {
            iter->second.validation = ValidatePlan(iter->second.plan);
        }
        if (!iter->second.validation.ok()) {
            return core::Result::Rejected("execution cannot start: validation failed");
        }
        iter->second.stop->store(false, std::memory_order_relaxed);
        iter->second.paused->store(false, std::memory_order_relaxed);
        iter->second.handle.set_state(swarmkit::v1::EXECUTION_STARTED);
        iter->second.handle.set_start_unix_ms(unix_time_ms > 0 ? unix_time_ms : NowUnixMs());
        iter->second.handle.set_message("trajectory started");
        if (out_handle != nullptr) {
            *out_handle = iter->second.handle;
        }
        if (iter->second.worker.joinable()) {
            return core::Result::Rejected("execution worker already running");
        }
        iter->second.worker = std::thread([this, key, correlation = std::string(correlation_id)] {
            try {
                RunExecution(key, correlation);
            } catch (...) {
                static_cast<void>(std::fputs("Trajectory execution worker failed\n", stderr));
            }
        });
        PublishReport(iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_STARTED,
                      swarmkit::v1::REPORT_INFO, 0, 0.0, 0.0, 0, "trajectory started",
                      correlation_id);
    }
    return core::Result::Success("trajectory started");
}

core::Result TrajectoryExecutionManager::Pause(const std::string& drone_id,
                                               const std::string& execution_id,
                                               std::string_view correlation_id,
                                               swarmkit::v1::ExecutionHandle* out_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = executions_.find(Key(drone_id, execution_id));
    if (iter == executions_.end()) {
        return core::Result::Rejected("execution not found");
    }
    iter->second.paused->store(true, std::memory_order_relaxed);
    iter->second.handle.set_state(swarmkit::v1::EXECUTION_PAUSED);
    iter->second.handle.set_message("trajectory paused");
    if (out_handle != nullptr) {
        *out_handle = iter->second.handle;
    }
    PublishReport(iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_TRACKING,
                  swarmkit::v1::REPORT_INFO, iter->second.handle.active_segment(), 0.0, 0.0, 0,
                  "trajectory paused", correlation_id);
    return core::Result::Success("trajectory paused");
}

core::Result TrajectoryExecutionManager::Resume(const std::string& drone_id,
                                                const std::string& execution_id,
                                                std::string_view correlation_id,
                                                swarmkit::v1::ExecutionHandle* out_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = executions_.find(Key(drone_id, execution_id));
    if (iter == executions_.end()) {
        return core::Result::Rejected("execution not found");
    }
    iter->second.paused->store(false, std::memory_order_relaxed);
    iter->second.handle.set_state(swarmkit::v1::EXECUTION_STARTED);
    iter->second.handle.set_message("trajectory resumed");
    if (out_handle != nullptr) {
        *out_handle = iter->second.handle;
    }
    PublishReport(iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_TRACKING,
                  swarmkit::v1::REPORT_INFO, iter->second.handle.active_segment(), 0.0, 0.0, 0,
                  "trajectory resumed", correlation_id);
    return core::Result::Success("trajectory resumed");
}

core::Result TrajectoryExecutionManager::Abort(const std::string& drone_id,
                                               const std::string& execution_id,
                                               std::string_view correlation_id,
                                               swarmkit::v1::ExecutionHandle* out_handle) {
    std::thread old_worker;
    std::shared_ptr<std::atomic<bool>> old_stop;
    swarmkit::v1::TrajectoryPlan plan;
    swarmkit::v1::ExecutionHandle handle;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = executions_.find(Key(drone_id, execution_id));
        if (iter == executions_.end()) {
            return core::Result::Rejected("execution not found");
        }
        iter->second.stop->store(true, std::memory_order_relaxed);
        iter->second.handle.set_state(swarmkit::v1::EXECUTION_ABORTED);
        iter->second.handle.set_message("trajectory aborted");
        plan = iter->second.plan;
        handle = iter->second.handle;
        old_stop = iter->second.stop;
        old_worker = std::move(iter->second.worker);
        if (out_handle != nullptr) {
            *out_handle = iter->second.handle;
        }
    }
    if (old_stop) {
        old_stop->store(true, std::memory_order_relaxed);
    }
    if (old_worker.joinable()) {
        old_worker.join();
    }
    PublishReport(plan, handle, swarmkit::v1::TRAJECTORY_ABORTED, swarmkit::v1::REPORT_WARNING,
                  handle.active_segment(), 0.0, 0.0, 0, "trajectory aborted", correlation_id);
    return core::Result::Success("trajectory aborted");
}

std::optional<std::pair<swarmkit::v1::ExecutionHandle, swarmkit::v1::TrajectoryPlan>>
TrajectoryExecutionManager::Get(const std::string& drone_id,
                                const std::string& execution_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iter = executions_.find(Key(drone_id, execution_id));
    if (iter == executions_.end()) {
        return std::nullopt;
    }
    return std::make_pair(iter->second.handle, iter->second.plan);
}

std::vector<swarmkit::v1::ExecutionHandle> TrajectoryExecutionManager::List(
    const std::string& drone_id) const {
    std::vector<swarmkit::v1::ExecutionHandle> handles;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [_, runtime] : executions_) {
        if (drone_id.empty() || drone_id == "all" || runtime.plan.drone_id() == drone_id) {
            handles.push_back(runtime.handle);
        }
    }
    return handles;
}

swarmkit::v1::TimeSyncState TrajectoryExecutionManager::GetTimeSyncState(
    const std::string& drone_id) const {
    if (backend_ == nullptr) {
        return MakeDegradedTimeSyncState(drone_id, "backend-unavailable",
                                         "backend is unavailable; time synchronization cannot be "
                                         "verified");
    }

    const BackendCapabilities capabilities = backend_->GetCapabilities();
    if (!capabilities.supports_time_sync) {
        return MakeDegradedTimeSyncState(
            drone_id, "agent-clock-fallback",
            "backend does not support vehicle time synchronization; agent clock is not sufficient "
            "for synchronized execution");
    }

    BackendTimeSyncState state = backend_->GetTimeSyncState(drone_id);
    if (state.drone_id.empty()) {
        state.drone_id = ResolvedDroneId(drone_id);
    }
    return ToProtoTimeSyncState(state);
}

void TrajectoryExecutionManager::Shutdown() {
    std::vector<std::thread> workers;
    std::vector<std::shared_ptr<std::atomic<bool>>> stops;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [_, runtime] : executions_) {
            stops.push_back(runtime.stop);
            workers.push_back(std::move(runtime.worker));
        }
        executions_.clear();
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

std::optional<core::TelemetryFrame> TrajectoryExecutionManager::LatestTelemetry(
    const std::string& drone_id) const {
    if (telemetry_ == nullptr || config_ == nullptr) {
        return std::nullopt;
    }
    TelemetryLease lease;
    if (const core::Result result = telemetry_->AcquireLease(
            drone_id, std::max(1, config_->default_telemetry_rate_hz), &lease);
        !result.IsOk()) {
        return std::nullopt;
    }
    core::TelemetryFrame frame;
    std::uint64_t sequence = 0;
    const bool got_frame =
        TelemetryManager::WaitForFrame(lease, &sequence, &frame, kTelemetryHealthSampleTimeout);
    telemetry_->ReleaseLease(lease);
    if (!got_frame) {
        return std::nullopt;
    }
    return frame;
}

void TrajectoryExecutionManager::PublishReport(
    const swarmkit::v1::TrajectoryPlan& plan, const swarmkit::v1::ExecutionHandle& /*unused*/,
    swarmkit::v1::TrajectoryReportStatus status, swarmkit::v1::ReportSeverity severity,
    int active_segment, double distance_to_target_m, double drift_m, std::int64_t schedule_error_ms,
    std::string_view message, std::string_view correlation_id) {
    if (reports_ == nullptr) {
        return;
    }
    swarmkit::v1::AgentReport report;
    report.set_drone_id(plan.drone_id());
    report.set_correlation_id(std::string(correlation_id));
    report.set_type(swarmkit::v1::TRAJECTORY_REPORT);
    report.set_severity(severity);
    report.set_message(std::string(message));
    auto* trajectory = report.mutable_trajectory();
    trajectory->set_drone_id(plan.drone_id());
    trajectory->set_execution_id(plan.execution_id());
    trajectory->set_revision(plan.revision());
    trajectory->set_status(status);
    trajectory->set_active_segment(active_segment);
    trajectory->set_distance_to_target_m(distance_to_target_m);
    trajectory->set_drift_m(drift_m);
    trajectory->set_schedule_error_ms(schedule_error_ms);
    trajectory->set_message(std::string(message));
    reports_->Publish(std::move(report));
}

core::Result TrajectoryExecutionManager::SendTrajectoryPoint(
    const swarmkit::v1::TrajectoryPlan& plan, const swarmkit::v1::TrajectoryPoint& point,
    std::string_view correlation_id) const {
    if (backend_ == nullptr) {
        return core::Result::Failed("backend unavailable");
    }
    if (point.has_command()) {
        auto command = ConvertTimedProtoCommand(point.command());
        if (!command.has_value()) {
            return command.error();
        }
        commands::CommandEnvelope envelope;
        envelope.context.drone_id = plan.drone_id();
        if (const auto iter = plan.labels().find("swarmkit.context.client_id");
            iter != plan.labels().end() && !iter->second.empty()) {
            envelope.context.client_id = iter->second;
        } else {
            envelope.context.client_id = "swarmkit-agent-execution";
        }
        envelope.context.priority = PriorityFromPlanLabels(plan.labels());
        envelope.context.correlation_id = std::string(correlation_id);
        envelope.command = std::move(*command);
        return backend_->Execute(envelope);
    }
    if (point.use_local_position()) {
        return core::Result::Rejected("local trajectory execution requires backend-native support");
    }
    commands::CommandEnvelope envelope;
    envelope.context.drone_id = plan.drone_id();
    envelope.context.client_id = "swarmkit-agent-execution";
    envelope.context.priority = commands::CommandPriority::kSupervisor;
    envelope.context.correlation_id = std::string(correlation_id);
    envelope.command = commands::NavCmd{commands::CmdGoto{
        .lat_deg = point.position().lat_deg(),
        .lon_deg = point.position().lon_deg(),
        .alt_m = point.position().alt_m(),
        .speed_mps = point.has_velocity() ? std::hypot(point.vx_mps(), point.vy_mps())
                                          : config_->vehicle_profile.cruise_speed_mps,
        .yaw_deg = point.yaw_deg(),
        .use_yaw = point.has_yaw(),
    }};
    return backend_->Execute(envelope);
}

core::Result TrajectoryExecutionManager::SendPayloadAction(
    const swarmkit::v1::TrajectoryPlan& plan, const swarmkit::v1::PayloadAction& action,
    std::string_view correlation_id) const {
    if (backend_ == nullptr) {
        return core::Result::Failed("backend unavailable");
    }
    commands::CmdBackendCommand command;
    command.backend_namespace = action.action_namespace();
    command.name = action.name();
    for (const auto& [key, value] : action.params()) {
        command.params.emplace(key, value);
    }
    commands::CommandEnvelope envelope;
    envelope.context.drone_id = plan.drone_id();
    envelope.context.client_id = "swarmkit-agent-execution";
    envelope.context.priority = commands::CommandPriority::kSupervisor;
    envelope.context.correlation_id = std::string(correlation_id);
    envelope.command = commands::BackendCmd{std::move(command)};
    return backend_->Execute(envelope);
}

std::int64_t TrajectoryExecutionManager::ComputeTrajectoryReachTimeoutMs(double distance_m) const {
    if (config_ == nullptr) {
        return agent::kDefaultGoalMarginMs;
    }
    const float speed_mps =
        std::max(config_->vehicle_profile.cruise_speed_mps, kMinimumTrajectorySpeedMps);
    const auto travel_ms = static_cast<std::int64_t>((distance_m / speed_mps) * 1000.0);
    const std::int64_t timeout_ms =
        travel_ms + static_cast<std::int64_t>(config_->vehicle_profile.goal_margin_ms);
    return std::clamp(timeout_ms,
                      static_cast<std::int64_t>(config_->vehicle_profile.goal_margin_ms),
                      static_cast<std::int64_t>(config_->vehicle_profile.max_goal_timeout_ms));
}

void TrajectoryExecutionManager::RunExecution(const std::string& key,
                                              const std::string& correlation_id) {
    TelemetryLease lease;
    std::uint64_t last_sequence = 0;
    bool telemetry_active = false;
    if (telemetry_ != nullptr && config_ != nullptr) {
        swarmkit::v1::TrajectoryPlan plan_snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (auto iter = executions_.find(key); iter != executions_.end()) {
                plan_snapshot = iter->second.plan;
            }
        }
        if (!plan_snapshot.drone_id().empty()) {
            telemetry_active =
                telemetry_
                    ->AcquireLease(plan_snapshot.drone_id(),
                                   std::max(1, config_->default_telemetry_rate_hz), &lease)
                    .IsOk();
        }
    }

    std::size_t next_payload_index = 0;
    while (true) {
        swarmkit::v1::TrajectoryPlan plan;
        swarmkit::v1::ExecutionHandle handle;
        std::shared_ptr<std::atomic<bool>> stop;
        std::shared_ptr<std::atomic<bool>> paused;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto iter = executions_.find(key);
            if (iter == executions_.end()) {
                break;
            }
            plan = iter->second.plan;
            handle = iter->second.handle;
            stop = iter->second.stop;
            paused = iter->second.paused;
        }

        if (!stop || stop->load(std::memory_order_relaxed)) {
            break;
        }

        bool completed = true;
        for (int index = handle.active_segment(); index < plan.points_size(); ++index) {
            if (stop->load(std::memory_order_relaxed)) {
                completed = false;
                break;
            }
            while (paused && paused->load(std::memory_order_relaxed) &&
                   !stop->load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(std::chrono::milliseconds{100});
            }
            const auto& point = plan.points(index);
            const std::int64_t scheduled_ms = PointScheduleMs(point, handle.start_unix_ms());
            while (NowUnixMs() < scheduled_ms && !stop->load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(std::chrono::milliseconds{20});
            }
            if (stop->load(std::memory_order_relaxed)) {
                completed = false;
                break;
            }
            const std::int64_t schedule_error_ms = NowUnixMs() - scheduled_ms;
            if (schedule_error_ms > kLateThresholdMs) {
                PublishReport(plan, handle, swarmkit::v1::TRAJECTORY_LATE,
                              swarmkit::v1::REPORT_WARNING, index, 0.0, 0.0, schedule_error_ms,
                              "trajectory point dispatched late", correlation_id);
            }
            if (const core::Result send_result = SendTrajectoryPoint(plan, point, correlation_id);
                !send_result.IsOk()) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (auto iter = executions_.find(key); iter != executions_.end()) {
                    iter->second.handle.set_state(swarmkit::v1::EXECUTION_FAILED);
                    iter->second.handle.set_active_segment(index);
                    iter->second.handle.set_message(send_result.message);
                    PublishReport(iter->second.plan, iter->second.handle,
                                  swarmkit::v1::TRAJECTORY_FAILED, swarmkit::v1::REPORT_ERROR,
                                  index, 0.0, 0.0, schedule_error_ms, send_result.message,
                                  correlation_id);
                }
                completed = false;
                break;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (auto iter = executions_.find(key); iter != executions_.end()) {
                    iter->second.handle.set_active_segment(index);
                    iter->second.handle.set_message("trajectory tracking");
                    handle = iter->second.handle;
                }
            }

            while (next_payload_index < static_cast<std::size_t>(plan.payload_timeline_size())) {
                const auto& action = plan.payload_timeline(static_cast<int>(next_payload_index));
                const std::int64_t due_ms = action.unix_time_ms() > 0
                                                ? action.unix_time_ms()
                                                : handle.start_unix_ms() + action.time_offset_ms();
                if (due_ms > NowUnixMs()) {
                    break;
                }
                if (action.has_action()) {
                    static_cast<void>(SendPayloadAction(plan, action.action(), correlation_id));
                }
                ++next_payload_index;
            }

            if (telemetry_active) {
                core::TelemetryFrame frame;
                if (TelemetryManager::WaitForFrame(lease, &last_sequence, &frame,
                                                   kTelemetryWaitTimeout)) {
                    const double distance =
                        point.has_position() && frame.HasPosition()
                            ? DistanceMeters(frame.lat_deg, frame.lon_deg,
                                             point.position().lat_deg(), point.position().lon_deg())
                            : 0.0;
                    const float tolerance = plan.validation().tracking_tolerance_m() > 0.0F
                                                ? plan.validation().tracking_tolerance_m()
                                                : kDefaultTrackingToleranceM;
                    PublishReport(
                        plan, handle,
                        distance > tolerance ? swarmkit::v1::TRAJECTORY_DRIFTING
                                             : swarmkit::v1::TRAJECTORY_TRACKING,
                        distance > tolerance ? swarmkit::v1::REPORT_WARNING
                                             : swarmkit::v1::REPORT_INFO,
                        index, distance, distance, schedule_error_ms,
                        distance > tolerance ? "trajectory drifting" : "trajectory tracking",
                        correlation_id);
                }
            }
        }
        if (completed && telemetry_active && plan.points_size() > 0 &&
            plan.points(plan.points_size() - 1).has_position()) {
            const auto& final_point = plan.points(plan.points_size() - 1);
            const float tolerance = plan.validation().tracking_tolerance_m() > 0.0F
                                        ? plan.validation().tracking_tolerance_m()
                                        : kDefaultTrackingToleranceM;
            bool reached = false;
            bool deadline_initialized = false;
            std::int64_t deadline_ms =
                NowUnixMs() + static_cast<std::int64_t>(agent::kDefaultMaxGoalTimeoutMs);
            std::int64_t next_report_ms = 0;
            double last_distance_m = 0.0;

            while (!stop->load(std::memory_order_relaxed)) {
                core::TelemetryFrame frame;
                if (!TelemetryManager::WaitForFrame(lease, &last_sequence, &frame,
                                                    kTelemetryWaitTimeout)) {
                    if (NowUnixMs() > deadline_ms) {
                        break;
                    }
                    continue;
                }

                last_distance_m = DistanceToPointMeters(frame, final_point);
                if (!deadline_initialized) {
                    deadline_ms = NowUnixMs() + ComputeTrajectoryReachTimeoutMs(last_distance_m);
                    deadline_initialized = true;
                }
                if (last_distance_m <= tolerance) {
                    reached = true;
                    PublishReport(plan, handle, swarmkit::v1::TRAJECTORY_TRACKING,
                                  swarmkit::v1::REPORT_INFO, handle.active_segment(),
                                  last_distance_m, last_distance_m, 0,
                                  "trajectory final target reached", correlation_id);
                    break;
                }

                const std::int64_t now_ms = NowUnixMs();
                if (now_ms >= next_report_ms) {
                    PublishReport(plan, handle, swarmkit::v1::TRAJECTORY_DRIFTING,
                                  swarmkit::v1::REPORT_WARNING, handle.active_segment(),
                                  last_distance_m, last_distance_m, 0,
                                  "trajectory waiting for final target", correlation_id);
                    next_report_ms = now_ms + kFinalTargetReportInterval.count();
                }
                if (now_ms > deadline_ms) {
                    break;
                }
            }

            if (!reached) {
                completed = false;
                std::lock_guard<std::mutex> lock(mutex_);
                if (auto iter = executions_.find(key); iter != executions_.end()) {
                    iter->second.handle.set_state(swarmkit::v1::EXECUTION_FAILED);
                    iter->second.handle.set_message("trajectory final target not reached");
                    PublishReport(
                        iter->second.plan, iter->second.handle, swarmkit::v1::TRAJECTORY_FAILED,
                        swarmkit::v1::REPORT_ERROR, iter->second.handle.active_segment(),
                        last_distance_m, last_distance_m, 0,
                        "trajectory final target not reached before timeout", correlation_id);
                }
            }
        }

        if (completed) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (auto iter = executions_.find(key); iter != executions_.end()) {
                iter->second.handle.set_state(swarmkit::v1::EXECUTION_COMPLETED);
                iter->second.handle.set_message("trajectory completed");
                PublishReport(iter->second.plan, iter->second.handle,
                              swarmkit::v1::TRAJECTORY_COMPLETED, swarmkit::v1::REPORT_INFO,
                              iter->second.handle.active_segment(), 0.0, 0.0, 0,
                              "trajectory completed", correlation_id);
            }
        }
        break;
    }

    if (telemetry_active) {
        telemetry_->ReleaseLease(lease);
    }
}

}  // namespace swarmkit::agent::internal
