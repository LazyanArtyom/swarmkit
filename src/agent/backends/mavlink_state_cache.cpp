// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_state_cache.h"

#include <string>

namespace swarmkit::agent::mavlink {
namespace {

constexpr int kMinimumUsableGpsFixType = 3;
constexpr std::uint16_t kInvalidGpsEph = UINT16_MAX;
constexpr auto kArdupilotEkfRequiredFlags =
    static_cast<std::uint16_t>(EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_REL);
constexpr auto kPx4EstimatorRequiredFlags = static_cast<std::uint16_t>(
    ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_POS_HORIZ_REL);

[[nodiscard]] bool HasAllFlags(std::uint16_t flags, std::uint16_t required) {
    return (flags & required) == required;
}

[[nodiscard]] bool HasAnyFlag(std::uint16_t flags, std::uint16_t candidate) {
    return (flags & candidate) != 0U;
}

}  // namespace

void MavlinkStateCache::UpdateHeartbeat(const mavlink_message_t& message,
                                        const mavlink_heartbeat_t& heartbeat) {
    const bool failsafe = heartbeat.system_status == MAV_STATE_CRITICAL ||
                          heartbeat.system_status == MAV_STATE_EMERGENCY ||
                          heartbeat.system_status == MAV_STATE_FLIGHT_TERMINATION;

    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_heartbeat_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.mav_type = heartbeat.type;
    state_.autopilot = heartbeat.autopilot;
    state_.base_mode = heartbeat.base_mode;
    state_.system_status = heartbeat.system_status;
    state_.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0U;
    state_.custom_mode = static_cast<int>(heartbeat.custom_mode);
    state_.failsafe = failsafe;
}

void MavlinkStateCache::UpdateTelemetry(const mavlink_message_t& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
}

void MavlinkStateCache::UpdateGlobalPosition(const mavlink_message_t& message,
                                             const mavlink_global_position_int_t& position) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.relative_alt_m = static_cast<float>(position.relative_alt) / kMillimetresPerMetre;
    state_.has_relative_altitude = true;
}

void MavlinkStateCache::UpdateGps(const mavlink_message_t& message,
                                  const mavlink_gps_raw_int_t& gps) {
    const bool has_hdop = gps.eph != kInvalidGpsEph;
    const bool gps_ok = gps.fix_type >= kMinimumUsableGpsFixType &&
                        gps.satellites_visible > 0U && has_hdop;

    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.gps_seen = true;
    state_.gps_ok = gps_ok;
    state_.gps_fix_type = gps.fix_type;
    state_.satellites_visible = gps.satellites_visible;
    state_.gps_hdop = has_hdop ? static_cast<float>(gps.eph) / kCentimetresPerMetre : 0.0F;
}

void MavlinkStateCache::UpdateSysStatus(const mavlink_message_t& message,
                                        const mavlink_sys_status_t& sys_status) {
    const bool gps_enabled =
        (sys_status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_GPS) != 0U;
    const bool gps_healthy =
        (sys_status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_GPS) != 0U;

    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    if (gps_enabled && !gps_healthy) {
        state_.gps_seen = true;
        state_.gps_ok = false;
    }
}

void MavlinkStateCache::UpdateExtendedSysState(const mavlink_message_t& message,
                                               const mavlink_extended_sys_state_t& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;

    switch (state.landed_state) {
        case MAV_LANDED_STATE_ON_GROUND:
            state_.landed = true;
            state_.landed_known = true;
            break;
        case MAV_LANDED_STATE_IN_AIR:
        case MAV_LANDED_STATE_TAKEOFF:
        case MAV_LANDED_STATE_LANDING:
            state_.landed = false;
            state_.landed_known = true;
            break;
        case MAV_LANDED_STATE_UNDEFINED:
        default:
            state_.landed_known = false;
            break;
    }
}

void MavlinkStateCache::UpdateEstimatorStatus(const mavlink_message_t& message,
                                              const mavlink_estimator_status_t& estimator) {
    const bool healthy = HasAllFlags(estimator.flags, kPx4EstimatorRequiredFlags) &&
                         !HasAnyFlag(estimator.flags,
                                     ESTIMATOR_GPS_GLITCH | ESTIMATOR_ACCEL_ERROR);

    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.ekf_seen = true;
    state_.ekf_ok = healthy;
}

void MavlinkStateCache::UpdateEkfStatus(const mavlink_message_t& message,
                                        const mavlink_ekf_status_report_t& ekf) {
    const bool healthy = HasAllFlags(ekf.flags, kArdupilotEkfRequiredFlags) &&
                         !HasAnyFlag(ekf.flags, EKF_UNINITIALIZED | EKF_GPS_GLITCHING);

    std::lock_guard<std::mutex> lock(mutex_);
    state_.last_telemetry_unix_ms = NowUnixMs();
    state_.system_id = message.sysid;
    state_.component_id = message.compid;
    state_.ekf_seen = true;
    state_.ekf_ok = healthy;
}

MavlinkVehicleState MavlinkStateCache::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

BackendHealth MavlinkStateCache::Health() const {
    const std::int64_t now_ms = NowUnixMs();
    const MavlinkVehicleState state = Snapshot();

    BackendHealth health;
    health.backend_name = "mavlink";
    health.protocol = "mavlink";
    health.last_heartbeat_unix_ms = state.last_heartbeat_unix_ms;
    health.last_telemetry_unix_ms = state.last_telemetry_unix_ms;
    health.armed = state.armed;
    health.landed = state.landed_known ? state.landed : !state.armed;
    health.custom_mode = state.custom_mode;
    health.failsafe = state.failsafe;
    health.gps_ok = state.gps_seen && state.gps_ok;
    health.gps_fix_type = state.gps_fix_type;
    health.satellites_visible = state.satellites_visible;
    health.gps_hdop = state.gps_hdop;
    health.ekf_ok = state.ekf_seen ? state.ekf_ok : !state.failsafe;
    health.has_relative_altitude = state.has_relative_altitude;
    health.relative_alt_m = state.relative_alt_m;

    if (state.last_heartbeat_unix_ms == 0) {
        health.ready = false;
        health.message = "MAVLink heartbeat not received";
        return health;
    }
    if (now_ms - state.last_heartbeat_unix_ms > kHeartbeatStaleTimeoutMs) {
        health.ready = false;
        health.message = "MAVLink heartbeat stale";
        return health;
    }
    if (state.failsafe) {
        health.ready = false;
        health.message = "MAVLink system status is failsafe/emergency";
        return health;
    }

    health.ready = true;
    health.message = "MAVLink ready sysid=" + std::to_string(state.system_id) +
                     " compid=" + std::to_string(state.component_id) +
                     " armed=" + (state.armed ? "true" : "false") +
                     " landed=" + (health.landed ? "true" : "false") +
                     " gps=" + (health.gps_ok ? "ok" : "bad") +
                     " gps_fix=" + std::to_string(state.gps_fix_type) +
                     " sats=" + std::to_string(state.satellites_visible) +
                     " hdop=" + std::to_string(state.gps_hdop) +
                     " ekf=" + (health.ekf_ok ? "ok" : "bad") +
                     " custom_mode=" + std::to_string(state.custom_mode);
    if (state.last_telemetry_unix_ms != 0 &&
        now_ms - state.last_telemetry_unix_ms > kTelemetryStaleTimeoutMs) {
        health.message += " telemetry=stale";
    }
    return health;
}

}  // namespace swarmkit::agent::mavlink
