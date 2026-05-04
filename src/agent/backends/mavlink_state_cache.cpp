// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_state_cache.h"

#include <string>

namespace swarmkit::agent::mavlink {

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

MavlinkVehicleState MavlinkStateCache::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

BackendHealth MavlinkStateCache::Health() const {
    const std::int64_t now_ms = NowUnixMs();
    const MavlinkVehicleState state = Snapshot();

    BackendHealth health;
    health.last_heartbeat_unix_ms = state.last_heartbeat_unix_ms;
    health.last_telemetry_unix_ms = state.last_telemetry_unix_ms;
    health.armed = state.armed;
    health.custom_mode = state.custom_mode;
    health.failsafe = state.failsafe;

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
                     " custom_mode=" + std::to_string(state.custom_mode);
    if (state.last_telemetry_unix_ms != 0 &&
        now_ms - state.last_telemetry_unix_ms > kTelemetryStaleTimeoutMs) {
        health.message += " telemetry=stale";
    }
    return health;
}

}  // namespace swarmkit::agent::mavlink
