// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_telemetry_decoder.h"

namespace swarmkit::agent::mavlink {

MavlinkTelemetryDecodeResult MavlinkTelemetryDecoder::Decode(
    const mavlink_message_t& message, TelemetryCache* telemetry_cache,
    MavlinkStateCache* state_cache) {
    MavlinkTelemetryDecodeResult result;
    if (telemetry_cache == nullptr || state_cache == nullptr) {
        return result;
    }

    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            telemetry_cache->mode = ModeString(heartbeat);
            telemetry_cache->armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0U;
            telemetry_cache->failsafe = heartbeat.system_status == MAV_STATE_CRITICAL ||
                                        heartbeat.system_status == MAV_STATE_EMERGENCY ||
                                        heartbeat.system_status == MAV_STATE_FLIGHT_TERMINATION;
            telemetry_cache->landed = heartbeat.system_status == MAV_STATE_STANDBY;
            state_cache->UpdateHeartbeat(message, heartbeat);
            result.should_publish = true;
            if (!message_intervals_requested_) {
                message_intervals_requested_ = true;
                result.should_request_intervals = true;
            }
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t position{};
            mavlink_msg_global_position_int_decode(&message, &position);
            telemetry_cache->lat_deg = static_cast<double>(position.lat) / kDegE7;
            telemetry_cache->lon_deg = static_cast<double>(position.lon) / kDegE7;
            telemetry_cache->abs_alt_m =
                static_cast<float>(position.alt) / kMillimetresPerMetre;
            telemetry_cache->rel_alt_m =
                static_cast<float>(position.relative_alt) / kMillimetresPerMetre;
            telemetry_cache->vx_mps = static_cast<float>(position.vx) / kCentimetresPerMetre;
            telemetry_cache->vy_mps = static_cast<float>(position.vy) / kCentimetresPerMetre;
            telemetry_cache->vz_mps = static_cast<float>(position.vz) / kCentimetresPerMetre;
            state_cache->UpdateTelemetry(message);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status{};
            mavlink_msg_sys_status_decode(&message, &sys_status);
            if (sys_status.battery_remaining >= 0) {
                telemetry_cache->battery_percent = static_cast<float>(sys_status.battery_remaining);
            }
            state_cache->UpdateTelemetry(message);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS: {
            mavlink_battery_status_t battery{};
            mavlink_msg_battery_status_decode(&message, &battery);
            if (battery.battery_remaining >= 0) {
                telemetry_cache->battery_percent = static_cast<float>(battery.battery_remaining);
            }
            state_cache->UpdateTelemetry(message);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps{};
            mavlink_msg_gps_raw_int_decode(&message, &gps);
            telemetry_cache->gps_fix_type = gps.fix_type;
            telemetry_cache->satellites_visible = gps.satellites_visible;
            telemetry_cache->gps_hdop = static_cast<float>(gps.eph) / kCentimetresPerMetre;
            state_cache->UpdateTelemetry(message);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude{};
            mavlink_msg_attitude_decode(&message, &attitude);
            telemetry_cache->roll_deg = attitude.roll * kRadiansToDegrees;
            telemetry_cache->pitch_deg = attitude.pitch * kRadiansToDegrees;
            telemetry_cache->yaw_deg = attitude.yaw * kRadiansToDegrees;
            state_cache->UpdateTelemetry(message);
            result.should_publish = true;
            break;
        }
        default:
            break;
    }
    return result;
}

}  // namespace swarmkit::agent::mavlink
