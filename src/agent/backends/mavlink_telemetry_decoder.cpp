// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_telemetry_decoder.h"

namespace swarmkit::agent::mavlink {
namespace {

constexpr std::uint16_t kInvalidGpsEph = UINT16_MAX;

[[nodiscard]] bool EstimateArdupilotEkfOk(std::uint16_t flags) {
    constexpr auto required =
        static_cast<std::uint16_t>(EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_REL);
    return (flags & required) == required &&
           (flags & static_cast<std::uint16_t>(EKF_UNINITIALIZED | EKF_GPS_GLITCHING)) == 0U;
}

[[nodiscard]] bool EstimateCommonEstimatorOk(std::uint16_t flags) {
    constexpr auto required = static_cast<std::uint16_t>(
        ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_POS_HORIZ_REL);
    return (flags & required) == required &&
           (flags & static_cast<std::uint16_t>(ESTIMATOR_GPS_GLITCH | ESTIMATOR_ACCEL_ERROR)) ==
               0U;
}

}  // namespace

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
            state_cache->UpdateGlobalPosition(message, position);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status{};
            mavlink_msg_sys_status_decode(&message, &sys_status);
            if (sys_status.battery_remaining >= 0) {
                telemetry_cache->battery_percent = static_cast<float>(sys_status.battery_remaining);
            }
            state_cache->UpdateSysStatus(message, sys_status);
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
            telemetry_cache->gps_hdop =
                gps.eph == kInvalidGpsEph ? 0.0F
                                          : static_cast<float>(gps.eph) / kCentimetresPerMetre;
            state_cache->UpdateGps(message, gps);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
            mavlink_extended_sys_state_t sys_state{};
            mavlink_msg_extended_sys_state_decode(&message, &sys_state);
            switch (sys_state.landed_state) {
                case MAV_LANDED_STATE_ON_GROUND:
                    telemetry_cache->landed = true;
                    break;
                case MAV_LANDED_STATE_IN_AIR:
                case MAV_LANDED_STATE_TAKEOFF:
                case MAV_LANDED_STATE_LANDING:
                    telemetry_cache->landed = false;
                    break;
                case MAV_LANDED_STATE_UNDEFINED:
                default:
                    break;
            }
            state_cache->UpdateExtendedSysState(message, sys_state);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
            mavlink_estimator_status_t estimator{};
            mavlink_msg_estimator_status_decode(&message, &estimator);
            telemetry_cache->ekf_ok = EstimateCommonEstimatorOk(estimator.flags);
            state_cache->UpdateEstimatorStatus(message, estimator);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t ekf{};
            mavlink_msg_ekf_status_report_decode(&message, &ekf);
            telemetry_cache->ekf_ok = EstimateArdupilotEkfOk(ekf.flags);
            state_cache->UpdateEkfStatus(message, ekf);
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
