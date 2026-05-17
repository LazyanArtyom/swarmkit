// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_telemetry_decoder.h"

#include <limits>

namespace swarmkit::agent::mavlink {
namespace {

constexpr std::uint16_t kInvalidGpsEph = UINT16_MAX;
constexpr std::uint32_t kInvalidAccuracy = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint64_t kUnixEpochUsecThreshold = 1000000000000000ULL;

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

void ApplyGpsSourceTime(std::uint64_t time_usec, TelemetryCache* telemetry_cache) {
    if (telemetry_cache == nullptr || time_usec == 0U) {
        return;
    }
    if (time_usec >= kUnixEpochUsecThreshold) {
        telemetry_cache->source_unix_time_ms =
            static_cast<std::int64_t>(time_usec / 1000ULL);
    } else {
        telemetry_cache->source_time_boot_ms =
            static_cast<std::int64_t>(time_usec / 1000ULL);
    }
}

[[nodiscard]] core::GpsQuality GpsQualityFromFixType(std::uint8_t fix_type) {
    switch (fix_type) {
        case GPS_FIX_TYPE_NO_GPS:
        case GPS_FIX_TYPE_NO_FIX:
            return core::GpsQuality::kNoFix;
        case GPS_FIX_TYPE_2D_FIX:
            return core::GpsQuality::kFix2D;
        case GPS_FIX_TYPE_3D_FIX:
            return core::GpsQuality::kFix3D;
        case GPS_FIX_TYPE_DGPS:
            return core::GpsQuality::kDgps;
        case GPS_FIX_TYPE_RTK_FLOAT:
            return core::GpsQuality::kRtkFloat;
        case GPS_FIX_TYPE_RTK_FIXED:
            return core::GpsQuality::kRtkFixed;
        case GPS_FIX_TYPE_STATIC:
            return core::GpsQuality::kStatic;
        case GPS_FIX_TYPE_PPP:
            return core::GpsQuality::kPpp;
        default:
            return core::GpsQuality::kUnknown;
    }
}

void ApplyGpsAccuracy(const mavlink_gps_raw_int_t& gps, TelemetryCache* telemetry_cache) {
    if (telemetry_cache == nullptr) {
        return;
    }
    if (gps.h_acc > 0U && gps.h_acc < kInvalidAccuracy) {
        telemetry_cache->accuracy.horizontal_position_valid = true;
        telemetry_cache->accuracy.horizontal_position_m =
            static_cast<float>(gps.h_acc) / kMillimetresPerMetre;
    }
    if (gps.v_acc > 0U && gps.v_acc < kInvalidAccuracy) {
        telemetry_cache->accuracy.vertical_position_valid = true;
        telemetry_cache->accuracy.vertical_position_m =
            static_cast<float>(gps.v_acc) / kMillimetresPerMetre;
    }
    if (gps.vel_acc > 0U && gps.vel_acc < kInvalidAccuracy) {
        telemetry_cache->accuracy.velocity_valid = true;
        telemetry_cache->accuracy.velocity_mps =
            static_cast<float>(gps.vel_acc) / kMillimetresPerMetre;
    }
    if (gps.hdg_acc > 0U && gps.hdg_acc < kInvalidAccuracy) {
        telemetry_cache->accuracy.heading_valid = true;
        telemetry_cache->accuracy.heading_deg = static_cast<float>(gps.hdg_acc) / 100000.0F;
    }
}

void ApplyCommonEstimatorState(std::uint16_t flags, TelemetryCache* telemetry_cache) {
    if (telemetry_cache == nullptr) {
        return;
    }
    telemetry_cache->validity.estimator = true;
    telemetry_cache->estimator_flags = flags;
    telemetry_cache->estimator_attitude_ok = (flags & ESTIMATOR_ATTITUDE) != 0U;
    telemetry_cache->estimator_velocity_ok =
        (flags & static_cast<std::uint16_t>(ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_VELOCITY_VERT)) != 0U;
    telemetry_cache->estimator_position_ok =
        (flags & static_cast<std::uint16_t>(ESTIMATOR_POS_HORIZ_REL |
                                            ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_POS_VERT_AGL)) != 0U;
    const bool fault =
        (flags & static_cast<std::uint16_t>(ESTIMATOR_GPS_GLITCH | ESTIMATOR_ACCEL_ERROR)) != 0U;
    if (fault) {
        telemetry_cache->estimator_state = core::EstimatorState::kFault;
    } else if (EstimateCommonEstimatorOk(flags)) {
        telemetry_cache->estimator_state = core::EstimatorState::kHealthy;
    } else if (flags == 0U) {
        telemetry_cache->estimator_state = core::EstimatorState::kInitializing;
    } else {
        telemetry_cache->estimator_state = core::EstimatorState::kDegraded;
    }
}

void ApplyArdupilotEkfState(std::uint16_t flags, TelemetryCache* telemetry_cache) {
    if (telemetry_cache == nullptr) {
        return;
    }
    telemetry_cache->validity.estimator = true;
    telemetry_cache->estimator_flags = flags;
    telemetry_cache->estimator_attitude_ok = (flags & EKF_ATTITUDE) != 0U;
    telemetry_cache->estimator_velocity_ok =
        (flags & static_cast<std::uint16_t>(EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT)) != 0U;
    telemetry_cache->estimator_position_ok =
        (flags & static_cast<std::uint16_t>(EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS |
                                            EKF_POS_VERT_ABS | EKF_POS_VERT_AGL)) != 0U;
    const bool fault =
        (flags & static_cast<std::uint16_t>(EKF_GPS_GLITCHING | EKF_CONST_POS_MODE)) != 0U;
    if ((flags & EKF_UNINITIALIZED) != 0U || flags == 0U) {
        telemetry_cache->estimator_state = core::EstimatorState::kInitializing;
    } else if (fault) {
        telemetry_cache->estimator_state = core::EstimatorState::kFault;
    } else if (EstimateArdupilotEkfOk(flags)) {
        telemetry_cache->estimator_state = core::EstimatorState::kHealthy;
    } else {
        telemetry_cache->estimator_state = core::EstimatorState::kDegraded;
    }
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
            telemetry_cache->validity.mode = true;
            telemetry_cache->validity.armed = true;
            telemetry_cache->validity.failsafe = true;
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
            telemetry_cache->source_time_boot_ms = position.time_boot_ms;
            telemetry_cache->position_frame = core::CoordinateFrame::kWgs84;
            telemetry_cache->velocity_frame = core::CoordinateFrame::kLocalNed;
            telemetry_cache->validity.position = true;
            telemetry_cache->validity.relative_altitude = true;
            telemetry_cache->validity.absolute_altitude = true;
            telemetry_cache->validity.velocity = true;
            state_cache->UpdateGlobalPosition(message, position);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status{};
            mavlink_msg_sys_status_decode(&message, &sys_status);
            if (sys_status.battery_remaining >= 0) {
                telemetry_cache->battery_percent = static_cast<float>(sys_status.battery_remaining);
                telemetry_cache->validity.battery = true;
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
                telemetry_cache->validity.battery = true;
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
            telemetry_cache->gps_quality = GpsQualityFromFixType(gps.fix_type);
            telemetry_cache->validity.gps = true;
            telemetry_cache->gps_hdop =
                gps.eph == kInvalidGpsEph ? 0.0F
                                          : static_cast<float>(gps.eph) / kCentimetresPerMetre;
            telemetry_cache->validity.gps_hdop = gps.eph != kInvalidGpsEph;
            ApplyGpsSourceTime(gps.time_usec, telemetry_cache);
            ApplyGpsAccuracy(gps, telemetry_cache);
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
            if (sys_state.landed_state != MAV_LANDED_STATE_UNDEFINED) {
                telemetry_cache->validity.landed = true;
            }
            state_cache->UpdateExtendedSysState(message, sys_state);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
            mavlink_estimator_status_t estimator{};
            mavlink_msg_estimator_status_decode(&message, &estimator);
            telemetry_cache->ekf_ok = EstimateCommonEstimatorOk(estimator.flags);
            ApplyCommonEstimatorState(estimator.flags, telemetry_cache);
            state_cache->UpdateEstimatorStatus(message, estimator);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t ekf{};
            mavlink_msg_ekf_status_report_decode(&message, &ekf);
            telemetry_cache->ekf_ok = EstimateArdupilotEkfOk(ekf.flags);
            ApplyArdupilotEkfState(ekf.flags, telemetry_cache);
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
            telemetry_cache->source_time_boot_ms = attitude.time_boot_ms;
            telemetry_cache->validity.attitude = true;
            state_cache->UpdateTelemetry(message);
            result.should_publish = true;
            break;
        }
        case MAVLINK_MSG_ID_HOME_POSITION: {
            mavlink_home_position_t home{};
            mavlink_msg_home_position_decode(&message, &home);
            telemetry_cache->home_origin.frame = core::CoordinateFrame::kWgs84;
            telemetry_cache->home_origin.lat_deg = static_cast<double>(home.latitude) / kDegE7;
            telemetry_cache->home_origin.lon_deg = static_cast<double>(home.longitude) / kDegE7;
            telemetry_cache->home_origin.alt_m =
                static_cast<float>(home.altitude) / kMillimetresPerMetre;
            telemetry_cache->home_origin.north_m = home.x;
            telemetry_cache->home_origin.east_m = home.y;
            telemetry_cache->home_origin.down_m = home.z;
            telemetry_cache->validity.home_origin = true;
            ApplyGpsSourceTime(home.time_usec, telemetry_cache);
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
