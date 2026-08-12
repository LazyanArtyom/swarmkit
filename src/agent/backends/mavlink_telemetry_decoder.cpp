// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#include "mavlink_telemetry_decoder.h"

#include <algorithm>
#include <chrono>
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
           (flags & static_cast<std::uint16_t>(ESTIMATOR_GPS_GLITCH | ESTIMATOR_ACCEL_ERROR)) == 0U;
}

[[nodiscard]] core::TimestampEvidence GpsSourceTime(std::uint64_t time_usec) {
    if (time_usec == 0U) {
        return {};
    }
    if (time_usec >= kUnixEpochUsecThreshold) {
        return {
            .timestamp_ms = static_cast<std::int64_t>(time_usec / 1000ULL),
            .clock_domain = core::ClockDomain::kUnixEpoch,
            .synchronization = core::ClockSynchronization::kUnknown,
        };
    }
    return {
        .timestamp_ms = static_cast<std::int64_t>(time_usec / 1000ULL),
        .clock_domain = core::ClockDomain::kVehicleBoot,
        .synchronization = core::ClockSynchronization::kUnsynchronized,
    };
}

[[nodiscard]] core::TimestampEvidence VehicleBootTime(std::uint32_t time_boot_ms) {
    if (time_boot_ms == 0U) {
        return {};
    }
    return {
        .timestamp_ms = static_cast<std::int64_t>(time_boot_ms),
        .clock_domain = core::ClockDomain::kVehicleBoot,
        .synchronization = core::ClockSynchronization::kUnsynchronized,
    };
}

void MarkUpdated(core::MeasurementProvenance* provenance, std::string source,
                 core::TimestampEvidence source_time = {}) {
    if (provenance == nullptr) {
        return;
    }
    provenance->updated = true;
    provenance->source = std::move(source);
    provenance->source_time = source_time;
}

void MergeMeasurement(const core::MeasurementProvenance& update,
                      core::MeasurementProvenance* pending) {
    if (pending != nullptr && update.updated) {
        *pending = update;
    }
}

void MergeProvenance(const core::TelemetryProvenance& update, core::TelemetryProvenance* pending) {
    if (pending == nullptr) {
        return;
    }
    MergeMeasurement(update.position, &pending->position);
    MergeMeasurement(update.velocity, &pending->velocity);
    MergeMeasurement(update.accuracy, &pending->accuracy);
    MergeMeasurement(update.estimator, &pending->estimator);
    MergeMeasurement(update.vehicle_state, &pending->vehicle_state);
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
        telemetry_cache->accuracy.horizontal_position = core::UncertaintyEstimate{
            .value = static_cast<float>(gps.h_acc) / kMillimetresPerMetre,
            .descriptor =
                {
                    .semantics = core::UncertaintySemantics::kBackendSpecific,
                    .source = "mavlink.GPS_RAW_INT.h_acc",
                },
        };
    }
    if (gps.v_acc > 0U && gps.v_acc < kInvalidAccuracy) {
        telemetry_cache->accuracy.vertical_position = core::UncertaintyEstimate{
            .value = static_cast<float>(gps.v_acc) / kMillimetresPerMetre,
            .descriptor =
                {
                    .semantics = core::UncertaintySemantics::kBackendSpecific,
                    .source = "mavlink.GPS_RAW_INT.v_acc",
                },
        };
    }
    if (gps.vel_acc > 0U && gps.vel_acc < kInvalidAccuracy) {
        telemetry_cache->accuracy.speed = core::UncertaintyEstimate{
            .value = static_cast<float>(gps.vel_acc) / kMillimetresPerMetre,
            .descriptor =
                {
                    .semantics = core::UncertaintySemantics::kBackendSpecific,
                    .source = "mavlink.GPS_RAW_INT.vel_acc",
                },
        };
    }
    if (gps.hdg_acc > 0U && gps.hdg_acc < kInvalidAccuracy) {
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
        (flags & static_cast<std::uint16_t>(ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT)) !=
        0U;
    telemetry_cache->estimator_position_ok =
        (flags & static_cast<std::uint16_t>(ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_POS_VERT_ABS | ESTIMATOR_POS_VERT_AGL)) != 0U;
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

MavlinkTelemetryCoalescer::MavlinkTelemetryCoalescer(int rate_hz) {
    Reset(rate_hz);
}

void MavlinkTelemetryCoalescer::Reset(int rate_hz) {
    rate_hz_ = std::max(1, rate_hz);
    last_publish_time_.reset();
    pending_ = {};
}

std::optional<core::TelemetryProvenance> MavlinkTelemetryCoalescer::Push(
    const core::TelemetryProvenance& update, std::chrono::steady_clock::time_point now) {
    MergeProvenance(update, &pending_);
    const auto period = std::chrono::nanoseconds{std::chrono::nanoseconds::period::den / rate_hz_};
    if (last_publish_time_.has_value() && now - *last_publish_time_ < period) {
        return std::nullopt;
    }

    last_publish_time_ = now;
    core::TelemetryProvenance ready = std::move(pending_);
    pending_ = {};
    return ready;
}

MavlinkTelemetryDecodeResult MavlinkTelemetryDecoder::Decode(const mavlink_message_t& message,
                                                             TelemetryCache* telemetry_cache,
                                                             MavlinkStateCache* state_cache,
                                                             MavlinkAutopilotProfile profile) {
    MavlinkTelemetryDecodeResult result;
    if (telemetry_cache == nullptr || state_cache == nullptr) {
        return result;
    }

    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            telemetry_cache->mode = ModeString(heartbeat, profile);
            telemetry_cache->armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0U;
            telemetry_cache->failsafe = heartbeat.system_status == MAV_STATE_CRITICAL ||
                                        heartbeat.system_status == MAV_STATE_EMERGENCY ||
                                        heartbeat.system_status == MAV_STATE_FLIGHT_TERMINATION;
            telemetry_cache->validity.mode = true;
            telemetry_cache->validity.armed = true;
            telemetry_cache->validity.failsafe = true;
            state_cache->UpdateHeartbeat(message, heartbeat);
            result.should_publish = true;
            MarkUpdated(&result.provenance.vehicle_state, "mavlink.HEARTBEAT");
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
            telemetry_cache->abs_alt_m = static_cast<float>(position.alt) / kMillimetresPerMetre;
            telemetry_cache->rel_alt_m =
                static_cast<float>(position.relative_alt) / kMillimetresPerMetre;
            telemetry_cache->vx_mps = static_cast<float>(position.vx) / kCentimetresPerMetre;
            telemetry_cache->vy_mps = static_cast<float>(position.vy) / kCentimetresPerMetre;
            telemetry_cache->vz_mps = static_cast<float>(position.vz) / kCentimetresPerMetre;
            telemetry_cache->position_frame = core::CoordinateFrame::kWgs84;
            telemetry_cache->velocity_frame = core::CoordinateFrame::kLocalNed;
            telemetry_cache->validity.position = true;
            telemetry_cache->validity.relative_altitude = true;
            telemetry_cache->validity.absolute_altitude = true;
            telemetry_cache->validity.velocity = true;
            state_cache->UpdateGlobalPosition(message, position);
            result.should_publish = true;
            const core::TimestampEvidence source_time = VehicleBootTime(position.time_boot_ms);
            MarkUpdated(&result.provenance.position, "mavlink.GLOBAL_POSITION_INT", source_time);
            MarkUpdated(&result.provenance.velocity, "mavlink.GLOBAL_POSITION_INT", source_time);
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
            telemetry_cache->gps_hdop = gps.eph == kInvalidGpsEph
                                            ? 0.0F
                                            : static_cast<float>(gps.eph) / kCentimetresPerMetre;
            telemetry_cache->validity.gps_hdop = gps.eph != kInvalidGpsEph;
            ApplyGpsAccuracy(gps, telemetry_cache);
            state_cache->UpdateGps(message, gps);
            result.should_publish = true;
            MarkUpdated(&result.provenance.accuracy, "mavlink.GPS_RAW_INT",
                        GpsSourceTime(gps.time_usec));
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
            MarkUpdated(&result.provenance.vehicle_state, "mavlink.EXTENDED_SYS_STATE");
            break;
        }
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
            mavlink_estimator_status_t estimator{};
            mavlink_msg_estimator_status_decode(&message, &estimator);
            ApplyCommonEstimatorState(estimator.flags, telemetry_cache);
            state_cache->UpdateEstimatorStatus(message, estimator);
            result.should_publish = true;
            MarkUpdated(&result.provenance.estimator, "mavlink.ESTIMATOR_STATUS",
                        GpsSourceTime(estimator.time_usec));
            break;
        }
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
            mavlink_ekf_status_report_t ekf{};
            mavlink_msg_ekf_status_report_decode(&message, &ekf);
            ApplyArdupilotEkfState(ekf.flags, telemetry_cache);
            state_cache->UpdateEkfStatus(message, ekf);
            result.should_publish = true;
            MarkUpdated(&result.provenance.estimator, "mavlink.EKF_STATUS_REPORT");
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude{};
            mavlink_msg_attitude_decode(&message, &attitude);
            telemetry_cache->roll_deg = attitude.roll * kRadiansToDegrees;
            telemetry_cache->pitch_deg = attitude.pitch * kRadiansToDegrees;
            telemetry_cache->yaw_deg = attitude.yaw * kRadiansToDegrees;
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
