// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "swarmkit/core/execution.h"

namespace swarmkit::core {

/// Coordinate frame used by position, velocity, and origin fields.
enum class CoordinateFrame : std::uint8_t {
    kUnknown,
    kWgs84,
    kLocalNed,
    kBodyNed,
};

/// Normalized GPS quality. Raw backend fix values remain available separately.
enum class GpsQuality : std::uint8_t {
    kUnknown,
    kNoFix,
    kFix2D,
    kFix3D,
    kDgps,
    kRtkFloat,
    kRtkFixed,
    kStatic,
    kPpp,
};

/// Normalized estimator/ EKF state suitable for monitoring UI decisions.
enum class EstimatorState : std::uint8_t {
    kUnknown,
    kInitializing,
    kHealthy,
    kDegraded,
    kFault,
};

enum class ClockDomain : std::uint8_t {
    kUnknown,
    kUnixEpoch,
    kVehicleBoot,
    kAgentMonotonic,
    kSdkMonotonic,
};

enum class ClockSynchronization : std::uint8_t {
    kUnknown,
    kUnsynchronized,
    kEstimated,
    kSynchronized,
};

/// Timestamp value plus enough metadata to avoid treating unknown clock error
/// as zero. timestamp_ms and clock_uncertainty_ms use optional presence.
struct TimestampEvidence {
    std::optional<std::int64_t> timestamp_ms;
    ClockDomain clock_domain{ClockDomain::kUnknown};
    ClockSynchronization synchronization{ClockSynchronization::kUnknown};
    std::optional<double> clock_uncertainty_ms;

    bool operator==(const TimestampEvidence&) const = default;
};

/// Freshness evidence for one independently updated measurement group.
struct MeasurementProvenance {
    bool updated{false};
    std::uint64_t generation{};
    TimestampEvidence source_time;
    std::optional<std::int64_t> agent_receive_unix_time_ms;
    std::optional<std::int64_t> agent_receive_monotonic_time_ns;
    std::string source;

    bool operator==(const MeasurementProvenance&) const = default;
};

struct TelemetryProvenance {
    MeasurementProvenance position;
    MeasurementProvenance velocity;
    MeasurementProvenance accuracy;
    MeasurementProvenance estimator;
    MeasurementProvenance vehicle_state;

    bool operator==(const TelemetryProvenance&) const = default;
};

/// Explicit validity flags for scalar telemetry fields.
///
/// A numeric value in TelemetryFrame is meaningful only when the matching flag
/// is true. This avoids treating a default zero as a real measurement.
struct TelemetryValidityFlags {
    bool position{false};
    bool relative_altitude{false};
    bool absolute_altitude{false};
    bool velocity{false};
    bool attitude{false};
    bool battery{false};
    bool mode{false};
    bool armed{false};
    bool landed{false};
    bool failsafe{false};
    bool gps{false};
    bool gps_hdop{false};
    bool link_quality{false};
    bool estimator{false};
    bool home_origin{false};

    bool operator==(const TelemetryValidityFlags&) const = default;
};

enum class UncertaintySemantics : std::uint8_t {
    kUnknown,
    kStandardDeviation,
    kConfidenceBound,
    kEmpiricallyCalibratedBound,
    kDeterministicHardBound,
    kBackendSpecific,
};

struct UncertaintyDescriptor {
    UncertaintySemantics semantics{UncertaintySemantics::kUnknown};
    std::optional<double> confidence_level;
    std::string calibration_profile_id;
    std::string calibration_version;
    std::string source;
    std::uint64_t measurement_generation{};

    bool operator==(const UncertaintyDescriptor&) const = default;
};

struct UncertaintyEstimate {
    float value{};
    UncertaintyDescriptor descriptor;

    bool operator==(const UncertaintyEstimate&) const = default;
};

/// Optional measurement accuracy and covariance metadata.
struct TelemetryAccuracy {
    std::optional<UncertaintyEstimate> horizontal_position;
    std::optional<UncertaintyEstimate> vertical_position;
    std::optional<UncertaintyEstimate> horizontal_velocity;
    std::optional<UncertaintyEstimate> vertical_velocity;
    std::optional<UncertaintyEstimate> speed;
    std::optional<float> heading_deg;
    std::optional<float> attitude_deg;
    std::optional<std::array<float, 9>> position_covariance;
    std::optional<std::array<float, 9>> velocity_covariance;

    bool operator==(const TelemetryAccuracy&) const = default;
};

/// Vehicle home/takeoff origin when reported by the backend.
struct HomeOrigin {
    CoordinateFrame frame{CoordinateFrame::kWgs84};
    double lat_deg{};
    double lon_deg{};
    float alt_m{};
    float north_m{};
    float east_m{};
    float down_m{};

    bool operator==(const HomeOrigin&) const = default;
};

/// Single snapshot of drone telemetry data.
struct TelemetryFrame {
    std::string drone_id;  ///< Unique identifier of the reporting drone.

    /// Agent process lifetime and normalized producer identity.
    std::string agent_session_id;
    std::uint64_t telemetry_sequence{};

    /// Agent ingress times are part of the normalized evidence contract.
    std::int64_t agent_receive_unix_time_ms{};
    std::int64_t agent_receive_monotonic_time_ns{};

    double lat_deg{};         ///< Latitude in degrees.
    double lon_deg{};         ///< Longitude in degrees.
    float rel_alt_m{};        ///< Relative altitude in metres.
    float abs_alt_m{};        ///< Absolute/MSL altitude in metres when available.
    float vx_mps{};           ///< North/local X velocity in m/s when available.
    float vy_mps{};           ///< East/local Y velocity in m/s when available.
    float vz_mps{};           ///< Down/local Z velocity in m/s when available.
    float roll_deg{};         ///< Vehicle roll angle in degrees when available.
    float pitch_deg{};        ///< Vehicle pitch angle in degrees when available.
    float yaw_deg{};          ///< Vehicle yaw/heading angle in degrees when available.
    float battery_percent{};  ///< Remaining battery as a percentage (0-100).
    std::string mode;         ///< Current flight mode string.
    bool armed{false};
    bool landed{false};
    bool failsafe{false};
    int gps_fix_type{};            ///< Backend-specific fix quality; 0 = unknown/no fix.
    int satellites_visible{};      ///< Number of tracked/visible satellites.
    float gps_hdop{};              ///< Horizontal dilution / accuracy proxy.
    float link_quality_percent{};  ///< Radio/transport quality when backend can report it.
    CoordinateFrame position_frame{CoordinateFrame::kUnknown};
    CoordinateFrame velocity_frame{CoordinateFrame::kUnknown};
    TelemetryValidityFlags validity;
    TelemetryAccuracy accuracy;
    HomeOrigin home_origin;
    GpsQuality gps_quality{GpsQuality::kUnknown};
    EstimatorState estimator_state{EstimatorState::kUnknown};
    std::uint32_t estimator_flags{};
    bool estimator_position_ok{false};
    bool estimator_velocity_ok{false};
    bool estimator_attitude_ok{false};
    std::optional<ExecutionHandle> execution_handle;
    TelemetryProvenance provenance;

    [[nodiscard]] bool HasPosition() const {
        return validity.position;
    }
    [[nodiscard]] bool HasRelativeAltitude() const {
        return validity.relative_altitude;
    }
    [[nodiscard]] bool HasAbsoluteAltitude() const {
        return validity.absolute_altitude;
    }
    [[nodiscard]] bool HasVelocity() const {
        return validity.velocity;
    }
    [[nodiscard]] bool HasAttitude() const {
        return validity.attitude;
    }
    [[nodiscard]] bool HasBattery() const {
        return validity.battery;
    }
    [[nodiscard]] bool HasGpsQuality() const {
        return validity.gps;
    }
    [[nodiscard]] bool HasEstimatorState() const {
        return validity.estimator;
    }

    /// Default equality -- useful in tests.
    bool operator==(const TelemetryFrame&) const = default;
};

}  // namespace swarmkit::core
