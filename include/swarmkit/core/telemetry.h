// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <array>
#include <cstdint>
#include <string>

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

/// Optional measurement accuracy and covariance metadata.
struct TelemetryAccuracy {
    bool horizontal_position_valid{false};
    float horizontal_position_m{};
    bool vertical_position_valid{false};
    float vertical_position_m{};
    bool velocity_valid{false};
    float velocity_mps{};
    bool heading_valid{false};
    float heading_deg{};
    bool attitude_valid{false};
    float attitude_deg{};
    bool position_covariance_valid{false};
    std::array<float, 9> position_covariance{};
    bool velocity_covariance_valid{false};
    std::array<float, 9> velocity_covariance{};

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

    /// SDK/agent receive timestamp in milliseconds since Unix epoch.
    std::int64_t unix_time_ms{};

    /// Vehicle/source timestamp in milliseconds since Unix epoch when available.
    std::int64_t source_unix_time_ms{};

    /// Vehicle/source monotonic boot timestamp in milliseconds when available.
    std::int64_t source_time_boot_ms{};

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
    bool ekf_ok{true};
    int gps_fix_type{};           ///< Backend-specific fix quality; 0 = unknown/no fix.
    int satellites_visible{};     ///< Number of tracked/visible satellites.
    float gps_hdop{};             ///< Horizontal dilution / accuracy proxy.
    float link_quality_percent{}; ///< Radio/transport quality when backend can report it.
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
    std::string active_command_id;
    std::string active_goal_id;
    std::string correlation_id;

    [[nodiscard]] bool HasPosition() const { return validity.position; }
    [[nodiscard]] bool HasRelativeAltitude() const { return validity.relative_altitude; }
    [[nodiscard]] bool HasAbsoluteAltitude() const { return validity.absolute_altitude; }
    [[nodiscard]] bool HasVelocity() const { return validity.velocity; }
    [[nodiscard]] bool HasAttitude() const { return validity.attitude; }
    [[nodiscard]] bool HasBattery() const { return validity.battery; }
    [[nodiscard]] bool HasGpsQuality() const { return validity.gps; }
    [[nodiscard]] bool HasEstimatorState() const { return validity.estimator; }

    /// Default equality -- useful in tests.
    bool operator==(const TelemetryFrame&) const = default;
};

}  // namespace swarmkit::core
