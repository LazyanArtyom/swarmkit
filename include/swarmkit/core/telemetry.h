// Copyright (c) 2026 Artyom Lazyan. All rights reserved.
// SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
//
// This file is part of SwarmKit.
// See LICENSE.md in the repository root for full license terms.

#pragma once

#include <cstdint>
#include <string>

namespace swarmkit::core {

/// Single snapshot of drone telemetry data.
struct TelemetryFrame {
    std::string drone_id;         ///< Unique identifier of the reporting drone.
    std::int64_t unix_time_ms{};  ///< UTC timestamp in milliseconds since epoch.

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

    /// Default equality -- useful in tests.
    bool operator==(const TelemetryFrame&) const = default;
};

}  // namespace swarmkit::core
