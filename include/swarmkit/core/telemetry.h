#pragma once

#include <cstdint>
#include <string>

namespace swarmkit::core {

/// Single snapshot of drone telemetry data.
struct TelemetryFrame {
    std::string drone_id;            ///< Unique identifier of the reporting drone.
    std::int64_t unix_time_ms{};     ///< UTC timestamp in milliseconds since epoch.

    double lat_deg{};                ///< Latitude in degrees.
    double lon_deg{};                ///< Longitude in degrees.
    float rel_alt_m{};               ///< Relative altitude in metres.
    float battery_percent{};         ///< Remaining battery as a percentage (0-100).
    std::string mode;                ///< Current flight mode string.

    /// Default equality -- useful in tests.
    bool operator==(const TelemetryFrame&) const = default;
};

} // namespace swarmkit::core
